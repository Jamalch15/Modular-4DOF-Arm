from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, replace
from datetime import datetime, timezone
from math import isfinite, sqrt
from typing import Any
from uuid import uuid4

import numpy as np

from .cartesian_calibration import calibration_context, profile_freshness, sample_coverage
from .config import DHRowConfig, LinkConfig, RobotConfig
from .kinematics import forward_kinematics


PARAMETER_GROUPS: dict[str, list[str]] = {
    "joint_zeros": ["joint_1_zero_deg", "joint_2_zero_deg", "joint_3_zero_deg", "joint_4_zero_deg"],
    "geometry_basic": ["base_height_mm", "base_side_offset_mm", "upper_arm_mm", "forearm_mm"],
    "joint_zeros_geometry": [
        "joint_1_zero_deg",
        "joint_2_zero_deg",
        "joint_3_zero_deg",
        "joint_4_zero_deg",
        "base_height_mm",
        "base_side_offset_mm",
        "upper_arm_mm",
        "forearm_mm",
    ],
}

PARAMETER_META: dict[str, dict[str, float | str]] = {
    "joint_1_zero_deg": {"step": 0.05, "scale": 2.0, "bound": 8.0, "unit": "deg"},
    "joint_2_zero_deg": {"step": 0.05, "scale": 2.0, "bound": 8.0, "unit": "deg"},
    "joint_3_zero_deg": {"step": 0.05, "scale": 2.0, "bound": 8.0, "unit": "deg"},
    "joint_4_zero_deg": {"step": 0.05, "scale": 2.0, "bound": 8.0, "unit": "deg"},
    "base_height_mm": {"step": 0.1, "scale": 8.0, "bound": 30.0, "unit": "mm"},
    "base_side_offset_mm": {"step": 0.1, "scale": 8.0, "bound": 25.0, "unit": "mm"},
    "upper_arm_mm": {"step": 0.1, "scale": 8.0, "bound": 25.0, "unit": "mm"},
    "forearm_mm": {"step": 0.1, "scale": 8.0, "bound": 25.0, "unit": "mm"},
}


def _sample_vectors(samples: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    joints = np.asarray([sample["reported_joints_deg"] for sample in samples], dtype=float)
    measured = np.asarray(
        [
            [
                sample["measured"]["x_mm"],
                sample["measured"]["y_mm"],
                sample["measured"]["z_mm"],
            ]
            for sample in samples
        ],
        dtype=float,
    )
    quality = np.asarray([max(0.05, float(sample.get("quality", 1.0))) for sample in samples], dtype=float)
    if joints.shape != (len(samples), 4) or measured.shape != (len(samples), 3):
        raise ValueError("physical-model samples contain invalid joint or measured TCP vectors")
    if not np.all(np.isfinite(joints)) or not np.all(np.isfinite(measured)):
        raise ValueError("physical-model samples contain non-finite values")
    return joints, measured, quality


def _links_with_deltas(config: RobotConfig, names: list[str], values: np.ndarray) -> LinkConfig:
    deltas = dict(zip(names, [float(value) for value in values], strict=True))
    rows: list[DHRowConfig] = []
    for index, row in enumerate(config.links.dh_rows):
        rows.append(
            replace(
                row,
                zero_offset_deg=float(row.zero_offset_deg) + float(deltas.get(f"joint_{index + 1}_zero_deg", 0.0)),
                d_mm=(
                    float(row.d_mm) + float(deltas.get("base_height_mm", 0.0))
                    if index == 0
                    else float(row.d_mm)
                ),
                a_mm=(
                    float(row.a_mm) + float(deltas.get("upper_arm_mm", 0.0))
                    if index == 1
                    else float(row.a_mm) + float(deltas.get("forearm_mm", 0.0))
                    if index == 2
                    else float(row.a_mm)
                ),
            )
        )
    return replace(
        config.links,
        base_height_mm=float(config.links.base_height_mm) + float(deltas.get("base_height_mm", 0.0)),
        base_side_offset_mm=float(config.links.base_side_offset_mm) + float(deltas.get("base_side_offset_mm", 0.0)),
        upper_arm_mm=float(config.links.upper_arm_mm) + float(deltas.get("upper_arm_mm", 0.0)),
        forearm_mm=float(config.links.forearm_mm) + float(deltas.get("forearm_mm", 0.0)),
        dh_rows=rows,
    )


def _predicted_positions(
    config: RobotConfig,
    names: list[str],
    values: np.ndarray,
    joints: np.ndarray,
) -> np.ndarray:
    links = _links_with_deltas(config, names, values)
    return np.asarray(
        [
            [
                (fk := forward_kinematics([float(value) for value in angles], links))["x_mm"],
                fk["y_mm"],
                fk["z_mm"],
            ]
            for angles in joints
        ],
        dtype=float,
    )


def _jacobian(
    config: RobotConfig,
    names: list[str],
    values: np.ndarray,
    joints: np.ndarray,
) -> np.ndarray:
    base = _predicted_positions(config, names, values, joints).reshape(-1)
    columns: list[np.ndarray] = []
    for index, name in enumerate(names):
        shifted = values.copy()
        step = float(PARAMETER_META[name]["step"])
        shifted[index] += step
        columns.append((_predicted_positions(config, names, shifted, joints).reshape(-1) - base) / step)
    return np.column_stack(columns)


def _observability(jacobian: np.ndarray, names: list[str]) -> dict[str, Any]:
    scales = np.asarray([float(PARAMETER_META[name]["scale"]) for name in names], dtype=float)
    normalized = jacobian * scales[None, :]
    singular_values = np.linalg.svd(normalized, compute_uv=False)
    rank = int(np.linalg.matrix_rank(normalized, tol=max(normalized.shape) * np.finfo(float).eps * singular_values[0]))
    condition = float(singular_values[0] / singular_values[-1]) if singular_values[-1] > 1e-12 else float("inf")
    return {
        "rank": rank,
        "parameter_count": len(names),
        "condition_number": condition,
        "singular_values": singular_values.tolist(),
        "identifiable": rank == len(names) and isfinite(condition) and condition <= 2.0e4,
    }


def _metrics(vectors: np.ndarray) -> dict[str, Any]:
    if vectors.size == 0:
        return {
            "count": 0,
            "xy_rmse_mm": None,
            "xy_max_mm": None,
            "z_rmse_mm": None,
            "z_max_abs_mm": None,
            "xyz_rmse_mm": None,
        }
    xy = np.linalg.norm(vectors[:, :2], axis=1)
    z = np.abs(vectors[:, 2])
    xyz = np.linalg.norm(vectors, axis=1)
    return {
        "count": int(len(vectors)),
        "xy_rmse_mm": float(sqrt(float(np.mean(xy**2)))),
        "xy_max_mm": float(np.max(xy)),
        "z_rmse_mm": float(sqrt(float(np.mean(z**2)))),
        "z_max_abs_mm": float(np.max(z)),
        "xyz_rmse_mm": float(sqrt(float(np.mean(xyz**2)))),
    }


def _passes(metrics: dict[str, Any], thresholds: dict[str, Any]) -> bool:
    return bool(
        metrics.get("count")
        and float(metrics["xy_rmse_mm"]) <= float(thresholds.get("good_xy_rmse_mm", 5.0))
        and float(metrics["xy_max_mm"]) <= float(thresholds.get("acceptable_xy_max_mm", 10.0))
        and float(metrics["z_rmse_mm"]) <= float(thresholds.get("good_z_rmse_mm", 3.0))
        and float(metrics["z_max_abs_mm"]) <= float(thresholds.get("acceptable_z_max_mm", 5.0))
    )


def _fit_values(
    config: RobotConfig,
    names: list[str],
    samples: list[dict[str, Any]],
) -> tuple[np.ndarray, dict[str, Any]]:
    joints, measured, quality = _sample_vectors(samples)
    values = np.zeros(len(names), dtype=float)
    bounds = np.asarray([float(PARAMETER_META[name]["bound"]) for name in names], dtype=float)
    scales = np.asarray([float(PARAMETER_META[name]["scale"]) for name in names], dtype=float)
    damping = 1e-2
    best_cost = float("inf")
    best_values = values.copy()
    iterations = 0
    for iterations in range(1, 61):
        predicted = _predicted_positions(config, names, values, joints)
        residual = (predicted - measured).reshape(-1)
        sample_norm = np.linalg.norm((predicted - measured), axis=1)
        median = float(np.median(sample_norm))
        mad = float(np.median(np.abs(sample_norm - median)))
        robust_limit = max(3.0, median + 3.5 * 1.4826 * mad)
        robust = np.minimum(1.0, robust_limit / np.maximum(sample_norm, 1e-9))
        weights = np.repeat(np.sqrt(quality * robust), 3)
        weighted_residual = residual * weights
        jacobian = _jacobian(config, names, values, joints)
        weighted_jacobian = jacobian * weights[:, None]
        prior = np.diag(1.0 / (scales**2))
        system = weighted_jacobian.T @ weighted_jacobian + damping * np.identity(len(names)) + 0.02 * prior
        gradient = weighted_jacobian.T @ weighted_residual + 0.02 * prior @ values
        try:
            delta = np.linalg.solve(system, -gradient)
        except np.linalg.LinAlgError as exc:
            raise ValueError("physical-model fit became singular; collect wider pose coverage") from exc
        candidate = np.clip(values + delta, -bounds, bounds)
        candidate_residual = _predicted_positions(config, names, candidate, joints) - measured
        candidate_cost = float(np.sum((candidate_residual * np.sqrt(quality)[:, None]) ** 2))
        if candidate_cost < best_cost:
            best_cost = candidate_cost
            best_values = candidate.copy()
            values = candidate
            damping = max(1e-6, damping * 0.5)
        else:
            damping = min(1e6, damping * 5.0)
        if float(np.linalg.norm(delta / scales)) < 1e-5:
            break
    final_jacobian = _jacobian(config, names, best_values, joints)
    return best_values, {
        "iterations": iterations,
        "weighted_cost": best_cost,
        "observability": _observability(final_jacobian, names),
    }


def _evaluate(
    config: RobotConfig,
    names: list[str],
    values: np.ndarray,
    samples: list[dict[str, Any]],
) -> dict[str, Any]:
    if not samples:
        return {"before": _metrics(np.empty((0, 3))), "after": _metrics(np.empty((0, 3)))}
    joints, measured, _ = _sample_vectors(samples)
    before = _predicted_positions(config, names, np.zeros(len(names)), joints) - measured
    after = _predicted_positions(config, names, values, joints) - measured
    return {"before": _metrics(before), "after": _metrics(after)}


def fit_physical_model(
    profile: dict[str, Any],
    config: RobotConfig,
    *,
    parameter_group: str = "joint_zeros",
    thresholds: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if parameter_group not in PARAMETER_GROUPS:
        raise ValueError(f"parameter_group must be one of {sorted(PARAMETER_GROUPS)}")
    freshness = profile_freshness(profile, config)
    if not freshness["fresh"]:
        raise ValueError("profile samples are stale: " + "; ".join(freshness["messages"]))
    all_samples = [sample for sample in profile.get("samples", []) if isinstance(sample, dict)]
    fit_samples = [sample for sample in all_samples if sample.get("role", "fit") == "fit"]
    validation_samples = [sample for sample in all_samples if sample.get("role") == "validation"]
    names = PARAMETER_GROUPS[parameter_group]
    minimum = max(8, len(names) + 2)
    if len(fit_samples) < minimum:
        raise ValueError(f"{parameter_group} requires at least {minimum} fit samples")
    coverage = sample_coverage(fit_samples, thresholds)
    if not coverage.get("adequate_for_physical_model"):
        raise ValueError("physical-model fit coverage is inadequate: " + "; ".join(coverage.get("warnings", [])))
    zero_joints, _, _ = _sample_vectors(fit_samples)
    initial_observability = _observability(
        _jacobian(config, names, np.zeros(len(names)), zero_joints),
        names,
    )
    if not initial_observability["identifiable"]:
        raise ValueError(
            "selected physical-model parameters are not identifiable from these samples "
            f"(rank {initial_observability['rank']}/{initial_observability['parameter_count']}, "
            f"condition {initial_observability['condition_number']:.1f})"
        )
    values, solver = _fit_values(config, names, fit_samples)
    fit_metrics = _evaluate(config, names, values, fit_samples)
    validation_metrics = _evaluate(config, names, values, validation_samples)
    active_thresholds = thresholds or {}
    validation_pass = len(validation_samples) >= 2 and _passes(validation_metrics["after"], active_thresholds)
    shifts = [
        {
            "name": name,
            "delta": float(value),
            "unit": str(PARAMETER_META[name]["unit"]),
            "bound": float(PARAMETER_META[name]["bound"]),
        }
        for name, value in zip(names, values, strict=True)
    ]
    large = [
        item["name"]
        for item in shifts
        if abs(float(item["delta"])) > (5.0 if item["unit"] == "mm" else 2.0)
    ]
    reasons: list[str] = []
    if len(validation_samples) < 2:
        reasons.append("collect at least two held-out validation samples")
    if not validation_pass:
        reasons.append("held-out physical-model validation does not pass")
    if large:
        reasons.append("large parameter changes require direct physical remeasurement: " + ", ".join(large))
    if not solver["observability"]["identifiable"]:
        reasons.append("final fit is poorly conditioned")
    return {
        "id": str(uuid4()),
        "fitted_at": datetime.now(timezone.utc).isoformat(),
        "parameter_group": parameter_group,
        "parameters": shifts,
        "coverage": coverage,
        "solver": solver,
        "fit": fit_metrics,
        "validation": validation_metrics,
        "context": calibration_context(config, str(profile.get("tool") or "")),
        "safe_to_apply": not reasons,
        "apply_blockers": reasons,
        "notes": [
            "This is a constrained physical-model update, not Cartesian command correction.",
            "Final-link length and forward TCP length are intentionally not fitted together because they are geometrically confounded.",
            "Applying a result changes the existing geometry/DH configuration and never commands motion.",
        ],
    }


def physical_model_updates(config: RobotConfig, result: dict[str, Any]) -> dict[str, Any]:
    if not result.get("safe_to_apply"):
        raise ValueError("physical-model result is not safe to apply: " + "; ".join(result.get("apply_blockers", [])))
    deltas = {str(item["name"]): float(item["delta"]) for item in result.get("parameters", [])}
    rows = [asdict(row) for row in config.kinematics.dh_rows]
    for index, row in enumerate(rows):
        row["joint"] = index + 1
        row["zero_offset_deg"] = float(row.get("zero_offset_deg", 0.0)) + deltas.get(
            f"joint_{index + 1}_zero_deg", 0.0
        )
        if index == 0:
            row["d_mm"] = float(row["d_mm"]) + deltas.get("base_height_mm", 0.0)
        elif index == 1:
            row["a_mm"] = float(row["a_mm"]) + deltas.get("upper_arm_mm", 0.0)
        elif index == 2:
            row["a_mm"] = float(row["a_mm"]) + deltas.get("forearm_mm", 0.0)

    geometry = deepcopy(config.raw.get("geometry") or {})
    active_name = str(geometry.get("active_preset") or "")
    presets = geometry.get("presets") if isinstance(geometry.get("presets"), dict) else {}
    active = deepcopy(presets.get(active_name) or {}) if active_name else {}
    dimensions = deepcopy(active.get("dimensions_mm") or {})
    if dimensions:
        dimensions["L_1"] = float(dimensions.get("L_1", 0.0)) + deltas.get("base_height_mm", 0.0)
        dimensions["L_2"] = float(dimensions.get("L_2", 0.0)) + deltas.get("base_side_offset_mm", 0.0)
        dimensions["L_5"] = float(dimensions.get("L_5", 0.0)) + deltas.get("upper_arm_mm", 0.0)
        dimensions["L_7"] = float(dimensions.get("L_7", 0.0)) + deltas.get("forearm_mm", 0.0)
        active["dimensions_mm"] = dimensions
        presets[active_name] = active
        geometry["presets"] = presets

    calibration = deepcopy(config.raw.get("calibration") or {})
    history = calibration.get("physical_model_history")
    history = list(history) if isinstance(history, list) else []
    history.append(
        {
            "applied_at": datetime.now(timezone.utc).isoformat(),
            "result_id": result.get("id"),
            "parameter_group": result.get("parameter_group"),
            "parameters": deepcopy(result.get("parameters") or []),
            "previous_context": calibration_context(config),
        }
    )
    calibration["physical_model_history"] = history[-10:]
    return {
        "links_mm": {
            "base_height": float(config.links.base_height_mm) + deltas.get("base_height_mm", 0.0),
            "base_side_offset": float(config.links.base_side_offset_mm) + deltas.get("base_side_offset_mm", 0.0),
            "upper_arm": float(config.links.upper_arm_mm) + deltas.get("upper_arm_mm", 0.0),
            "forearm": float(config.links.forearm_mm) + deltas.get("forearm_mm", 0.0),
            "wrist": float(config.links.wrist_mm),
            "tool": float(config.links.tool_mm),
        },
        "kinematics": {"dh_rows": rows},
        "geometry": geometry,
        "calibration": calibration,
    }
