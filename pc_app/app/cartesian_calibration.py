from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
from datetime import datetime, timezone
from hashlib import sha256
from math import hypot, isfinite, sqrt
from typing import Any
from uuid import uuid4

import numpy as np

from .config import RobotConfig


SCHEMA_VERSION = 2
SUPPORTED_MODELS = {"constant_xyz", "affine_xy_z_offset"}

DEFAULT_THRESHOLDS: dict[str, float] = {
    "good_xy_rmse_mm": 5.0,
    "acceptable_xy_max_mm": 10.0,
    "good_z_rmse_mm": 3.0,
    "acceptable_z_max_mm": 5.0,
    "warn_xy_rmse_mm": 8.0,
    "warn_xy_max_mm": 15.0,
    "warn_z_rmse_mm": 5.0,
    "warn_z_max_mm": 8.0,
    "minimum_sample_quality": 0.2,
    "maximum_sample_residual_mm": 250.0,
    "outlier_floor_mm": 5.0,
    "outlier_mad_scale": 3.5,
    "minimum_validation_samples": 2.0,
    "minimum_xy_span_mm": 80.0,
    "minimum_z_span_mm": 15.0,
    "minimum_phi_span_deg": 20.0,
    "maximum_enable_xy_correction_mm": 35.0,
    "maximum_enable_z_correction_mm": 20.0,
}


def calibration_settings(config: RobotConfig) -> dict[str, Any]:
    defaults: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "enabled": False,
        "active_profile": active_tool_name(config),
        "default_model": "affine_xy_z_offset",
        "thresholds": deepcopy(DEFAULT_THRESHOLDS),
        "profiles": {},
    }
    raw = config.raw.get("kinematics_calibration")
    if not isinstance(raw, dict):
        return defaults
    merged = deepcopy(defaults)
    for key, value in raw.items():
        if key == "thresholds" and isinstance(value, dict):
            merged["thresholds"].update(deepcopy(value))
        elif key == "profiles" and isinstance(value, dict):
            merged["profiles"] = deepcopy(value)
        else:
            merged[key] = deepcopy(value)
    merged["schema_version"] = SCHEMA_VERSION
    return merged


def active_tool_name(config: RobotConfig) -> str:
    tools = config.raw.get("tools")
    if isinstance(tools, dict):
        return str(tools.get("active") or "gripper")
    return "gripper"


def _stable_signature(payload: Any) -> str:
    return sha256(repr(payload).encode("utf-8")).hexdigest()[:16]


def tool_context(config: RobotConfig, tool_name: str | None = None) -> dict[str, Any]:
    tools = config.raw.get("tools")
    active = str(tool_name or active_tool_name(config))
    presets = tools.get("presets") if isinstance(tools, dict) else None
    preset = presets.get(active) if isinstance(presets, dict) else None
    preset = deepcopy(preset) if isinstance(preset, dict) else {}
    tcp = preset.get("tcp_offset_mm") if isinstance(preset.get("tcp_offset_mm"), dict) else {}
    payload = {
        "tool": active,
        "type": str(preset.get("type") or "generic"),
        "tcp_offset_mm": {
            "x": float(tcp.get("x", tcp.get("x_mm", 0.0))),
            "y": float(tcp.get("y", tcp.get("y_mm", 0.0))),
            "z": float(tcp.get("z", tcp.get("z_mm", 0.0))),
        },
    }
    return {**payload, "signature": _stable_signature(payload)}


def model_context(config: RobotConfig) -> dict[str, Any]:
    payload = {
        "links": asdict(config.links),
        "dh_rows": [asdict(row) for row in config.kinematics.dh_rows],
        "joint_limits": [
            {"name": joint.name, "min_deg": joint.min_deg, "max_deg": joint.max_deg}
            for joint in config.joints
        ],
        "coordinate_frame": "robot_base_xyz_mm_z_up",
    }
    return {"signature": _stable_signature(payload), "coordinate_frame": payload["coordinate_frame"]}


def actuator_context(config: RobotConfig) -> dict[str, Any]:
    payload = [
        {
            "name": joint.name,
            "actuator": joint.actuator,
            "zero_offset_deg": joint.zero_offset_deg,
            "direction_sign": joint.direction_sign,
            "hardware": asdict(joint.hardware),
        }
        for joint in config.joints
    ]
    return {"signature": _stable_signature(payload)}


def measurement_reference_context(config: RobotConfig) -> dict[str, Any]:
    calibration = config.raw.get("calibration")
    reference = calibration.get("measurement_reference") if isinstance(calibration, dict) else None
    reference = deepcopy(reference) if isinstance(reference, dict) else {}
    payload = {
        "frame": str(reference.get("frame") or "robot_base"),
        "workspace_plane_z_mm": float(reference.get("workspace_plane_z_mm", 0.0)),
        "z_reference": str(reference.get("z_reference") or "robot_base"),
        "measured_point": str(reference.get("measured_point") or "active_tcp"),
        "notes": str(reference.get("notes") or ""),
    }
    return {**payload, "signature": _stable_signature(payload)}


def workspace_context(config: RobotConfig) -> dict[str, Any]:
    camera = config.raw.get("camera")
    calibration = camera.get("calibration") if isinstance(camera, dict) else None
    workspace = calibration.get("workspace_aruco") if isinstance(calibration, dict) else None
    if not isinstance(workspace, dict):
        return {
            "calibrated": False,
            "source": "none",
            "signature": "",
            "last_calibrated_at": None,
            "message": "no planar workspace calibration is configured",
        }
    reference = {
        "reference_points_px": workspace.get("reference_points_px"),
        "reference_workspace_corners_px": workspace.get("reference_workspace_corners_px"),
        "reference_resolution": workspace.get("reference_resolution"),
        "workspace_polygon_robot_mm": workspace.get("workspace_polygon_robot_mm"),
        "last_calibrated_at": workspace.get("last_calibrated_at"),
    }
    signature = sha256(
        repr(reference).encode("utf-8")
    ).hexdigest()[:16]
    has_reference = bool(workspace.get("reference_points_px"))
    return {
        "calibrated": bool(workspace.get("enabled", True) and has_reference),
        "source": "workspace_aruco_saved" if has_reference else "workspace_aruco_unavailable",
        "signature": signature if has_reference else "",
        "last_calibrated_at": workspace.get("last_calibrated_at"),
        "message": (
            "saved planar workspace calibration is available"
            if has_reference
            else "planar workspace mapping has no saved reference points"
        ),
    }


def calibration_context(config: RobotConfig, tool_name: str | None = None) -> dict[str, Any]:
    return {
        "tool": tool_context(config, tool_name),
        "model": model_context(config),
        "actuator": actuator_context(config),
        "workspace": workspace_context(config),
        "measurement_reference": measurement_reference_context(config),
    }


def profile_freshness(profile: dict[str, Any] | None, config: RobotConfig) -> dict[str, Any]:
    if not isinstance(profile, dict):
        return {"fresh": False, "reasons": ["profile_missing"], "messages": ["calibration profile is missing"]}
    expected = profile.get("context")
    if not isinstance(expected, dict):
        return {
            "fresh": False,
            "reasons": ["legacy_profile"],
            "messages": ["calibration profile predates model/tool/reference signatures; refit it"],
        }
    profile_tool = str(profile.get("tool") or active_tool_name(config))
    current = calibration_context(config, profile_tool)
    checks = [
        ("tool", "tool or TCP changed"),
        ("model", "robot geometry, DH mapping, or joint limits changed"),
        ("actuator", "actuator zero/sign/hardware mapping changed"),
        ("measurement_reference", "measurement reference or workspace-plane Z changed"),
    ]
    reasons: list[str] = []
    messages: list[str] = []
    if profile_tool != active_tool_name(config):
        reasons.append("active_tool_mismatch")
        messages.append(f"profile is for {profile_tool}, but the active tool is {active_tool_name(config)}")
    for key, message in checks:
        expected_signature = (expected.get(key) or {}).get("signature")
        current_signature = (current.get(key) or {}).get("signature")
        if not expected_signature or expected_signature != current_signature:
            reasons.append(f"{key}_mismatch")
            messages.append(message)
    expected_workspace = expected.get("workspace") if isinstance(expected.get("workspace"), dict) else {}
    current_workspace = current.get("workspace") if isinstance(current.get("workspace"), dict) else {}
    if expected_workspace.get("signature") and expected_workspace.get("signature") != current_workspace.get("signature"):
        reasons.append("workspace_mismatch")
        messages.append("camera/workspace calibration changed")
    return {"fresh": not reasons, "reasons": reasons, "messages": messages, "current": current}


def _profile_key(settings: dict[str, Any], config: RobotConfig, requested: str | None = None) -> str:
    if requested:
        return str(requested)
    active = str(settings.get("active_profile") or "").strip()
    tool = active_tool_name(config)
    if active and active == tool:
        return active
    return tool


def active_profile(config: RobotConfig, requested: str | None = None) -> tuple[str, dict[str, Any] | None]:
    settings = calibration_settings(config)
    key = _profile_key(settings, config, requested)
    profiles = settings.get("profiles")
    profile = profiles.get(key) if isinstance(profiles, dict) else None
    return key, profile if isinstance(profile, dict) else None


def _target_xyz(target: dict[str, Any]) -> np.ndarray:
    values = np.array(
        [
            float(target.get("x_mm", target.get("x"))),
            float(target.get("y_mm", target.get("y"))),
            float(target.get("z_mm", target.get("z"))),
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("Cartesian target must contain finite x_mm, y_mm, and z_mm values")
    return values


def _model_coefficients(profile: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, float]:
    result = profile.get("result")
    if not isinstance(result, dict):
        raise ValueError("calibration profile has no fitted result")
    coefficients = result.get("coefficients")
    if not isinstance(coefficients, dict):
        raise ValueError("calibration result has no coefficients")
    matrix = np.asarray(coefficients.get("xy_matrix"), dtype=float)
    offset = np.asarray(coefficients.get("xy_offset_mm"), dtype=float)
    z_offset = float(coefficients.get("z_offset_mm"))
    if matrix.shape != (2, 2) or offset.shape != (2,):
        raise ValueError("calibration XY coefficients have invalid dimensions")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(offset)) or not isfinite(z_offset):
        raise ValueError("calibration coefficients contain non-finite values")
    if abs(float(np.linalg.det(matrix))) < 1e-8:
        raise ValueError("calibration XY transform is singular")
    return matrix, offset, z_offset


def predict_physical_pose(
    model_pose: dict[str, Any],
    config: RobotConfig,
    *,
    profile_key: str | None = None,
    require_enabled: bool = True,
) -> dict[str, Any]:
    settings = calibration_settings(config)
    key, profile = active_profile(config, profile_key)
    pose = deepcopy(model_pose)
    if (
        profile is None
        or (require_enabled and not bool(settings.get("enabled", False)))
        or (require_enabled and not bool(profile.get("enabled", True)))
    ):
        return pose
    matrix, offset, z_offset = _model_coefficients(profile)
    xyz = _target_xyz(pose)
    predicted_xy = matrix @ xyz[:2] + offset
    pose["x_mm"] = float(predicted_xy[0])
    pose["y_mm"] = float(predicted_xy[1])
    pose["z_mm"] = float(xyz[2] + z_offset)
    pose["calibration_profile"] = key
    return pose


def correct_cartesian_target(
    target: dict[str, Any],
    config: RobotConfig,
    *,
    apply_enabled: bool = True,
    profile_key: str | None = None,
    validation_trial: bool = False,
) -> tuple[dict[str, Any], dict[str, Any]]:
    requested = deepcopy(target)
    command = deepcopy(target)
    settings = calibration_settings(config)
    key, profile = active_profile(config, profile_key)
    metadata: dict[str, Any] = {
        "applied": False,
        "profile_key": key,
        "model_type": profile.get("model_type") if isinstance(profile, dict) else None,
        "requested_target": requested,
        "command_target": command,
        "warnings": [],
        "validation_trial": validation_trial,
    }
    if not apply_enabled:
        metadata["reason"] = "disabled_for_request"
        return command, metadata
    if not validation_trial and not bool(settings.get("enabled", False)):
        metadata["reason"] = "disabled"
        return command, metadata
    if not isinstance(profile, dict):
        metadata["reason"] = "profile_missing"
        return command, metadata
    if not validation_trial and not bool(profile.get("enabled", True)):
        metadata["reason"] = "profile_disabled"
        return command, metadata
    freshness = profile_freshness(profile, config)
    metadata["freshness"] = freshness
    if not freshness["fresh"]:
        metadata["reason"] = "stale_profile"
        metadata["warnings"].extend(freshness["messages"])
        return command, metadata
    activation = profile.get("activation")
    if not validation_trial and (not isinstance(activation, dict) or not bool(activation.get("eligible", False))):
        metadata["reason"] = "validation_required"
        metadata["warnings"].append("profile has not passed the validation and correction-magnitude activation gate")
        return command, metadata
    try:
        matrix, offset, z_offset = _model_coefficients(profile)
        desired = _target_xyz(requested)
        command_xy = np.linalg.solve(matrix, desired[:2] - offset)
        command["x_mm"] = float(command_xy[0])
        command["y_mm"] = float(command_xy[1])
        command["z_mm"] = float(desired[2] - z_offset)
    except (TypeError, ValueError, np.linalg.LinAlgError) as exc:
        metadata["reason"] = "invalid_result"
        metadata["warnings"].append(str(exc))
        return deepcopy(target), metadata

    metadata.update(
        {
            "applied": True,
            "reason": "validation_trial" if validation_trial else "enabled",
            "command_target": deepcopy(command),
            "result_id": profile.get("result", {}).get("id"),
        }
    )
    return command, metadata


def correct_waypoint_program(
    waypoints: list[dict[str, Any]],
    config: RobotConfig,
    *,
    apply_enabled: bool = True,
    validation_trial: bool = False,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    corrected: list[dict[str, Any]] = []
    metadata: list[dict[str, Any]] = []
    for index, waypoint in enumerate(waypoints):
        item = deepcopy(waypoint)
        kind = str(item.get("type") or item.get("kind") or "cartesian").lower()
        if kind != "cartesian":
            corrected.append(item)
            continue
        raw_target = item.get("target") if isinstance(item.get("target"), dict) else item
        command_target, correction = correct_cartesian_target(
            raw_target,
            config,
            apply_enabled=apply_enabled,
            validation_trial=validation_trial,
        )
        correction["waypoint_index"] = index
        correction["label"] = item.get("label") or item.get("name") or f"waypoint {index + 1}"
        metadata.append(correction)
        if isinstance(item.get("target"), dict):
            item["target"] = command_target
        else:
            item.update(command_target)
        corrected.append(item)
    return corrected, metadata


def _finite_vector(value: Any, count: int, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{name} must contain exactly {count} values")
    result = [float(item) for item in value]
    if not all(isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def create_sample(
    payload: dict[str, Any],
    config: RobotConfig,
    reported_joints_deg: list[float],
    fk_predicted: dict[str, Any],
) -> dict[str, Any]:
    intended = deepcopy(payload.get("intended_target"))
    command = deepcopy(payload.get("command_target") or intended)
    measured = deepcopy(payload.get("measured"))
    if not isinstance(intended, dict):
        raise ValueError("intended_target is required")
    if not isinstance(command, dict):
        raise ValueError("command_target is required")
    if not isinstance(measured, dict):
        raise ValueError("measured is required")
    intended_xyz = _target_xyz(intended)
    command_xyz = _target_xyz(command)
    fk_xyz = _target_xyz(fk_predicted)
    measured_xyz = _target_xyz(measured)
    joints = _finite_vector(reported_joints_deg, 4, "reported_joints_deg")
    quality = float(payload.get("quality", 1.0))
    thresholds = calibration_settings(config).get("thresholds", DEFAULT_THRESHOLDS)
    minimum_quality = float(thresholds.get("minimum_sample_quality", 0.2))
    if not isfinite(quality) or not 0.0 <= quality <= 1.0:
        raise ValueError("quality must be between 0 and 1")
    if quality < minimum_quality:
        raise ValueError(f"sample quality {quality:.2f} is below the minimum {minimum_quality:.2f}")
    role = str(payload.get("role") or "fit").lower()
    if role not in {"fit", "validation"}:
        raise ValueError("sample role must be fit or validation")
    model_residual = measured_xyz - fk_xyz
    maximum_residual = float(thresholds.get("maximum_sample_residual_mm", 250.0))
    if float(np.linalg.norm(model_residual)) > maximum_residual:
        raise ValueError(
            f"measured TCP differs from FK by more than {maximum_residual:.1f} mm; verify frame, units, and marker"
        )
    intended_phi = intended.get("phi_deg", intended.get("phi"))
    command_phi = command.get("phi_deg", command.get("phi"))
    measured_point = str(payload.get("measured_point") or "active_tcp")
    reference_frame = str(payload.get("reference_frame") or "robot_base")
    if measured_point != "active_tcp":
        raise ValueError("residual-correction samples must measure the active_tcp point")
    if reference_frame != "robot_base":
        raise ValueError("calibration sample measurements must be expressed in the robot_base frame")
    context = calibration_context(config)
    sample = {
        "id": str(payload.get("id") or uuid4()),
        "role": role,
        "timestamp": str(payload.get("timestamp") or datetime.now(timezone.utc).isoformat()),
        "tool": active_tool_name(config),
        "context": context,
        "workspace": context["workspace"],
        "measured_point": measured_point,
        "reference_frame": reference_frame,
        "intended_target": {
            "x_mm": float(intended_xyz[0]),
            "y_mm": float(intended_xyz[1]),
            "z_mm": float(intended_xyz[2]),
            "phi_deg": float(intended_phi) if intended_phi is not None else None,
        },
        "command_target": {
            "x_mm": float(command_xyz[0]),
            "y_mm": float(command_xyz[1]),
            "z_mm": float(command_xyz[2]),
            "phi_deg": float(command_phi) if command_phi is not None else None,
        },
        "reported_joints_deg": joints,
        "joint_source": str(payload.get("joint_source") or "reported"),
        "fk_predicted": {
            "x_mm": float(fk_xyz[0]),
            "y_mm": float(fk_xyz[1]),
            "z_mm": float(fk_xyz[2]),
            "phi_deg": float(fk_predicted.get("tool_phi_deg", fk_predicted.get("phi_deg", 0.0))),
        },
        "measured": {
            "x_mm": float(measured_xyz[0]),
            "y_mm": float(measured_xyz[1]),
            "z_mm": float(measured_xyz[2]),
        },
        "measurement_source": deepcopy(payload.get("measurement_source") or {}),
        "capture": deepcopy(payload.get("capture") or {}),
        "approach": deepcopy(payload.get("approach") or {}),
        "quality": quality,
        "notes": str(payload.get("notes") or ""),
        "residuals": {
            "model_mm": {
                "x": float(model_residual[0]),
                "y": float(model_residual[1]),
                "z": float(model_residual[2]),
                "xy": float(hypot(model_residual[0], model_residual[1])),
            },
            "command_mm": {
                "x": float(measured_xyz[0] - command_xyz[0]),
                "y": float(measured_xyz[1] - command_xyz[1]),
                "z": float(measured_xyz[2] - command_xyz[2]),
            },
            "ik_target_mm": {
                "x": float(fk_xyz[0] - command_xyz[0]),
                "y": float(fk_xyz[1] - command_xyz[1]),
                "z": float(fk_xyz[2] - command_xyz[2]),
                "xyz": float(np.linalg.norm(fk_xyz - command_xyz)),
            },
            "landing_mm": {
                "x": float(measured_xyz[0] - intended_xyz[0]),
                "y": float(measured_xyz[1] - intended_xyz[1]),
                "z": float(measured_xyz[2] - intended_xyz[2]),
                "xy": float(hypot(measured_xyz[0] - intended_xyz[0], measured_xyz[1] - intended_xyz[1])),
            },
        },
    }
    return sample


def sample_coverage(samples: list[dict[str, Any]], thresholds: dict[str, Any] | None = None) -> dict[str, Any]:
    active = [sample for sample in samples if isinstance(sample, dict)]
    if not active:
        return {
            "count": 0,
            "xy_span_mm": 0.0,
            "z_span_mm": 0.0,
            "phi_span_deg": 0.0,
            "non_collinear_xy": False,
            "adequate_for_affine": False,
            "adequate_for_physical_model": False,
            "warnings": ["no samples collected"],
        }
    limits = thresholds or DEFAULT_THRESHOLDS
    targets = np.array(
        [
            [
                float(sample["fk_predicted"]["x_mm"]),
                float(sample["fk_predicted"]["y_mm"]),
                float(sample["fk_predicted"]["z_mm"]),
            ]
            for sample in active
        ],
        dtype=float,
    )
    phis = np.array([float(sample["fk_predicted"].get("phi_deg", 0.0)) for sample in active], dtype=float)
    xy_span = float(np.linalg.norm(np.ptp(targets[:, :2], axis=0)))
    z_span = float(np.ptp(targets[:, 2]))
    phi_span = float(np.ptp(phis))
    design = np.column_stack((targets[:, 0], targets[:, 1], np.ones(len(active))))
    non_collinear = bool(len(active) >= 3 and np.linalg.matrix_rank(design) >= 3)
    warnings: list[str] = []
    if xy_span < float(limits.get("minimum_xy_span_mm", 80.0)):
        warnings.append("samples cover too little X/Y range")
    if z_span < float(limits.get("minimum_z_span_mm", 15.0)):
        warnings.append("samples cover too little Z range to separate constant and pose-dependent vertical error")
    if phi_span < float(limits.get("minimum_phi_span_deg", 20.0)):
        warnings.append("samples cover too little tool-pitch range to diagnose TCP versus joint/model error")
    if not non_collinear:
        warnings.append("X/Y samples are collinear or repeated")
    return {
        "count": len(active),
        "xy_span_mm": xy_span,
        "z_span_mm": z_span,
        "phi_span_deg": phi_span,
        "non_collinear_xy": non_collinear,
        "adequate_for_affine": len(active) >= 4 and non_collinear and xy_span >= float(limits.get("minimum_xy_span_mm", 80.0)),
        "adequate_for_physical_model": (
            len(active) >= 8
            and non_collinear
            and xy_span >= float(limits.get("minimum_xy_span_mm", 80.0))
            and z_span >= float(limits.get("minimum_z_span_mm", 15.0))
            and phi_span >= float(limits.get("minimum_phi_span_deg", 20.0))
        ),
        "warnings": warnings,
    }


def _sample_arrays(samples: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    expected = np.array(
        [
            [
                float(sample["fk_predicted"]["x_mm"]),
                float(sample["fk_predicted"]["y_mm"]),
                float(sample["fk_predicted"]["z_mm"]),
            ]
            for sample in samples
        ],
        dtype=float,
    )
    measured = np.array(
        [
            [
                float(sample["measured"]["x_mm"]),
                float(sample["measured"]["y_mm"]),
                float(sample["measured"]["z_mm"]),
            ]
            for sample in samples
        ],
        dtype=float,
    )
    quality = np.array([max(0.01, float(sample.get("quality", 1.0))) for sample in samples], dtype=float)
    return expected, measured, quality


def _solve_model(
    samples: list[dict[str, Any]],
    model_type: str,
) -> tuple[np.ndarray, np.ndarray, float]:
    expected, measured, quality = _sample_arrays(samples)
    weights = np.sqrt(quality)[:, None]
    if model_type == "constant_xyz":
        residual = measured - expected
        weighted = residual * quality[:, None]
        offset_xyz = weighted.sum(axis=0) / max(float(quality.sum()), 1e-9)
        return np.identity(2), offset_xyz[:2], float(offset_xyz[2])
    if model_type != "affine_xy_z_offset":
        raise ValueError(f"unsupported calibration model {model_type}")
    design = np.column_stack((expected[:, 0], expected[:, 1], np.ones(len(samples))))
    if np.linalg.matrix_rank(design) < 3:
        raise ValueError("affine XY fitting requires samples spanning at least three non-collinear XY locations")
    weighted_design = design * weights
    solution_x, *_ = np.linalg.lstsq(weighted_design, measured[:, 0:1] * weights, rcond=None)
    solution_y, *_ = np.linalg.lstsq(weighted_design, measured[:, 1:2] * weights, rcond=None)
    matrix = np.array(
        [
            [float(solution_x[0, 0]), float(solution_x[1, 0])],
            [float(solution_y[0, 0]), float(solution_y[1, 0])],
        ]
    )
    offset = np.array([float(solution_x[2, 0]), float(solution_y[2, 0])])
    z_residual = measured[:, 2] - expected[:, 2]
    z_offset = float(np.average(z_residual, weights=quality))
    condition = float(np.linalg.cond(matrix))
    if not isfinite(condition) or condition > 100.0 or abs(float(np.linalg.det(matrix))) < 1e-5:
        raise ValueError("fitted affine XY correction is ill-conditioned; collect wider, non-collinear samples")
    return matrix, offset, z_offset


def _predict_array(expected: np.ndarray, matrix: np.ndarray, offset: np.ndarray, z_offset: float) -> np.ndarray:
    predicted = np.empty_like(expected)
    predicted[:, :2] = expected[:, :2] @ matrix.T + offset
    predicted[:, 2] = expected[:, 2] + z_offset
    return predicted


def _metrics(vectors: np.ndarray) -> dict[str, Any]:
    if vectors.size == 0:
        return {
            "count": 0,
            "xy_rmse_mm": None,
            "xy_max_mm": None,
            "z_rmse_mm": None,
            "z_max_abs_mm": None,
            "xyz_rmse_mm": None,
            "worst_samples": [],
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


def _quality_status(metrics: dict[str, Any], thresholds: dict[str, Any]) -> str:
    if not metrics.get("count"):
        return "not_run"
    good = (
        float(metrics["xy_rmse_mm"]) <= float(thresholds.get("good_xy_rmse_mm", 5.0))
        and float(metrics["xy_max_mm"]) <= float(thresholds.get("acceptable_xy_max_mm", 10.0))
        and float(metrics["z_rmse_mm"]) <= float(thresholds.get("good_z_rmse_mm", 3.0))
        and float(metrics["z_max_abs_mm"]) <= float(thresholds.get("acceptable_z_max_mm", 5.0))
    )
    if good:
        return "pass"
    warning = (
        float(metrics["xy_rmse_mm"]) <= float(thresholds.get("warn_xy_rmse_mm", 8.0))
        and float(metrics["xy_max_mm"]) <= float(thresholds.get("warn_xy_max_mm", 15.0))
        and float(metrics["z_rmse_mm"]) <= float(thresholds.get("warn_z_rmse_mm", 5.0))
        and float(metrics["z_max_abs_mm"]) <= float(thresholds.get("warn_z_max_mm", 8.0))
    )
    return "warn" if warning else "fail"


def _outlier_mask(
    expected: np.ndarray,
    measured: np.ndarray,
    matrix: np.ndarray,
    offset: np.ndarray,
    z_offset: float,
    thresholds: dict[str, Any],
) -> np.ndarray:
    residual = measured - _predict_array(expected, matrix, offset, z_offset)
    scalar = np.linalg.norm(residual, axis=1)
    median = float(np.median(scalar))
    mad = float(np.median(np.abs(scalar - median)))
    robust_sigma = 1.4826 * mad
    limit = max(
        float(thresholds.get("outlier_floor_mm", 5.0)),
        median + float(thresholds.get("outlier_mad_scale", 3.5)) * robust_sigma,
    )
    return scalar <= limit


def _sample_diagnostics(
    fit_samples: list[dict[str, Any]],
    matrix: np.ndarray,
    offset: np.ndarray,
    z_offset: float,
) -> list[str]:
    notes: list[str] = []
    model_residuals = np.array(
        [
            [
                float(sample["residuals"]["model_mm"]["x"]),
                float(sample["residuals"]["model_mm"]["y"]),
                float(sample["residuals"]["model_mm"]["z"]),
            ]
            for sample in fit_samples
        ],
        dtype=float,
    )
    ik_errors = [
        float(sample.get("residuals", {}).get("ik_target_mm", {}).get("xyz", 0.0))
        for sample in fit_samples
    ]
    if ik_errors and max(ik_errors) > 2.0:
        notes.append(
            "some samples have FK-to-command error above 2 mm; inspect IK reachability, joint tracking, or reported-angle quality separately"
        )
    mean_offset = np.mean(model_residuals, axis=0)
    spread = np.std(model_residuals, axis=0)
    if float(np.linalg.norm(mean_offset)) > max(3.0, float(np.linalg.norm(spread)) * 1.5):
        notes.append("residuals are dominated by a consistent offset, which is compatible with TCP or zero-offset error")
    affine_delta = float(np.linalg.norm(matrix - np.identity(2)))
    if affine_delta > 0.04:
        notes.append(
            "the XY fit includes noticeable scale/skew; verify workspace calibration and geometry before treating it as robot-only error"
        )
    if abs(z_offset) > 10.0:
        phi_values = [
            float(sample.get("fk_predicted", {}).get("phi_deg", 0.0))
            for sample in fit_samples
        ]
        vertical_tcp_authority = max((abs(np.sin(np.deg2rad(phi))) for phi in phi_values), default=0.0)
        if vertical_tcp_authority < 0.15:
            notes.append(
                "large Z offset was measured with an almost horizontal tool-forward axis; forward TCP length alone is unlikely to explain it"
            )
            notes.append(
                "likely workspace/base Z reference, measured-point definition, shoulder/elbow zero, or actuator mapping issue"
            )
        else:
            notes.append("large fitted Z offset suggests TCP length/sign, touch-off reference, or joint-zero error")
    if len(fit_samples) >= 2:
        expected = np.array(
            [
                [sample["fk_predicted"]["x_mm"], sample["fk_predicted"]["y_mm"]]
                for sample in fit_samples
            ],
            dtype=float,
        )
        measured = np.array(
            [[sample["measured"]["x_mm"], sample["measured"]["y_mm"]] for sample in fit_samples],
            dtype=float,
        )
        repeatability_spreads: list[float] = []
        for index in range(len(fit_samples)):
            neighbours = np.linalg.norm(expected - expected[index], axis=1) <= 3.0
            if int(np.count_nonzero(neighbours)) >= 2:
                group = measured[neighbours]
                repeatability_spreads.append(
                    float(np.max(np.linalg.norm(group - np.mean(group, axis=0), axis=1)))
                )
        if repeatability_spreads and max(repeatability_spreads) > 3.0:
            notes.append(
                "repeated nearby poses vary by more than 3 mm; backlash, compliance, or measurement repeatability may limit calibration"
            )
    return notes


def _activation_assessment(
    result: dict[str, Any],
    coverage: dict[str, Any],
    thresholds: dict[str, Any],
) -> dict[str, Any]:
    reasons: list[str] = []
    validation = result.get("validation") if isinstance(result.get("validation"), dict) else {}
    validation_count = int(validation.get("after_model", {}).get("count") or 0)
    minimum_validation = int(thresholds.get("minimum_validation_samples", 2))
    if validation_count < minimum_validation:
        reasons.append(f"collect at least {minimum_validation} held-out validation samples")
    if validation.get("status") != "pass":
        reasons.append("held-out model residual validation must pass before correction can be enabled")
    if validation.get("landing_status") != "pass":
        reasons.append("held-out corrected landing validation must pass before correction can be enabled")
    coefficients = result.get("coefficients") if isinstance(result.get("coefficients"), dict) else {}
    xy_offset = np.asarray(coefficients.get("xy_offset_mm", [0.0, 0.0]), dtype=float)
    xy_matrix = np.asarray(coefficients.get("xy_matrix", np.identity(2)), dtype=float)
    z_offset = float(coefficients.get("z_offset_mm", 0.0))
    xy_bound = float(thresholds.get("maximum_enable_xy_correction_mm", 35.0))
    z_bound = float(thresholds.get("maximum_enable_z_correction_mm", 20.0))
    if float(np.linalg.norm(xy_offset)) > xy_bound:
        reasons.append(f"XY offset exceeds the {xy_bound:.1f} mm automatic-enable limit")
    if abs(z_offset) > z_bound:
        reasons.append(f"Z offset exceeds the {z_bound:.1f} mm automatic-enable limit")
    if float(np.linalg.norm(xy_matrix - np.identity(2))) > 0.20:
        reasons.append("XY scale/skew is too large for safe automatic enablement")
    if result.get("model_type") == "affine_xy_z_offset" and not coverage.get("adequate_for_affine"):
        reasons.append("fit sample X/Y coverage is inadequate for affine correction")
    return {
        "eligible": not reasons,
        "reasons": reasons,
        "validation_sample_count": validation_count,
        "minimum_validation_samples": minimum_validation,
        "correction_magnitude": {
            "xy_offset_norm_mm": float(np.linalg.norm(xy_offset)),
            "z_offset_abs_mm": abs(z_offset),
            "xy_matrix_delta_norm": float(np.linalg.norm(xy_matrix - np.identity(2))),
        },
    }


def _evaluate_samples(
    samples: list[dict[str, Any]],
    matrix: np.ndarray,
    offset: np.ndarray,
    z_offset: float,
    thresholds: dict[str, Any],
) -> dict[str, Any]:
    if not samples:
        empty = _metrics(np.empty((0, 3)))
        return {
            "before": empty,
            "after_model": empty,
            "landing": empty,
            "status": "not_run",
            "landing_status": "not_run",
            "worst_samples": [],
        }
    expected, measured, _ = _sample_arrays(samples)
    intended = np.array(
        [
            [
                float(sample["intended_target"]["x_mm"]),
                float(sample["intended_target"]["y_mm"]),
                float(sample["intended_target"]["z_mm"]),
            ]
            for sample in samples
        ],
        dtype=float,
    )
    before_vectors = measured - expected
    after_vectors = measured - _predict_array(expected, matrix, offset, z_offset)
    landing_vectors = measured - intended
    after_norm = np.linalg.norm(after_vectors, axis=1)
    worst_indices = np.argsort(after_norm)[::-1][:5]
    worst = [
        {
            "id": samples[int(index)].get("id"),
            "role": samples[int(index)].get("role"),
            "error_xyz_mm": float(after_norm[int(index)]),
            "error_xy_mm": float(np.linalg.norm(after_vectors[int(index), :2])),
            "error_z_mm": float(after_vectors[int(index), 2]),
        }
        for index in worst_indices
    ]
    after_metrics = _metrics(after_vectors)
    landing_metrics = _metrics(landing_vectors)
    return {
        "before": _metrics(before_vectors),
        "after_model": after_metrics,
        "landing": landing_metrics,
        "status": _quality_status(after_metrics, thresholds),
        "landing_status": _quality_status(landing_metrics, thresholds),
        "worst_samples": worst,
    }


def fit_profile(
    settings: dict[str, Any],
    config: RobotConfig,
    *,
    profile_key: str | None = None,
    model_type: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    updated = deepcopy(settings)
    key = _profile_key(updated, config, profile_key)
    profiles = updated.setdefault("profiles", {})
    profile = deepcopy(profiles.get(key) or {})
    samples = profile.get("samples")
    if not isinstance(samples, list):
        samples = []
    chosen_model = str(model_type or profile.get("model_type") or updated.get("default_model") or "affine_xy_z_offset")
    if chosen_model not in SUPPORTED_MODELS:
        raise ValueError(f"model_type must be one of {sorted(SUPPORTED_MODELS)}")
    fit_samples = [sample for sample in samples if isinstance(sample, dict) and sample.get("role", "fit") == "fit"]
    current_context = calibration_context(config, key)
    stale_sample_ids: list[str] = []
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        sample_context = sample.get("context")
        if not isinstance(sample_context, dict):
            stale_sample_ids.append(str(sample.get("id") or "unknown"))
            continue
        for context_key in ["tool", "model", "actuator", "measurement_reference"]:
            if (sample_context.get(context_key) or {}).get("signature") != (
                current_context.get(context_key) or {}
            ).get("signature"):
                stale_sample_ids.append(str(sample.get("id") or "unknown"))
                break
    if stale_sample_ids:
        raise ValueError(
            "samples were collected with a different or unsigned tool/model/reference context; "
            "delete and recapture them: " + ", ".join(stale_sample_ids[:5])
        )
    minimum = 2 if chosen_model == "constant_xyz" else 4
    if len(fit_samples) < minimum:
        raise ValueError(f"{chosen_model} requires at least {minimum} fit samples")
    thresholds = updated.get("thresholds") if isinstance(updated.get("thresholds"), dict) else deepcopy(DEFAULT_THRESHOLDS)

    inliers = list(range(len(fit_samples)))
    rejected: list[int] = []
    for _ in range(3):
        active_samples = [fit_samples[index] for index in inliers]
        matrix, offset, z_offset = _solve_model(active_samples, chosen_model)
        expected, measured, _ = _sample_arrays(active_samples)
        local_mask = _outlier_mask(expected, measured, matrix, offset, z_offset, thresholds)
        next_inliers = [index for index, keep in zip(inliers, local_mask, strict=True) if bool(keep)]
        next_rejected = [index for index, keep in zip(inliers, local_mask, strict=True) if not bool(keep)]
        if len(next_inliers) < minimum or not next_rejected:
            break
        rejected.extend(next_rejected)
        inliers = next_inliers

    active_samples = [fit_samples[index] for index in inliers]
    matrix, offset, z_offset = _solve_model(active_samples, chosen_model)
    fit_evaluation = _evaluate_samples(active_samples, matrix, offset, z_offset, thresholds)
    validation_samples = [
        sample for sample in samples if isinstance(sample, dict) and sample.get("role") == "validation"
    ]
    validation = _evaluate_samples(validation_samples, matrix, offset, z_offset, thresholds)
    diagnostics = _sample_diagnostics(active_samples, matrix, offset, z_offset)
    coverage = sample_coverage(active_samples, thresholds)
    rejected_ids = [fit_samples[index].get("id") for index in sorted(set(rejected))]
    result = {
        "id": str(uuid4()),
        "fitted_at": datetime.now(timezone.utc).isoformat(),
        "model_type": chosen_model,
        "coordinate_frame": "robot base frame; millimetres; +Z upward",
        "fit_sample_count": len(active_samples),
        "validation_sample_count": len(validation_samples),
        "rejected_sample_ids": rejected_ids,
        "coefficients": {
            "xy_matrix": matrix.tolist(),
            "xy_offset_mm": offset.tolist(),
            "z_offset_mm": float(z_offset),
        },
        "fit": fit_evaluation,
        "validation": validation,
        "diagnostics": diagnostics,
        "coverage": coverage,
    }
    activation = _activation_assessment(result, coverage, thresholds)
    result["activation"] = activation
    profile.update(
        {
            "tool": key,
            "enabled": bool(profile.get("enabled", False)),
            "model_type": chosen_model,
            "workspace": workspace_context(config),
            "context": calibration_context(config, key),
            "samples": samples,
            "result": result,
            "activation": activation,
        }
    )
    profiles[key] = profile
    updated["schema_version"] = SCHEMA_VERSION
    updated["active_profile"] = key
    return updated, result


def calibration_summary(config: RobotConfig) -> dict[str, Any]:
    settings = calibration_settings(config)
    key, profile = active_profile(config)
    result = profile.get("result") if isinstance(profile, dict) else None
    freshness = profile_freshness(profile, config)
    activation = profile.get("activation") if isinstance(profile, dict) else None
    activation = deepcopy(activation) if isinstance(activation, dict) else {
        "eligible": False,
        "reasons": ["fit and validate a fresh profile before enabling correction"],
    }
    return {
        "settings": settings,
        "active_profile_key": key,
        "active_profile": deepcopy(profile),
        "enabled": bool(
            settings.get("enabled", False)
            and isinstance(profile, dict)
            and profile.get("enabled", True)
            and isinstance(result, dict)
            and freshness.get("fresh")
            and activation.get("eligible")
        ),
        "freshness": freshness,
        "activation": activation,
        "context": calibration_context(config),
        "workspace": workspace_context(config),
        "supported_models": sorted(SUPPORTED_MODELS),
        "thresholds": deepcopy(settings.get("thresholds") or DEFAULT_THRESHOLDS),
        "fit_quality": deepcopy(result.get("fit")) if isinstance(result, dict) else None,
        "validation_quality": deepcopy(result.get("validation")) if isinstance(result, dict) else None,
        "coverage": deepcopy(result.get("coverage")) if isinstance(result, dict) else None,
    }
