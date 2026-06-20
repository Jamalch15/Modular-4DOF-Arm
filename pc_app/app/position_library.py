from __future__ import annotations

from copy import deepcopy
from math import isfinite
from typing import Any

from .config import RobotConfig
from .kinematics import forward_kinematics, inverse_kinematics
from .safety import validate_joint_targets


POSITION_LIBRARY_SCHEMA_VERSION = 1
POSITION_UNITS = {"length": "mm", "angle": "deg"}


class PositionLibraryError(ValueError):
    """Raised when a position-library record cannot be normalized."""


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise PositionLibraryError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise PositionLibraryError(f"{name} must be a finite number") from exc
    if not isfinite(number):
        raise PositionLibraryError(f"{name} must be a finite number")
    return number


def _display_name(position_id: str, raw: dict[str, Any]) -> str:
    for key in ("display_name", "name", "label"):
        value = raw.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return position_id.replace("_", " ").replace("-", " ").title()


def _string_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [part.strip() for part in value.split(",") if part.strip()]
    if isinstance(value, (list, tuple, set)):
        return [str(part).strip() for part in value if str(part).strip()]
    return []


def _normalize_target(raw: dict[str, Any], name: str) -> dict[str, Any]:
    target = raw.get("target") if isinstance(raw.get("target"), dict) else raw
    if not isinstance(target, dict):
        raise PositionLibraryError(f"{name} target must be an object")
    pose: dict[str, Any] = {
        "x_mm": _finite_number(target.get("x_mm", target.get("x")), f"{name}.x_mm"),
        "y_mm": _finite_number(target.get("y_mm", target.get("y")), f"{name}.y_mm"),
        "z_mm": _finite_number(target.get("z_mm", target.get("z", 0.0)), f"{name}.z_mm"),
    }
    raw_phi = target.get("phi_deg", target.get("phi"))
    if bool(target.get("phi_auto", False)) or raw_phi is None:
        pose["phi_auto"] = True
        if target.get("preferred_phi_deg", target.get("phi_preference_deg")) is not None:
            pose["preferred_phi_deg"] = _finite_number(
                target.get("preferred_phi_deg", target.get("phi_preference_deg")),
                f"{name}.preferred_phi_deg",
            )
    else:
        pose["phi_deg"] = _finite_number(raw_phi, f"{name}.phi_deg")
    return pose


def normalize_position_record(
    config: RobotConfig,
    position_id: str,
    raw: dict[str, Any],
    *,
    source: str = "position_library",
) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise PositionLibraryError(f"position {position_id} must be an object")

    normalized_id = str(raw.get("id") or position_id).strip()
    if not normalized_id:
        raise PositionLibraryError("position id must not be empty")

    kind = str(raw.get("type") or raw.get("kind") or "joint").strip().lower()
    if kind == "tcp":
        kind = "cartesian"
    if kind not in {"joint", "cartesian"}:
        raise PositionLibraryError(f"position {normalized_id} type must be joint or cartesian")

    record: dict[str, Any] = {
        "schema_version": POSITION_LIBRARY_SCHEMA_VERSION,
        "id": normalized_id,
        "display_name": _display_name(normalized_id, raw),
        "type": kind,
        "units": dict(POSITION_UNITS),
        "source": source,
    }
    if isinstance(raw.get("description"), str) and raw["description"].strip():
        record["description"] = raw["description"].strip()
    tags = _string_list(raw.get("tags"))
    if tags:
        record["tags"] = tags
    for raw_key, output_key in [
        ("preferred_motion_mode", "preferred_motion_mode"),
        ("motion_mode", "preferred_motion_mode"),
        ("tool", "tool"),
        ("required_tool", "required_tool"),
    ]:
        value = raw.get(raw_key)
        if isinstance(value, str) and value.strip():
            record[output_key] = value.strip()
    if isinstance(raw.get("metadata"), dict):
        record["metadata"] = deepcopy(raw["metadata"])
    for key in ("created_at", "updated_at"):
        value = raw.get(key)
        if isinstance(value, str) and value.strip():
            record[key] = value.strip()

    if normalized_id == "home":
        record["type"] = "joint"
        record["angles_deg"] = [float(value) for value in config.home_pose]
        return record

    if kind == "joint":
        angles = raw.get("angles_deg", raw.get("joints_deg"))
        if not isinstance(angles, list):
            raise PositionLibraryError(f"position {normalized_id} is missing angles_deg")
        record["angles_deg"] = [
            _finite_number(value, f"position {normalized_id} angles_deg[{index}]")
            for index, value in enumerate(angles)
        ]
        return record

    record["target"] = _normalize_target(raw, f"position {normalized_id}")
    return record


def _raw_position_library_positions(config: RobotConfig) -> dict[str, dict[str, Any]]:
    raw_library = config.raw.get("position_library")
    if not isinstance(raw_library, dict):
        return {}
    raw_positions = raw_library.get("positions")
    if isinstance(raw_positions, dict):
        return {
            str(position_id): deepcopy(position)
            for position_id, position in raw_positions.items()
            if isinstance(position, dict)
        }
    return {
        str(position_id): deepcopy(position)
        for position_id, position in raw_library.items()
        if position_id not in {"schema_version", "updated_at"} and isinstance(position, dict)
    }


def position_library_records(
    config: RobotConfig,
    legacy_positions: dict[str, dict[str, Any]] | None = None,
    *,
    include_legacy: bool = True,
) -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    if include_legacy and isinstance(legacy_positions, dict):
        for position_id, position in legacy_positions.items():
            records[str(position_id)] = normalize_position_record(
                config,
                str(position_id),
                position,
                source="named_positions_compat",
            )
    for position_id, position in _raw_position_library_positions(config).items():
        records[str(position_id)] = normalize_position_record(
            config,
            str(position_id),
            position,
            source="position_library",
        )
    if "home" in records:
        records["home"]["type"] = "joint"
        records["home"]["angles_deg"] = [float(value) for value in config.home_pose]
        records["home"].pop("target", None)
    return records


def position_record_to_legacy_position(record: dict[str, Any]) -> dict[str, Any]:
    kind = str(record.get("type", "joint")).lower()
    legacy: dict[str, Any] = {
        "type": "joint" if kind == "joint" else "cartesian",
        "label": str(record.get("display_name") or record.get("id") or ""),
    }
    if record.get("description"):
        legacy["description"] = deepcopy(record["description"])
    if record.get("tags"):
        legacy["tags"] = deepcopy(record["tags"])
    if record.get("preferred_motion_mode"):
        legacy["preferred_motion_mode"] = record["preferred_motion_mode"]
    if kind == "joint":
        legacy["angles_deg"] = [float(value) for value in record.get("angles_deg", [])]
    else:
        legacy["target"] = deepcopy(record.get("target", {}))
    return legacy


def legacy_named_positions_from_position_library(config: RobotConfig) -> dict[str, dict[str, Any]]:
    return {
        position_id: position_record_to_legacy_position(record)
        for position_id, record in position_library_records(config, include_legacy=False).items()
    }


def validate_position_record(config: RobotConfig, position_id: str, record: dict[str, Any]) -> list[str]:
    try:
        normalized = normalize_position_record(config, position_id, record, source=str(record.get("source", "validation")))
    except PositionLibraryError as exc:
        return [str(exc)]

    if normalized["type"] == "joint":
        result = validate_joint_targets(config, normalized.get("angles_deg", []))
        return [] if result.ok else [result.reason]

    target = normalized.get("target", {})
    ik = inverse_kinematics(target, config.links, config.joints, config.home_pose)
    return [] if ik["ok"] else [f"position {normalized['id']} has no valid IK solution"]


def position_library_errors(
    config: RobotConfig,
    records: dict[str, dict[str, Any]] | None = None,
) -> dict[str, list[str]]:
    checked = records if records is not None else position_library_records(config)
    return {
        position_id: errors
        for position_id, record in checked.items()
        if (errors := validate_position_record(config, position_id, record))
    }


def position_target(
    config: RobotConfig,
    position_id: str,
    records: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any] | None:
    library = records if records is not None else position_library_records(config, include_legacy=False)
    record = library.get(str(position_id))
    if not isinstance(record, dict):
        return None
    if validate_position_record(config, str(position_id), record):
        return None
    if str(record.get("type", "")).lower() == "joint":
        fk = forward_kinematics([float(value) for value in record.get("angles_deg", [])], config.links)
        target = {
            "x_mm": float(fk["x_mm"]),
            "y_mm": float(fk["y_mm"]),
            "z_mm": float(fk["z_mm"]),
            "phi_deg": float(fk["tool_phi_deg"]),
        }
    else:
        target = deepcopy(record.get("target", {}))
    target["position_id"] = str(position_id)
    target["position_display_name"] = str(record.get("display_name") or position_id)
    target["position_type"] = str(record.get("type") or "")
    return target
