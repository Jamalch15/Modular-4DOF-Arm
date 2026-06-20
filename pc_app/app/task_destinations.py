from __future__ import annotations

from copy import deepcopy
from math import isfinite
from typing import Any

from .config import RobotConfig
from .kinematics import forward_kinematics
from .position_library import position_target


TASK_DESTINATIONS_SCHEMA_VERSION = 1


class TaskDestinationError(ValueError):
    """Raised when a task destination cannot be normalized."""


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise TaskDestinationError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise TaskDestinationError(f"{name} must be a finite number") from exc
    if not isfinite(number):
        raise TaskDestinationError(f"{name} must be a finite number")
    return number


def _display_name(destination_id: str, raw: dict[str, Any]) -> str:
    for key in ("display_name", "label", "name"):
        value = raw.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return destination_id.replace("_", " ").replace("-", " ").title()


def _pose_from_target(target: dict[str, Any], name: str, default_z_mm: float = 45.0) -> dict[str, Any]:
    pose: dict[str, Any] = {
        "x_mm": _finite_number(target.get("x_mm", target.get("x")), f"{name} x_mm"),
        "y_mm": _finite_number(target.get("y_mm", target.get("y")), f"{name} y_mm"),
        "z_mm": _finite_number(target.get("z_mm", target.get("z", default_z_mm)), f"{name} z_mm"),
    }
    raw_phi = target.get("phi_deg", target.get("phi"))
    if bool(target.get("phi_auto", False)) or raw_phi is None:
        pose["phi_auto"] = True
        if target.get("preferred_phi_deg", target.get("phi_preference_deg")) is not None:
            pose["preferred_phi_deg"] = _finite_number(
                target.get("preferred_phi_deg", target.get("phi_preference_deg")),
                f"{name} preferred_phi_deg",
            )
    else:
        pose["phi_deg"] = _finite_number(raw_phi, f"{name} phi_deg")
    return pose


def _position_reference(value: dict[str, Any]) -> str:
    for key in ("position_id", "position_ref", "position"):
        reference = value.get(key)
        if isinstance(reference, str) and reference.strip():
            return reference.strip()
    target = value.get("target")
    if isinstance(target, dict):
        return _position_reference(target)
    return ""


def _target_from_legacy_position(
    config: RobotConfig,
    reference: str,
    legacy_positions: dict[str, dict[str, Any]] | None,
) -> dict[str, Any] | None:
    if not isinstance(legacy_positions, dict):
        return None
    referenced_position = legacy_positions.get(reference)
    if not isinstance(referenced_position, dict):
        return None
    if str(referenced_position.get("type", "joint")).lower() == "joint":
        angles = referenced_position.get("angles_deg")
        if not isinstance(angles, list):
            return None
        fk = forward_kinematics([float(value) for value in angles], config.links)
        return {
            "x_mm": fk["x_mm"],
            "y_mm": fk["y_mm"],
            "z_mm": fk["z_mm"],
            "phi_deg": fk["tool_phi_deg"],
        }
    target = referenced_position.get("target") if isinstance(referenced_position.get("target"), dict) else referenced_position
    return deepcopy(target) if isinstance(target, dict) else None


def default_task_destinations(config: RobotConfig) -> dict[str, dict[str, Any]]:
    """Return independent task defaults, not live references to named positions."""
    return {
        "dropoff_a": {
            "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
            "id": "dropoff_a",
            "label": "Dropoff A",
            "type": "cartesian",
            "source": "defaults",
            "x_mm": -160.0,
            "y_mm": 180.0,
            "z_mm": 45.0,
            "phi_deg": 0.0,
        },
        "dropoff_b": {
            "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
            "id": "dropoff_b",
            "label": "Dropoff B",
            "type": "cartesian",
            "source": "defaults",
            "x_mm": 120.0,
            "y_mm": 180.0,
            "z_mm": 45.0,
            "phi_deg": 0.0,
        },
    }


def _raw_destinations(config: RobotConfig) -> tuple[dict[str, Any] | None, str, str]:
    raw = config.raw.get("task_destinations")
    if raw is not None:
        if not isinstance(raw, dict):
            raise TaskDestinationError("task_destinations must be an object")
        raw_destinations = raw.get("destinations")
        if isinstance(raw_destinations, dict):
            return (
                {
                    str(destination_id): deepcopy(destination)
                    for destination_id, destination in raw_destinations.items()
                },
                "task_destinations",
                "task destination",
            )
        return (
            {
                str(destination_id): deepcopy(destination)
                for destination_id, destination in raw.items()
                if destination_id not in {"schema_version", "updated_at"}
            },
            "task_destinations",
            "task destination",
        )
    raw_drop_zones = config.raw.get("drop_zones")
    if raw_drop_zones is not None:
        if not isinstance(raw_drop_zones, dict):
            raise TaskDestinationError("drop_zones must be an object")
        return (
            {
                str(destination_id): deepcopy(destination)
                for destination_id, destination in raw_drop_zones.items()
            },
            "drop_zones_compat",
            "drop zone",
        )
    return None, "defaults", "task destination"


def normalize_task_destination(
    config: RobotConfig,
    destination_id: str,
    raw: dict[str, Any],
    *,
    legacy_positions: dict[str, dict[str, Any]] | None = None,
    source: str = "task_destinations",
    label_prefix: str = "task destination",
) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise TaskDestinationError(f"{label_prefix} {destination_id} must be an object")

    normalized_id = str(destination_id).strip()
    if not normalized_id:
        raise TaskDestinationError("task destination id must not be empty")
    declared_id = raw.get("id")
    if declared_id is not None and str(declared_id).strip() != normalized_id:
        raise TaskDestinationError(
            f"{label_prefix} id {declared_id} must match its stable mapping key {normalized_id}"
        )

    reference = _position_reference(raw)
    referenced_target = position_target(config, reference) if reference else None
    if referenced_target is None and reference:
        referenced_target = _target_from_legacy_position(config, reference, legacy_positions)
    if reference and referenced_target is None:
        raise TaskDestinationError(f"{label_prefix} {normalized_id} references missing position {reference}")

    target = referenced_target or (raw.get("target") if isinstance(raw.get("target"), dict) else raw)
    if not isinstance(target, dict):
        raise TaskDestinationError(f"{label_prefix} {normalized_id} target must be an object")

    destination: dict[str, Any] = {
        "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
        "id": normalized_id,
        "label": _display_name(normalized_id, raw),
        "type": "position_ref" if reference else "cartesian",
        "source": source,
        **_pose_from_target(target, f"{label_prefix} {normalized_id}"),
    }
    if reference:
        destination["position_id"] = reference
        for key in ("position_display_name", "position_type"):
            if key in referenced_target:
                destination[key] = deepcopy(referenced_target[key])
    if isinstance(raw.get("description"), str) and raw["description"].strip():
        destination["description"] = raw["description"].strip()
    for key in ("grid", "placement"):
        if isinstance(raw.get(key), dict):
            destination[key] = deepcopy(raw[key])
    return destination


def task_destinations(
    config: RobotConfig,
    legacy_positions: dict[str, dict[str, Any]] | None = None,
) -> dict[str, dict[str, Any]]:
    raw_destinations, source, label_prefix = _raw_destinations(config)
    if raw_destinations is None:
        return deepcopy(default_task_destinations(config))
    return {
        destination_id: normalize_task_destination(
            config,
            destination_id,
            raw,
            legacy_positions=legacy_positions,
            source=source,
            label_prefix=label_prefix,
        )
        for destination_id, raw in raw_destinations.items()
    }


def task_destination_payload(destinations: dict[str, dict[str, Any]]) -> dict[str, Any]:
    return {
        "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
        "destinations": deepcopy(destinations),
    }


def legacy_drop_zones_from_task_destinations(destinations: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return {
        destination_id: {
            key: deepcopy(value)
            for key, value in destination.items()
            if key not in {"schema_version", "id", "source"}
        }
        for destination_id, destination in destinations.items()
    }


def task_destination_errors(
    config: RobotConfig,
    legacy_positions: dict[str, dict[str, Any]] | None = None,
) -> dict[str, list[str]]:
    try:
        raw_destinations, source, label_prefix = _raw_destinations(config)
    except TaskDestinationError as exc:
        return {"_schema": [str(exc)]}
    if raw_destinations is None:
        return {}
    errors: dict[str, list[str]] = {}
    for destination_id, raw in raw_destinations.items():
        try:
            normalize_task_destination(
                config,
                destination_id,
                raw,
                legacy_positions=legacy_positions,
                source=source,
                label_prefix=label_prefix,
            )
        except TaskDestinationError as exc:
            errors[destination_id] = [str(exc)]
    return errors
