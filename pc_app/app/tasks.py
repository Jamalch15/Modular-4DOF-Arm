from __future__ import annotations

from copy import deepcopy
from math import hypot, isfinite
from time import time
from typing import Any

from .config import RobotConfig
from .demo_settings import (
    calibration_settings,
    color_sorting_task_defaults,
    drop_zones,
    named_positions,
    task_defaults,
    tool_settings,
    tools_settings,
    validate_named_position,
)
from .kinematics import forward_kinematics
from .task_destinations import task_destination_errors


SUPPORTED_STRATEGIES = {"closed_loop", "batch_once"}
SUPPORTED_ORDERING = {"nearest_to_safe", "largest", "left_to_right", "color_priority", "manual", "color"}
SUPPORTED_ORIENTATION = {"fixed", "auto", "per_color", "prefer_downward"}
SUPPORTED_MOTION_MODES = {"joint", "linear"}
SUPPORTED_PLACEMENT = {"fixed", "grid", "zone_grid"}
SUPPORTED_MISSING_ZONE_POLICIES = {"error", "ignore"}
SUPPORTED_UNKNOWN_COLOR_POLICIES = {"error", "ignore"}
DEFAULT_DOWNWARD_PHI_DEG = -90.0


class TaskSettingsError(ValueError):
    """Raised when task settings would otherwise be silently coerced."""


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise TaskSettingsError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise TaskSettingsError(f"{name} must be a finite number") from exc
    if not isfinite(number):
        raise TaskSettingsError(f"{name} must be a finite number")
    return number


def _nonnegative_number(value: Any, name: str) -> float:
    number = _finite_number(value, name)
    if number < 0.0:
        raise TaskSettingsError(f"{name} must be zero or greater")
    return number


def _nonnegative_int(value: Any, name: str) -> int:
    number = _finite_number(value, name)
    if not number.is_integer() or number < 0.0:
        raise TaskSettingsError(f"{name} must be a nonnegative integer")
    return int(number)


def _positive_int(value: Any, name: str) -> int:
    number = _nonnegative_int(value, name)
    if number < 1:
        raise TaskSettingsError(f"{name} must be at least 1")
    return number


def _choice(value: Any, allowed: set[str], name: str) -> str:
    normalized = str(value).strip().lower().replace("-", "_")
    if normalized not in allowed:
        choices = ", ".join(sorted(allowed))
        raise TaskSettingsError(f"{name} must be one of: {choices}")
    return normalized


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
        return number if number == number else default
    except (TypeError, ValueError):
        return default


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        if value.lower() in {"1", "true", "yes", "on"}:
            return True
        if value.lower() in {"0", "false", "no", "off"}:
            return False
    return default


def _normalize_choice(value: Any, allowed: set[str], default: str) -> str:
    normalized = str(value or default).strip().lower().replace("-", "_")
    return normalized if normalized in allowed else default


def _list_of_strings(value: Any) -> list[str]:
    if value is None or value == "":
        return []
    if isinstance(value, str):
        return [part.strip().lower() for part in value.split(",") if part.strip()]
    if isinstance(value, (list, tuple, set)):
        return [str(part).strip().lower() for part in value if str(part).strip()]
    return []


def _deep_merge(target: dict[str, Any], source: dict[str, Any]) -> None:
    for key, value in source.items():
        if value is None:
            continue
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _deep_merge(target[key], value)
        else:
            target[key] = deepcopy(value)


def _apply_settings_aliases(settings: dict[str, Any], raw: dict[str, Any]) -> None:
    """Accept legacy and compact setting shapes without changing stored config."""

    if "strategy" in raw:
        settings["execution_strategy"] = raw["strategy"]
    if "selection_order" in raw:
        settings.setdefault("ordering", {})["policy"] = raw["selection_order"]
    if "object_selection_policy" in raw:
        settings.setdefault("ordering", {})["policy"] = raw["object_selection_policy"]
    if "color_priority" in raw:
        settings.setdefault("ordering", {})["color_priority"] = raw["color_priority"]
    if "min_confidence" in raw:
        settings.setdefault("filters", {})["min_confidence"] = raw["min_confidence"]
    if "min_area_px" in raw:
        settings.setdefault("filters", {})["min_area_px"] = raw["min_area_px"]
    if "include_colors" in raw:
        settings.setdefault("filters", {})["include_colors"] = raw["include_colors"]
    if "require_robot_coordinates" in raw:
        settings.setdefault("filters", {})["require_robot_coordinates"] = raw["require_robot_coordinates"]
    if "limits" in raw and isinstance(raw["limits"], dict):
        if raw["limits"].get("max_objects") is not None:
            settings["max_objects"] = raw["limits"]["max_objects"]
    if "ordering" in raw and isinstance(raw["ordering"], dict):
        settings.setdefault("ordering", {}).update(deepcopy(raw["ordering"]))
    if "filters" in raw and isinstance(raw["filters"], dict):
        settings.setdefault("filters", {}).update(deepcopy(raw["filters"]))
    if "motion" in raw and isinstance(raw["motion"], dict):
        settings.setdefault("motion_modes", {}).update(deepcopy(raw["motion"]))
    if "motion_modes" in raw and isinstance(raw["motion_modes"], dict):
        settings.setdefault("motion_modes", {}).update(deepcopy(raw["motion_modes"]))
    if "placement" in raw and isinstance(raw["placement"], dict):
        if raw["placement"].get("policy") is not None:
            settings["placement_policy"] = raw["placement"]["policy"]
    if "capture" in raw and isinstance(raw["capture"], dict):
        if raw["capture"].get("settle_ms") is not None:
            settings["capture_settle_ms"] = raw["capture"]["settle_ms"]
    if "tool" in raw and isinstance(raw["tool"], dict):
        if raw["tool"].get("settle_ms") is not None:
            settings["tool_settle_ms"] = raw["tool"]["settle_ms"]
        if raw["tool"].get("action_delay_ms") is not None:
            settings["tool_action_delay_ms"] = raw["tool"]["action_delay_ms"]


def normalize_color_sorting_settings(
    config: RobotConfig,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return validated color-sorting settings.

    Units and frames:
    - pickup_z_mm and dropoff_z_mm are active-TCP robot-frame Z coordinates.
    - approach clearances are relative positive offsets above those Z values.
    - vision detections contribute robot-frame X/Y only.
    """

    settings = deepcopy(color_sorting_task_defaults(config))
    if overrides:
        _deep_merge(settings, overrides)
        _apply_settings_aliases(settings, overrides)

    legacy = task_defaults(config)
    if settings.get("pickup_height_mm") is not None:
        settings["pickup_z_mm"] = settings["pickup_height_mm"]
    if settings.get("dropoff_height_mm") is not None:
        settings["dropoff_z_mm"] = settings["dropoff_height_mm"]
    if settings.get("approach_height_mm") is not None:
        approach_height = _as_float(settings.get("approach_height_mm"), 80.0)
        pickup_z = _as_float(settings.get("pickup_z_mm"), _as_float(legacy.get("pickup_height_mm"), 25.0))
        dropoff_z = _as_float(settings.get("dropoff_z_mm"), _as_float(legacy.get("dropoff_height_mm"), 45.0))
        settings["approach_clearance_mm"] = max(0.0, approach_height - pickup_z)
        settings["drop_approach_clearance_mm"] = max(0.0, approach_height - dropoff_z)

    strategy = _choice(settings.get("execution_strategy", "closed_loop"), SUPPORTED_STRATEGIES, "execution_strategy")
    settings["execution_strategy"] = strategy
    settings["max_objects"] = _positive_int(settings.get("max_objects", 10), "max_objects")
    settings["pickup_z_mm"] = _nonnegative_number(settings.get("pickup_z_mm", 25.0), "pickup_z_mm")
    settings["dropoff_z_mm"] = _nonnegative_number(settings.get("dropoff_z_mm", 45.0), "dropoff_z_mm")
    settings["approach_clearance_mm"] = _nonnegative_number(
        settings.get("approach_clearance_mm", 55.0),
        "approach_clearance_mm",
    )
    settings["drop_approach_clearance_mm"] = _nonnegative_number(
        settings.get("drop_approach_clearance_mm", 35.0),
        "drop_approach_clearance_mm",
    )
    settings["pickup_phi_deg"] = _finite_number(settings.get("pickup_phi_deg", 0.0), "pickup_phi_deg")
    settings["drop_phi_deg"] = _finite_number(settings.get("drop_phi_deg", 0.0), "drop_phi_deg")
    settings["downward_phi_deg"] = _finite_number(
        settings.get("downward_phi_deg", DEFAULT_DOWNWARD_PHI_DEG),
        "downward_phi_deg",
    )
    settings["pickup_preferred_phi_deg"] = _finite_number(
        settings.get("pickup_preferred_phi_deg", settings.get("preferred_pickup_phi_deg")),
        "pickup_preferred_phi_deg",
    )
    settings["drop_preferred_phi_deg"] = _finite_number(
        settings.get("drop_preferred_phi_deg", settings.get("preferred_drop_phi_deg")),
        "drop_preferred_phi_deg",
    )
    settings["orientation_policy"] = _choice(
        settings.get("orientation_policy", "prefer_downward"),
        SUPPORTED_ORIENTATION,
        "orientation_policy",
    )
    settings["missing_drop_zone_policy"] = _choice(
        settings.get("missing_drop_zone_policy", "error"),
        SUPPORTED_MISSING_ZONE_POLICIES,
        "missing_drop_zone_policy",
    )
    settings["unknown_color_policy"] = _choice(
        settings.get("unknown_color_policy", "ignore"),
        SUPPORTED_UNKNOWN_COLOR_POLICIES,
        "unknown_color_policy",
    )
    settings["placement_policy"] = _choice(
        settings.get("placement_policy", "fixed"),
        SUPPORTED_PLACEMENT,
        "placement_policy",
    )
    settings["capture_settle_ms"] = _nonnegative_int(settings.get("capture_settle_ms", 250), "capture_settle_ms")
    settings["tool_settle_ms"] = _nonnegative_int(settings.get("tool_settle_ms", 150), "tool_settle_ms")
    settings["tool_action_delay_ms"] = _nonnegative_int(
        settings.get("tool_action_delay_ms", 150),
        "tool_action_delay_ms",
    )
    settings["safe_position"] = str(settings.get("safe_position") or legacy.get("safe_position", "safe"))
    settings["camera_clear_position"] = str(
        settings.get("camera_clear_position") or settings["safe_position"]
    )
    settings["default_drop_zone"] = str(settings.get("default_drop_zone") or legacy.get("default_drop_zone", "dropoff_a"))

    filters = settings.setdefault("filters", {})
    filters["min_confidence"] = _finite_number(filters.get("min_confidence", 0.0), "filters.min_confidence")
    if not 0.0 <= filters["min_confidence"] <= 1.0:
        raise TaskSettingsError("filters.min_confidence must be between 0 and 1")
    filters["min_area_px"] = _nonnegative_number(filters.get("min_area_px", 0.0), "filters.min_area_px")
    filters["include_colors"] = _list_of_strings(filters.get("include_colors"))
    filters["require_robot_coordinates"] = _as_bool(filters.get("require_robot_coordinates"), True)

    ordering = settings.setdefault("ordering", {})
    ordering["policy"] = _choice(
        ordering.get("policy", "nearest_to_safe"),
        SUPPORTED_ORDERING,
        "ordering.policy",
    )
    ordering["color_priority"] = _list_of_strings(ordering.get("color_priority"))

    modes = settings.setdefault("motion_modes", {})
    for key, default in [
        ("transfer", "joint"),
        ("pickup_approach", "linear"),
        ("pickup_descent", "linear"),
        ("lift", "linear"),
        ("drop_approach", "linear"),
        ("drop_descent", "linear"),
    ]:
        modes[key] = _choice(modes.get(key, default), SUPPORTED_MOTION_MODES, f"motion_modes.{key}")

    profiles = settings.get("object_profiles")
    if profiles is None:
        profiles = {}
    if not isinstance(profiles, dict):
        raise TaskSettingsError("object_profiles must be a JSON object")
    settings["object_profiles"] = deepcopy(profiles)
    for color, profile in settings["object_profiles"].items():
        if not isinstance(profile, dict):
            raise TaskSettingsError(f"object_profiles.{color} must be an object")
        for key in ("pickup_z_mm", "dropoff_z_mm", "approach_clearance_mm", "drop_approach_clearance_mm"):
            if key in profile:
                profile[key] = _nonnegative_number(profile[key], f"object_profiles.{color}.{key}")
        for key in (
            "pickup_phi_deg",
            "drop_phi_deg",
            "downward_phi_deg",
            "pickup_preferred_phi_deg",
            "drop_preferred_phi_deg",
        ):
            if key in profile:
                profile[key] = _finite_number(profile[key], f"object_profiles.{color}.{key}")
        if "orientation_policy" in profile:
            profile["orientation_policy"] = _choice(
                profile["orientation_policy"],
                SUPPORTED_ORIENTATION,
                f"object_profiles.{color}.orientation_policy",
            )
        profile_modes = profile.get("motion_modes")
        if profile_modes is not None:
            if not isinstance(profile_modes, dict):
                raise TaskSettingsError(f"object_profiles.{color}.motion_modes must be an object")
            for key, value in profile_modes.items():
                if key not in modes:
                    raise TaskSettingsError(f"object_profiles.{color}.motion_modes.{key} is unsupported")
                profile_modes[key] = _choice(
                    value,
                    SUPPORTED_MOTION_MODES,
                    f"object_profiles.{color}.motion_modes.{key}",
                )
    return settings


def _cartesian_target(raw: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise TaskSettingsError("cartesian target must be an object")
    target = raw.get("target") if isinstance(raw.get("target"), dict) else raw
    pose: dict[str, Any] = {
        "x_mm": _finite_number(target.get("x_mm", target.get("x")), "cartesian target x_mm"),
        "y_mm": _finite_number(target.get("y_mm", target.get("y")), "cartesian target y_mm"),
        "z_mm": _finite_number(target.get("z_mm", target.get("z", 0.0)), "cartesian target z_mm"),
    }
    raw_phi = target.get("phi_deg", target.get("phi"))
    if _as_bool(target.get("phi_auto"), False) or raw_phi is None:
        pose["phi_auto"] = True
    else:
        pose["phi_deg"] = _finite_number(raw_phi, "cartesian target phi_deg")
    return pose


def named_position_waypoint(config: RobotConfig, name: str, *, mode: str = "joint") -> dict[str, Any] | None:
    positions = named_positions(config)
    position = positions.get(str(name))
    if not isinstance(position, dict):
        return None
    if validate_named_position(config, str(name), position):
        return None
    if str(position.get("type", "joint")).lower() == "joint":
        angles = position.get("angles_deg")
        if not isinstance(angles, list):
            return None
        return {"type": "joint", "mode": "joint", "angles_deg": angles}
    return {"type": "cartesian", "mode": mode, "target": _cartesian_target(position)}


def _safe_waypoint(config: RobotConfig, settings: dict[str, Any] | None = None) -> dict[str, Any]:
    resolved = settings or normalize_color_sorting_settings(config, None)
    name = str(resolved.get("safe_position", "safe"))
    waypoint = named_position_waypoint(config, name)
    if waypoint is None:
        raise TaskSettingsError(f"safe position {name} is missing or invalid")
    return waypoint


def _tool_steps(config: RobotConfig) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    tools = tools_settings(config)
    active = str(tools.get("active", "")).strip()
    presets = tools.get("presets")
    if not active or not isinstance(presets, dict) or not isinstance(presets.get(active), dict):
        raise TaskSettingsError(f"active tool {active or '-'} is missing from tool presets")
    tool = presets[active]
    tool_type = str(tool.get("type", "")).lower()
    if tool_type not in {"servo_gripper", "electromagnet"}:
        raise TaskSettingsError(f"active tool {active} has unsupported type {tool_type or '-'}")
    tcp = tool.get("tcp_offset_mm")
    if not isinstance(tcp, dict):
        raise TaskSettingsError(f"active tool {active} is missing tcp_offset_mm")
    for axis in ("x", "y", "z"):
        _finite_number(tcp.get(axis), f"tools.{active}.tcp_offset_mm.{axis}")
    if tool_type == "electromagnet":
        return (
            {"kind": "tool", "label": "magnet off", "action": "off"},
            {"kind": "tool", "label": "magnet on", "action": "on"},
            {"kind": "tool", "label": "magnet off", "action": "off"},
        )
    return (
        {"kind": "tool", "label": "open gripper", "action": "open"},
        {"kind": "tool", "label": "close gripper", "action": "close"},
        {"kind": "tool", "label": "open gripper", "action": "open"},
    )


def _validate_drop_zone_config(config: RobotConfig) -> None:
    errors = task_destination_errors(config, named_positions(config))
    if not errors:
        return
    name, messages = next(iter(errors.items()))
    if name == "_schema":
        raise TaskSettingsError("; ".join(messages))
    raise TaskSettingsError("; ".join(messages))


def _target_with_phi(x_mm: float, y_mm: float, z_mm: float, phi: dict[str, Any]) -> dict[str, Any]:
    target: dict[str, Any] = {"x_mm": float(x_mm), "y_mm": float(y_mm), "z_mm": float(z_mm)}
    if phi.get("phi_auto"):
        target["phi_auto"] = True
        if phi.get("preferred_phi_deg") is not None:
            target["preferred_phi_deg"] = _as_float(phi.get("preferred_phi_deg"), DEFAULT_DOWNWARD_PHI_DEG)
    else:
        target["phi_deg"] = _as_float(phi.get("phi_deg"), 0.0)
    return target


def _object_profile(settings: dict[str, Any], color: str) -> dict[str, Any]:
    profile = deepcopy(settings)
    overrides = settings.get("object_profiles", {}).get(color)
    if isinstance(overrides, dict):
        _deep_merge(profile, overrides)
        _apply_settings_aliases(profile, overrides)
    profile["pickup_z_mm"] = _as_float(profile.get("pickup_z_mm"), settings["pickup_z_mm"])
    profile["dropoff_z_mm"] = _as_float(profile.get("dropoff_z_mm"), settings["dropoff_z_mm"])
    profile["approach_clearance_mm"] = max(0.0, _as_float(profile.get("approach_clearance_mm"), settings["approach_clearance_mm"]))
    profile["drop_approach_clearance_mm"] = max(
        0.0,
        _as_float(profile.get("drop_approach_clearance_mm"), settings["drop_approach_clearance_mm"]),
    )
    profile["pickup_phi_deg"] = _as_float(profile.get("pickup_phi_deg"), settings["pickup_phi_deg"])
    profile["drop_phi_deg"] = _as_float(profile.get("drop_phi_deg"), settings["drop_phi_deg"])
    profile["downward_phi_deg"] = _as_float(profile.get("downward_phi_deg"), settings["downward_phi_deg"])
    profile["pickup_preferred_phi_deg"] = _as_float(
        profile.get("pickup_preferred_phi_deg", profile.get("preferred_pickup_phi_deg")),
        settings["pickup_preferred_phi_deg"],
    )
    profile["drop_preferred_phi_deg"] = _as_float(
        profile.get("drop_preferred_phi_deg", profile.get("preferred_drop_phi_deg")),
        settings["drop_preferred_phi_deg"],
    )
    profile["orientation_policy"] = _normalize_choice(
        profile.get("orientation_policy"),
        SUPPORTED_ORIENTATION,
        settings["orientation_policy"],
    )
    profile["motion_modes"] = deepcopy(settings.get("motion_modes", {}))
    if isinstance(overrides, dict) and isinstance(overrides.get("motion_modes"), dict):
        profile["motion_modes"].update(deepcopy(overrides["motion_modes"]))
    return profile


def _resolve_phi(profile: dict[str, Any], color: str) -> tuple[dict[str, Any], dict[str, Any]]:
    policy = str(profile.get("orientation_policy", "fixed"))
    if policy == "auto":
        return {"phi_auto": True}, {"phi_auto": True}
    if policy == "prefer_downward":
        return (
            {"phi_auto": True, "preferred_phi_deg": _as_float(profile.get("pickup_preferred_phi_deg"), DEFAULT_DOWNWARD_PHI_DEG)},
            {"phi_auto": True, "preferred_phi_deg": _as_float(profile.get("drop_preferred_phi_deg"), DEFAULT_DOWNWARD_PHI_DEG)},
        )
    if policy == "per_color":
        per_color = profile.get("per_color_orientation", {})
        color_settings = per_color.get(color, {}) if isinstance(per_color, dict) else {}
        if isinstance(color_settings, dict):
            color_policy = _normalize_choice(
                color_settings.get("orientation_policy", color_settings.get("policy")),
                SUPPORTED_ORIENTATION,
                "",
            )
            if color_policy == "prefer_downward":
                preferred = _as_float(
                    color_settings.get("preferred_phi_deg", color_settings.get("downward_phi_deg")),
                    _as_float(profile.get("downward_phi_deg"), DEFAULT_DOWNWARD_PHI_DEG),
                )
                return {"phi_auto": True, "preferred_phi_deg": preferred}, {"phi_auto": True, "preferred_phi_deg": preferred}
            if color_policy == "auto":
                return {"phi_auto": True}, {"phi_auto": True}
            pickup = color_settings.get("pickup_phi_deg", profile.get("pickup_phi_deg"))
            drop = color_settings.get("drop_phi_deg", profile.get("drop_phi_deg"))
            if color_settings.get("phi_auto"):
                return {"phi_auto": True}, {"phi_auto": True}
            return {"phi_deg": _as_float(pickup, 0.0)}, {"phi_deg": _as_float(drop, 0.0)}
    return {"phi_deg": _as_float(profile.get("pickup_phi_deg"), 0.0)}, {
        "phi_deg": _as_float(profile.get("drop_phi_deg"), 0.0)
    }


def build_pick_and_place_sequence(
    config: RobotConfig,
    object_target: dict[str, Any],
    drop_zone: str | dict[str, Any] | None = None,
    *,
    task_settings: dict[str, Any] | None = None,
    object_profile: dict[str, Any] | None = None,
    drop_target: dict[str, Any] | None = None,
    color: str | None = None,
    detection_id: str | None = None,
    object_index: int | None = None,
    grid_slot: dict[str, Any] | None = None,
) -> dict[str, Any]:
    try:
        settings = normalize_color_sorting_settings(config, task_settings)
        profile = object_profile or _object_profile(settings, color or "")
        safe = _safe_waypoint(config, settings)
        release_tool, capture_tool, final_release_tool = _tool_steps(config)
        _validate_drop_zone_config(config)
    except TaskSettingsError as exc:
        return {"ok": False, "errors": [str(exc)], "error": str(exc), "steps": [], "waypoints": []}
    zones = drop_zones(config)
    zone_name = str(drop_zone or settings.get("default_drop_zone", "dropoff_a"))
    try:
        object_pose = _cartesian_target(object_target)
        if drop_target is not None:
            drop_pose = {**deepcopy(drop_target), **_cartesian_target(drop_target)}
            zone_name = str(drop_pose.get("drop_zone") or zone_name)
        elif isinstance(drop_zone, dict):
            drop_pose = _cartesian_target(drop_zone)
            zone_name = "custom"
        elif zone_name in zones:
            drop_pose = {**deepcopy(zones[zone_name]), **_cartesian_target(zones[zone_name])}
        else:
            return {"ok": False, "errors": [f"unknown drop zone {zone_name}"], "steps": [], "waypoints": []}
    except TaskSettingsError as exc:
        return {"ok": False, "errors": [str(exc)], "error": str(exc), "steps": [], "waypoints": []}

    pickup_phi, drop_phi = _resolve_phi(profile, color or "")
    pickup_z = _as_float(profile.get("pickup_z_mm"), settings["pickup_z_mm"])
    dropoff_z = _as_float(profile.get("dropoff_z_mm"), settings["dropoff_z_mm"])
    approach_clearance = max(0.0, _as_float(profile.get("approach_clearance_mm"), settings["approach_clearance_mm"]))
    drop_clearance = max(0.0, _as_float(profile.get("drop_approach_clearance_mm"), settings["drop_approach_clearance_mm"]))
    modes = profile.get("motion_modes", settings.get("motion_modes", {}))
    drop_reference_meta = {
        key: deepcopy(drop_pose[key])
        for key in ("position_id", "position_display_name", "position_type")
        if key in drop_pose
    }

    # Vision may include stale or placeholder Z/phi. The task contract uses X/Y
    # from perception and resolves active-TCP Z/phi from configuration.
    above_pick = _target_with_phi(object_pose["x_mm"], object_pose["y_mm"], pickup_z + approach_clearance, pickup_phi)
    at_pick = _target_with_phi(object_pose["x_mm"], object_pose["y_mm"], pickup_z, pickup_phi)
    above_drop = _target_with_phi(
        drop_pose["x_mm"],
        drop_pose["y_mm"],
        dropoff_z + drop_clearance,
        drop_phi,
    )
    at_drop = _target_with_phi(
        drop_pose["x_mm"],
        drop_pose["y_mm"],
        dropoff_z,
        drop_phi,
    )
    object_meta = {
        "object_index": object_index,
        "color": color,
        "detection_id": detection_id,
        "drop_zone": zone_name,
        "grid_slot": grid_slot,
    }

    def move(label: str, target: dict[str, Any], mode: str) -> dict[str, Any]:
        return {
            "kind": "move",
            "label": label,
            "waypoint": {"type": "cartesian", "mode": mode, "target": target},
            **{key: value for key, value in object_meta.items() if value is not None},
        }

    steps: list[dict[str, Any]] = [
        {"kind": "move", "label": "safe", "waypoint": safe, **{k: v for k, v in object_meta.items() if v is not None}},
        release_tool,
        move("above pickup", above_pick, str(modes.get("transfer", "joint"))),
        move("pickup", at_pick, str(modes.get("pickup_descent", "linear"))),
        capture_tool,
        move("lift", above_pick, str(modes.get("lift", "linear"))),
        move("above dropoff", above_drop, str(modes.get("transfer", "joint"))),
        move("dropoff", at_drop, str(modes.get("drop_descent", "linear"))),
        final_release_tool,
        move("lift from dropoff", above_drop, str(modes.get("lift", "linear"))),
        {"kind": "move", "label": "safe", "waypoint": safe, **{k: v for k, v in object_meta.items() if v is not None}},
    ]
    for tool_step in [release_tool, capture_tool, final_release_tool]:
        tool_step.update({key: value for key, value in object_meta.items() if value is not None})
    waypoints = [
        {**step["waypoint"], "label": step.get("label")}
        for step in steps
        if step["kind"] == "move"
    ]
    return {
        "ok": True,
        "task": "pick_and_place",
        "drop_zone": zone_name,
        "grid_slot": grid_slot,
        "steps": steps,
        "waypoints": waypoints,
        "object_target": {"x_mm": object_pose["x_mm"], "y_mm": object_pose["y_mm"], "z_mm": pickup_z, **pickup_phi},
        "drop_target": {**at_drop, **drop_reference_meta, "drop_zone": zone_name},
        "motion_modes": deepcopy(modes),
    }


def _detection_id(detection: dict[str, Any], index: int) -> str:
    for key in ["id", "detection_id", "object_id"]:
        if detection.get(key) is not None:
            return str(detection[key])
    return f"detection-{index + 1}"


def _detection_color(detection: dict[str, Any]) -> str:
    return str(detection.get("label", detection.get("color", ""))).strip().lower()


def _detection_robot_xy(detection: dict[str, Any]) -> tuple[float | None, float | None]:
    robot = detection.get("robot") or detection.get("target") or detection
    if not isinstance(robot, dict):
        return None, None
    x = robot.get("x_mm", robot.get("x"))
    y = robot.get("y_mm", robot.get("y"))
    if x is None or y is None:
        return None, None
    try:
        return _finite_number(x, "detection robot x_mm"), _finite_number(y, "detection robot y_mm")
    except TaskSettingsError:
        return None, None


def _detection_area(detection: dict[str, Any]) -> float:
    if detection.get("area_px") is not None:
        return _as_float(detection.get("area_px"), 0.0)
    bbox = detection.get("bbox_px") or detection.get("bbox")
    if isinstance(bbox, dict):
        return max(0.0, _as_float(bbox.get("width"), 0.0) * _as_float(bbox.get("height"), 0.0))
    return 0.0


def _ignored_detection(
    detection: dict[str, Any],
    index: int,
    reason_code: str,
    message: str,
) -> dict[str, Any]:
    return {
        "detection_id": _detection_id(detection, index),
        "color": _detection_color(detection),
        "reason_code": reason_code,
        "reason": message,
        "message": message,
        "ok": bool(detection.get("ok")),
    }


def filter_sorting_detections(
    config: RobotConfig,
    detections: list[dict[str, Any]],
    profiles: dict[str, dict[str, Any]],
    settings: dict[str, Any],
    selected_detection_ids: list[str] | None = None,
) -> dict[str, Any]:
    selected = {str(value) for value in selected_detection_ids or []}
    selected_order = {str(value): index for index, value in enumerate(selected_detection_ids or [])}
    filters = settings.get("filters", {})
    include_colors = set(_list_of_strings(filters.get("include_colors")))
    zones = drop_zones(config)
    ignored: list[dict[str, Any]] = []
    candidates: list[dict[str, Any]] = []
    errors: list[str] = []
    for index, detection in enumerate(detections):
        if not isinstance(detection, dict):
            ignored.append(
                {
                    "detection_id": f"detection-{index + 1}",
                    "color": "",
                    "reason_code": "invalid_detection",
                    "reason": "detection entry is not an object",
                    "message": "detection entry is not an object",
                    "ok": False,
                }
            )
            continue
        detection_id = _detection_id(detection, index)
        color = _detection_color(detection)
        if selected and detection_id not in selected:
            ignored.append(_ignored_detection(detection, index, "not_selected", "not selected for this task preview"))
            continue
        if not detection.get("ok", True):
            ignored.append(_ignored_detection(detection, index, "detector_rejected", str(detection.get("reason") or "detector marked detection unusable")))
            continue
        if not color:
            ignored.append(_ignored_detection(detection, index, "unknown_color", "detection has no color label"))
            continue
        confidence = _as_float(detection.get("confidence", detection.get("quality", 1.0)), 1.0)
        if confidence < _as_float(filters.get("min_confidence"), 0.0):
            ignored.append(_ignored_detection(detection, index, "low_confidence", "confidence is below task filter"))
            continue
        area_px = _detection_area(detection)
        if area_px < _as_float(filters.get("min_area_px"), 0.0):
            ignored.append(_ignored_detection(detection, index, "small_area", "area is below task filter"))
            continue
        if include_colors and color not in include_colors:
            ignored.append(_ignored_detection(detection, index, "color_filtered", "color is not enabled for this task run"))
            continue
        if color not in profiles:
            message = f"unknown color profile {color}"
            if settings.get("unknown_color_policy") == "error":
                errors.append(message)
            ignored.append(_ignored_detection(detection, index, "unknown_color", message))
            continue
        profile = profiles[color]
        if profile.get("enabled", True) is False:
            ignored.append(_ignored_detection(detection, index, "color_disabled", f"color profile {color} is disabled"))
            continue
        x_mm, y_mm = _detection_robot_xy(detection)
        if (x_mm is None or y_mm is None) and filters.get("require_robot_coordinates", True):
            ignored.append(
                _ignored_detection(
                    detection,
                    index,
                    "no_robot_coordinates",
                    str(detection.get("projection_error") or "detection has no calibrated robot coordinates"),
                )
            )
            continue
        zone_name = str(detection.get("drop_zone") or profile.get("drop_zone") or settings.get("default_drop_zone"))
        if zone_name not in zones:
            message = f"missing drop zone {zone_name} for detected color {color}"
            if settings.get("missing_drop_zone_policy") == "error":
                errors.append(message)
            ignored.append(_ignored_detection(detection, index, "missing_drop_zone", message))
            continue
        candidates.append(
            {
                "detection_id": detection_id,
                "source_index": index,
                "selection_rank": selected_order.get(detection_id, index),
                "color": color,
                "confidence": confidence,
                "area_px": area_px,
                "x_mm": x_mm,
                "y_mm": y_mm,
                "drop_zone": zone_name,
                "detection": deepcopy(detection),
            }
        )
    return {"candidates": candidates, "ignored": ignored, "errors": errors}


def _safe_xy(config: RobotConfig, settings: dict[str, Any]) -> tuple[float, float]:
    positions = named_positions(config)
    safe = positions.get(str(settings.get("safe_position", "safe")), {})
    if str(safe.get("type", "joint")).lower() == "joint":
        angles = safe.get("angles_deg", config.home_pose)
        fk = forward_kinematics([_as_float(value) for value in angles], config.links)
        return _as_float(fk.get("x_mm"), 0.0), _as_float(fk.get("y_mm"), 0.0)
    target = safe.get("target") if isinstance(safe.get("target"), dict) else safe
    return _as_float(target.get("x_mm", target.get("x")), 0.0), _as_float(target.get("y_mm", target.get("y")), 0.0)


def order_sorting_candidates(
    config: RobotConfig,
    candidates: list[dict[str, Any]],
    settings: dict[str, Any],
) -> list[dict[str, Any]]:
    policy = settings.get("ordering", {}).get("policy", "nearest_to_safe")
    if policy == "manual":
        return sorted(candidates, key=lambda item: item.get("selection_rank", item.get("source_index", 0)))
    if policy == "largest":
        return sorted(candidates, key=lambda item: (-_as_float(item.get("area_px"), 0.0), item.get("source_index", 0)))
    if policy == "left_to_right":
        return sorted(candidates, key=lambda item: (_as_float(item.get("x_mm"), 0.0), item.get("source_index", 0)))
    if policy == "color_priority":
        priority = {color: index for index, color in enumerate(settings.get("ordering", {}).get("color_priority", []))}
        return sorted(
            candidates,
            key=lambda item: (priority.get(item.get("color"), len(priority)), item.get("source_index", 0)),
        )
    if policy == "color":
        return sorted(candidates, key=lambda item: (str(item.get("color", "")), item.get("source_index", 0)))
    safe_x, safe_y = _safe_xy(config, settings)
    return sorted(
        candidates,
        key=lambda item: (
            hypot(_as_float(item.get("x_mm"), 0.0) - safe_x, _as_float(item.get("y_mm"), 0.0) - safe_y),
            item.get("source_index", 0),
        ),
    )


def _grid_settings(zone: dict[str, Any]) -> dict[str, Any] | None:
    raw = zone.get("grid")
    if not isinstance(raw, dict):
        raw = zone.get("placement") if isinstance(zone.get("placement"), dict) else None
    if not isinstance(raw, dict):
        return None
    rows = _positive_int(raw.get("rows", 1), "drop zone grid rows")
    columns = _positive_int(raw.get("columns", raw.get("cols", 1)), "drop zone grid columns")
    return {
        "rows": rows,
        "columns": columns,
        "x_spacing_mm": _finite_number(
            raw.get("x_spacing_mm", raw.get("spacing_x_mm", raw.get("x_spacing", 0.0))),
            "drop zone grid x_spacing_mm",
        ),
        "y_spacing_mm": _finite_number(
            raw.get("y_spacing_mm", raw.get("spacing_y_mm", raw.get("y_spacing", 0.0))),
            "drop zone grid y_spacing_mm",
        ),
        "order": str(raw.get("order", "row_major")),
        "capacity": rows * columns,
    }


def assign_drop_targets(
    config: RobotConfig,
    ordered: list[dict[str, Any]],
    settings: dict[str, Any],
) -> dict[str, Any]:
    zones = drop_zones(config)
    initial_counts = settings.get("_initial_zone_counts")
    zone_counts: dict[str, int] = {
        str(name): max(0, _as_int(value, 0))
        for name, value in (initial_counts.items() if isinstance(initial_counts, dict) else [])
    }
    assigned: list[dict[str, Any]] = []
    errors: list[str] = []
    placement_policy = str(settings.get("placement_policy", "fixed"))
    for candidate in ordered:
        zone_name = str(candidate.get("drop_zone") or settings.get("default_drop_zone"))
        zone = zones.get(zone_name)
        if not isinstance(zone, dict):
            errors.append(f"missing drop zone {zone_name} for detected color {candidate.get('color')}")
            continue
        try:
            zone_x = _finite_number(zone.get("x_mm"), f"drop zone {zone_name} x_mm")
            zone_y = _finite_number(zone.get("y_mm"), f"drop zone {zone_name} y_mm")
        except TaskSettingsError as exc:
            errors.append(str(exc))
            continue
        slot = None
        target = deepcopy(zone)
        target["x_mm"] = zone_x
        target["y_mm"] = zone_y
        if placement_policy in {"grid", "zone_grid"}:
            try:
                grid = _grid_settings(zone)
            except TaskSettingsError as exc:
                errors.append(str(exc))
                continue
            if grid is None:
                errors.append(f"drop zone {zone_name} has no finite grid configuration")
                continue
            slot_index = zone_counts.get(zone_name, 0)
            if slot_index >= grid["capacity"]:
                errors.append(f"drop zone {zone_name} grid capacity exceeded ({grid['capacity']} slots)")
                continue
            row = slot_index // grid["columns"]
            column = slot_index % grid["columns"]
            target["x_mm"] = zone_x + column * grid["x_spacing_mm"]
            target["y_mm"] = zone_y + row * grid["y_spacing_mm"]
            slot = {
                "zone": zone_name,
                "index": slot_index,
                "row": row,
                "column": column,
                "rows": grid["rows"],
                "columns": grid["columns"],
            }
            zone_counts[zone_name] = slot_index + 1
        assigned.append({**candidate, "drop_target": target, "grid_slot": slot})
        if placement_policy == "fixed":
            zone_counts[zone_name] = zone_counts.get(zone_name, 0) + 1
    return {"assigned": assigned, "errors": errors}


def build_color_sorting_plan(
    config: RobotConfig,
    detections: list[dict[str, Any]],
    color_profiles: dict[str, dict[str, Any]],
    *,
    task_settings: dict[str, Any] | None = None,
    selected_detection_ids: list[str] | None = None,
) -> dict[str, Any]:
    try:
        settings = normalize_color_sorting_settings(config, task_settings)
        _safe_waypoint(config, settings)
        _tool_steps(config)
        _validate_drop_zone_config(config)
    except TaskSettingsError as exc:
        return {
            "ok": False,
            "errors": [str(exc)],
            "error": str(exc),
            "task": "color_sorting",
            "steps": [],
            "waypoints": [],
            "task_preview": {"warnings": [], "ignored_detections": [], "selected_objects": []},
        }
    strategy = settings["execution_strategy"]
    filtering = filter_sorting_detections(config, detections, color_profiles, settings, selected_detection_ids)
    candidates = filtering["candidates"]
    ignored = filtering["ignored"]
    warnings: list[str] = []
    if not calibration_settings(config).get("tool_dimensions_validated", False):
        warnings.append("active tool dimensions are not validated for hardware")
    errors = list(filtering["errors"])
    if strategy == "closed_loop":
        include_colors = set(settings.get("filters", {}).get("include_colors", []))
        zones = drop_zones(config)
        for color, profile in color_profiles.items():
            if not isinstance(profile, dict) or profile.get("enabled", True) is False:
                continue
            if include_colors and color not in include_colors:
                continue
            zone_name = str(profile.get("drop_zone") or settings.get("default_drop_zone") or "")
            if zone_name not in zones:
                message = f"missing drop zone {zone_name or '-'} for enabled color {color}"
                if settings["missing_drop_zone_policy"] == "error":
                    errors.append(message)
                else:
                    warnings.append(message)
    ordered = order_sorting_candidates(config, candidates, settings)
    if selected_detection_ids:
        missing_selected = [
            str(selected_id)
            for selected_id in selected_detection_ids
            if str(selected_id) not in {candidate["detection_id"] for candidate in candidates}
        ]
        if missing_selected:
            warnings.append(f"selected detections not eligible: {', '.join(missing_selected)}")

    if errors:
        metadata = _task_preview_metadata(
            settings=settings,
            selected_objects=[],
            ignored=ignored,
            assigned=[],
            warnings=warnings,
            strategy=strategy,
            all_candidates=ordered,
        )
        return {
            "ok": False,
            "errors": errors,
            "error": "; ".join(errors),
            "task": "color_sorting",
            "steps": [],
            "waypoints": [],
            "task_preview": metadata,
        }

    if not ordered:
        metadata = _task_preview_metadata(
            settings=settings,
            selected_objects=[],
            ignored=ignored,
            assigned=[],
            warnings=warnings,
            strategy=strategy,
            all_candidates=[],
        )
        return {
            "ok": False,
            "errors": ["no calibrated detections match enabled color profiles"],
            "error": "no calibrated detections match enabled color profiles",
            "task": "color_sorting",
            "steps": [],
            "waypoints": [],
            "task_preview": metadata,
        }

    selection_required = settings.get("ordering", {}).get("policy") == "manual" and not selected_detection_ids
    if selection_required:
        metadata = _task_preview_metadata(
            settings=settings,
            selected_objects=[],
            ignored=ignored,
            assigned=[],
            warnings=warnings,
            strategy=strategy,
            all_candidates=ordered,
            selection_required=True,
        )
        return {
            "ok": False,
            "selection_required": True,
            "error": "manual selection required",
            "errors": ["manual selection required"],
            "task": "color_sorting",
            "steps": [],
            "waypoints": [],
            "task_preview": metadata,
        }

    max_objects = settings["max_objects"]
    selected = ordered[:max_objects]
    if strategy == "closed_loop":
        selected = selected[:1]
    assigned_result = assign_drop_targets(config, selected, settings)
    assigned = assigned_result["assigned"]
    if assigned_result["errors"]:
        metadata = _task_preview_metadata(
            settings=settings,
            selected_objects=[],
            ignored=ignored,
            assigned=[],
            warnings=warnings,
            strategy=strategy,
            all_candidates=ordered,
        )
        return {
            "ok": False,
            "errors": assigned_result["errors"],
            "error": "; ".join(assigned_result["errors"]),
            "task": "color_sorting",
            "steps": [],
            "waypoints": [],
            "task_preview": metadata,
        }

    all_steps: list[dict[str, Any]] = []
    all_waypoints: list[dict[str, Any]] = []
    objects: list[dict[str, Any]] = []
    for object_index, candidate in enumerate(assigned, start=1):
        color = str(candidate["color"])
        try:
            profile = _object_profile(settings, color)
        except TaskSettingsError as exc:
            return {
                "ok": False,
                "errors": [str(exc)],
                "error": str(exc),
                "steps": all_steps,
                "waypoints": all_waypoints,
                "task_preview": _task_preview_metadata(
                    settings=settings,
                    selected_objects=objects,
                    ignored=ignored,
                    assigned=assigned,
                    warnings=warnings,
                    strategy=strategy,
                    all_candidates=ordered,
                ),
            }
        object_target = {
            "x_mm": candidate["x_mm"],
            "y_mm": candidate["y_mm"],
            "z_mm": profile["pickup_z_mm"],
            "phi_deg": profile["pickup_phi_deg"],
        }
        sequence = build_pick_and_place_sequence(
            config,
            object_target,
            candidate["drop_zone"],
            task_settings=settings,
            object_profile=profile,
            drop_target=candidate["drop_target"],
            color=color,
            detection_id=candidate["detection_id"],
            object_index=object_index,
            grid_slot=candidate.get("grid_slot"),
        )
        if not sequence.get("ok"):
            return {
                "ok": False,
                "errors": [f"detection {object_index}: {'; '.join(sequence.get('errors', []))}"],
                "steps": all_steps,
                "waypoints": all_waypoints,
                "task_preview": _task_preview_metadata(
                    settings=settings,
                    selected_objects=objects,
                    ignored=ignored,
                    assigned=assigned,
                    warnings=warnings,
                    strategy=strategy,
                    all_candidates=ordered,
                ),
            }
        object_meta = {
            "index": object_index,
            "detection_id": candidate["detection_id"],
            "color": color,
            "confidence": candidate.get("confidence"),
            "area_px": candidate.get("area_px"),
            "drop_zone": sequence.get("drop_zone"),
            "grid_slot": candidate.get("grid_slot"),
            "object_target": sequence.get("object_target"),
            "drop_target": sequence.get("drop_target"),
            "motion_modes": sequence.get("motion_modes"),
        }
        objects.append(object_meta)
        for step in sequence["steps"]:
            step = dict(step)
            step["object_index"] = object_index
            all_steps.append(step)
        all_waypoints.extend(sequence["waypoints"])

    metadata = _task_preview_metadata(
        settings=settings,
        selected_objects=objects,
        ignored=ignored,
        assigned=assigned,
        warnings=warnings,
        strategy=strategy,
        all_candidates=ordered,
    )
    return {
        "ok": True,
        "task": "color_sorting",
        "strategy": strategy,
        "objects": objects,
        "steps": all_steps,
        "waypoints": all_waypoints,
        "object_count": len(objects),
        "ignored_detections": ignored,
        "task_preview": metadata,
    }


def _task_preview_metadata(
    *,
    settings: dict[str, Any],
    selected_objects: list[dict[str, Any]],
    ignored: list[dict[str, Any]],
    assigned: list[dict[str, Any]],
    warnings: list[str],
    strategy: str,
    all_candidates: list[dict[str, Any]],
    selection_required: bool = False,
) -> dict[str, Any]:
    assigned_targets = [
        {
            "detection_id": item.get("detection_id"),
            "color": item.get("color"),
            "drop_zone": item.get("drop_zone"),
            "target": {
                "x_mm": _as_float(item.get("drop_target", {}).get("x_mm"), 0.0),
                "y_mm": _as_float(item.get("drop_target", {}).get("y_mm"), 0.0),
                "z_mm": settings["dropoff_z_mm"],
            },
            "grid_slot": item.get("grid_slot"),
        }
        for item in assigned
    ]
    candidate_summary = [
        {
            "detection_id": item.get("detection_id"),
            "color": item.get("color"),
            "confidence": item.get("confidence"),
            "area_px": item.get("area_px"),
            "robot": {"x_mm": item.get("x_mm"), "y_mm": item.get("y_mm")},
            "drop_zone": item.get("drop_zone"),
        }
        for item in all_candidates
    ]
    return {
        "created_at": time(),
        "strategy": strategy,
        "normalized_settings": deepcopy(settings),
        "selected_objects": selected_objects,
        "next_object": selected_objects[0] if selected_objects else None,
        "candidate_objects": candidate_summary,
        "ignored_detections": ignored,
        "assigned_targets": assigned_targets,
        "motion_modes": deepcopy(settings.get("motion_modes", {})),
        "warnings": warnings,
        "selection_required": selection_required,
        "estimated_duration_s": 0.0,
    }


def build_sorting_sequence(
    config: RobotConfig,
    detection: dict[str, Any],
    color_profiles: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    plan = build_color_sorting_plan(
        config,
        [detection],
        color_profiles,
        task_settings={"execution_strategy": "batch_once"},
    )
    if plan.get("ok"):
        plan["task"] = "sorting"
        if plan.get("objects"):
            plan["color"] = plan["objects"][0].get("color")
            plan["drop_zone"] = plan["objects"][0].get("drop_zone")
            plan["object_target"] = plan["objects"][0].get("object_target")
    return plan


def build_batch_sorting_sequence(
    config: RobotConfig,
    detections: list[dict[str, Any]],
    color_profiles: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    return build_color_sorting_plan(
        config,
        detections,
        color_profiles,
        task_settings={"execution_strategy": "batch_once"},
    )
