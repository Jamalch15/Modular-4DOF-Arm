from __future__ import annotations

from typing import Any

from .config import RobotConfig
from .demo_settings import drop_zones, named_positions, task_defaults, tool_settings


def _cartesian_target(raw: dict[str, Any]) -> dict[str, Any]:
    target = raw.get("target") if isinstance(raw.get("target"), dict) else raw
    pose: dict[str, Any] = {
        "x_mm": float(target.get("x_mm", target.get("x", 0.0))),
        "y_mm": float(target.get("y_mm", target.get("y", 0.0))),
        "z_mm": float(target.get("z_mm", target.get("z", 0.0))),
    }
    raw_phi = target.get("phi_deg", target.get("phi"))
    if bool(target.get("phi_auto", False)) or raw_phi is None:
        pose["phi_auto"] = True
    else:
        pose["phi_deg"] = float(raw_phi)
    return pose


def _safe_waypoint(config: RobotConfig) -> dict[str, Any]:
    settings = task_defaults(config)
    positions = named_positions(config)
    safe = positions.get(str(settings.get("safe_position", "safe")), {})
    if str(safe.get("type", "joint")).lower() == "joint":
        return {"type": "joint", "mode": "joint", "angles_deg": safe.get("angles_deg", config.home_pose)}
    return {"type": "cartesian", "mode": "joint", "target": _cartesian_target(safe)}


def _tool_steps(config: RobotConfig) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    tool = tool_settings(config)
    if str(tool.get("type", "")).lower() == "electromagnet":
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


def build_pick_and_place_sequence(
    config: RobotConfig,
    object_target: dict[str, Any],
    drop_zone: str | dict[str, Any] | None = None,
) -> dict[str, Any]:
    settings = task_defaults(config)
    zones = drop_zones(config)
    object_pose = _cartesian_target(object_target)
    zone_name = str(drop_zone or settings.get("default_drop_zone", "dropoff_a"))
    if isinstance(drop_zone, dict):
        drop_pose = _cartesian_target(drop_zone)
        zone_name = "custom"
    elif zone_name in zones:
        drop_pose = zones[zone_name]
    else:
        return {"ok": False, "errors": [f"unknown drop zone {zone_name}"], "steps": [], "waypoints": []}

    approach_height = float(settings.get("approach_height_mm", 80.0))
    pickup_height = float(settings.get("pickup_height_mm", 25.0))
    dropoff_height = float(settings.get("dropoff_height_mm", drop_pose["z_mm"]))

    above_pick = {**object_pose, "z_mm": max(object_pose["z_mm"], approach_height)}
    at_pick = {**object_pose, "z_mm": pickup_height}
    above_drop = {**drop_pose, "z_mm": max(drop_pose["z_mm"], approach_height)}
    at_drop = {**drop_pose, "z_mm": dropoff_height}
    safe = _safe_waypoint(config)
    release_tool, capture_tool, final_release_tool = _tool_steps(config)

    steps: list[dict[str, Any]] = [
        {"kind": "move", "label": "safe", "waypoint": safe},
        release_tool,
        {"kind": "move", "label": "above pickup", "waypoint": {"type": "cartesian", "mode": "joint", "target": above_pick}},
        {"kind": "move", "label": "pickup", "waypoint": {"type": "cartesian", "mode": "linear", "target": at_pick}},
        capture_tool,
        {"kind": "move", "label": "lift", "waypoint": {"type": "cartesian", "mode": "linear", "target": above_pick}},
        {"kind": "move", "label": "above dropoff", "waypoint": {"type": "cartesian", "mode": "joint", "target": above_drop}},
        {"kind": "move", "label": "dropoff", "waypoint": {"type": "cartesian", "mode": "linear", "target": at_drop}},
        final_release_tool,
        {"kind": "move", "label": "lift from dropoff", "waypoint": {"type": "cartesian", "mode": "linear", "target": above_drop}},
        {"kind": "move", "label": "safe", "waypoint": safe},
    ]
    waypoints = [step["waypoint"] for step in steps if step["kind"] == "move"]
    return {
        "ok": True,
        "task": "pick_and_place",
        "drop_zone": zone_name,
        "steps": steps,
        "waypoints": waypoints,
        "object_target": object_pose,
    }


def build_sorting_sequence(
    config: RobotConfig,
    detection: dict[str, Any],
    color_profiles: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    color = str(detection.get("color", ""))
    if color not in color_profiles:
        return {"ok": False, "errors": [f"unknown color profile {color}"], "steps": [], "waypoints": []}
    robot = detection.get("robot") or detection.get("target") or detection
    profile = color_profiles[color]
    drop_zone = profile.get("drop_zone")
    sequence = build_pick_and_place_sequence(config, robot, drop_zone)
    sequence["task"] = "sorting"
    sequence["color"] = color
    return sequence


def build_batch_sorting_sequence(
    config: RobotConfig,
    detections: list[dict[str, Any]],
    color_profiles: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    valid = [detection for detection in detections if detection.get("ok") and detection.get("color")]
    if not valid:
        return {"ok": False, "errors": ["no sortable detections"], "steps": [], "waypoints": []}

    grouped = sorted(valid, key=lambda detection: str(detection.get("color", "")))
    all_steps: list[dict[str, Any]] = []
    all_waypoints: list[dict[str, Any]] = []
    objects: list[dict[str, Any]] = []
    for index, detection in enumerate(grouped, start=1):
        sequence = build_sorting_sequence(config, detection, color_profiles)
        if not sequence.get("ok"):
            return {
                "ok": False,
                "errors": [f"detection {index}: {'; '.join(sequence.get('errors', []))}"],
                "steps": all_steps,
                "waypoints": all_waypoints,
            }
        objects.append(
            {
                "index": index,
                "color": sequence.get("color"),
                "drop_zone": sequence.get("drop_zone"),
                "object_target": sequence.get("object_target"),
            }
        )
        for step in sequence["steps"]:
            step = dict(step)
            step["object_index"] = index
            all_steps.append(step)
        all_waypoints.extend(sequence["waypoints"])

    return {
        "ok": True,
        "task": "color_sorting",
        "objects": objects,
        "steps": all_steps,
        "waypoints": all_waypoints,
        "object_count": len(objects),
    }
