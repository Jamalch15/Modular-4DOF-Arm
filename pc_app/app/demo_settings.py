from __future__ import annotations

from copy import deepcopy
from typing import Any

from .config import DEFAULT_GEOMETRY_CONFIG, RobotConfig, matlab_geometry_to_dh_rows
from .kinematics import forward_kinematics, inverse_kinematics
from .position_library import legacy_named_positions_from_position_library
from .task_destinations import task_destinations
from .safety import validate_joint_targets


DEFAULT_COLOR_PROFILES: dict[str, dict[str, Any]] = {
    "red": {
        "enabled": True,
        "hsv_min": [0, 80, 60],
        "hsv_max": [12, 255, 255],
        "min_area_px": 250,
        "drop_zone": "dropoff_a",
    },
    "blue": {
        "enabled": True,
        "hsv_min": [95, 80, 50],
        "hsv_max": [130, 255, 255],
        "min_area_px": 250,
        "drop_zone": "dropoff_b",
    },
}


def default_named_positions(config: RobotConfig) -> dict[str, dict[str, Any]]:
    fk = forward_kinematics(config.home_pose, config.links)
    safe = config.home_pose.copy()
    if len(safe) >= 2:
        safe[1] = max(config.joints[1].min_deg, min(config.joints[1].max_deg, 35.0))
    return {
        "home": {"type": "joint", "angles_deg": config.home_pose},
        "safe": {"type": "joint", "angles_deg": safe},
        "pickup_test": {
            "type": "cartesian",
            "target": {
                "x_mm": -120.0,
                "y_mm": 150.0,
                "z_mm": 45.0,
                "phi_deg": 0.0,
            },
        },
        "dropoff_a": {
            "type": "cartesian",
            "target": {"x_mm": -160.0, "y_mm": 180.0, "z_mm": 45.0, "phi_deg": 0.0},
        },
        "dropoff_b": {
            "type": "cartesian",
            "target": {"x_mm": 120.0, "y_mm": 180.0, "z_mm": 45.0, "phi_deg": 0.0},
        },
    }


def named_positions(config: RobotConfig) -> dict[str, dict[str, Any]]:
    defaults = default_named_positions(config)
    raw = config.raw.get("named_positions")
    if isinstance(raw, dict) and raw:
        merged = deepcopy(defaults)
        merged.update(deepcopy(raw))
        merged.update(legacy_named_positions_from_position_library(config))
        merged["home"] = defaults["home"]
        return merged
    merged = deepcopy(defaults)
    merged.update(legacy_named_positions_from_position_library(config))
    merged["home"] = defaults["home"]
    return merged


def camera_settings(config: RobotConfig) -> dict[str, Any]:
    defaults = {
        "source_index": 1,
        "enabled": False,
        "resolution": {
            "width": 640,
            "height": 480,
        },
        "detection": {
            "provider": "workspace_color",
            "workspace_only": True,
            "show_unconfigured_colors": True,
            "min_object_area_px": 400,
            "min_saturation": 60,
            "min_value": 50,
            "morph_kernel_px": 5,
            "live_interval_ms": 450,
        },
        "display": {
            "flip_x": False,
            "project_live_view": False,
            "projection_opacity": 0.72,
            "projection_interval_ms": 80,
        },
        "intrinsics": {
            "source": "uncalibrated",
            "fx_px": None,
            "fy_px": None,
            "cx_px": None,
            "cy_px": None,
            "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0],
        },
        "calibration": {
            "image_points": [],
            "robot_points": [],
            "workspace_aruco": {
                "enabled": True,
                "dictionary": "DICT_4X4_50",
                "dictionary_candidates": ["DICT_4X4_50", "DICT_APRILTAG_36H11"],
                "required_ids": [0, 1, 2, 3],
                "invert_first": True,
                "allow_normal_fallback": True,
                "minimum_samples": 12,
                "max_calibration_rmse_mm": 3.0,
                "max_calibration_error_mm": 7.0,
                "max_calibration_tag_center_error_mm": 12.0,
                "max_verification_rmse_mm": 5.0,
                "max_verification_error_mm": 10.0,
                "tag_centers_robot_mm": {
                    "0": [-218.5, 111.5],
                    "1": [218.5, 111.5],
                    "2": [-218.5, 383.5],
                    "3": [218.5, 383.5],
                },
                "tag_box_corner_index": {
                    "0": 0,
                    "1": 1,
                    "2": 3,
                    "3": 2,
                },
                "reference_points_px": {
                    "0": [99.0, 88.75],
                    "1": [545.25, 94.0],
                    "2": [96.0, 367.0],
                    "3": [541.75, 374.0],
                },
                "reference_resolution": {
                    "width": 640,
                    "height": 480,
                },
                "workspace_polygon_robot_mm": [
                    [-239.0, 86.5],
                    [239.0, 86.5],
                    [239.0, 401.5],
                    [-239.0, 401.5],
                ],
                "projection_polygon_robot_mm": [
                    [-239.0, 86.5],
                    [239.0, 86.5],
                    [239.0, 401.5],
                    [-239.0, 401.5],
                ],
                "projection_mode": "workplate",
                "projection_padding_mm": 0.0,
                "projection_pixels_per_mm": 1.0,
                "projection_jpeg_quality": 82,
                "projection_alpha": 220,
            },
            "apriltag": {
                "enabled": True,
                "dictionary": "DICT_APRILTAG_36H11",
                "dictionary_candidates": ["DICT_APRILTAG_36H11"],
                "invert_first": False,
                "allow_normal_fallback": True,
                "tag_size_mm": 40.0,
                "required_ids": [0, 1, 2, 3],
                "min_tags_for_pose": 2,
                "min_samples": 12,
                "max_samples": 120,
                "max_reprojection_error_px": 2.5,
                "max_tilt_from_down_deg": 45.0,
                "tags": {
                    "0": {
                        "workspace_corner_mm": [-239.0, 86.5, 0.0],
                        "aligned_tag_corner": "bottom_left",
                        "yaw_deg": 0.0,
                    },
                    "1": {
                        "workspace_corner_mm": [239.0, 86.5, 0.0],
                        "aligned_tag_corner": "bottom_right",
                        "yaw_deg": 0.0,
                    },
                    "2": {
                        "workspace_corner_mm": [239.0, 401.5, 0.0],
                        "aligned_tag_corner": "top_right",
                        "yaw_deg": 0.0,
                    },
                    "3": {
                        "workspace_corner_mm": [-239.0, 401.5, 0.0],
                        "aligned_tag_corner": "top_left",
                        "yaw_deg": 0.0,
                    },
                },
                "result": None,
            },
        },
    }
    raw = config.raw.get("camera")
    if isinstance(raw, dict):
        def merge_mapping(target: dict[str, Any], source: dict[str, Any]) -> None:
            for key, value in source.items():
                if isinstance(value, dict) and isinstance(target.get(key), dict):
                    merge_mapping(target[key], value)
                else:
                    target[key] = deepcopy(value)

        merged = deepcopy(defaults)
        merge_mapping(merged, raw)
        return merged
    return defaults


def color_profiles(config: RobotConfig) -> dict[str, dict[str, Any]]:
    raw = config.raw.get("color_profiles")
    if isinstance(raw, dict) and raw:
        merged = deepcopy(DEFAULT_COLOR_PROFILES)
        merged.update(deepcopy(raw))
        return merged
    return deepcopy(DEFAULT_COLOR_PROFILES)


def drop_zones(config: RobotConfig) -> dict[str, dict[str, Any]]:
    return task_destinations(config, named_positions(config))


def tool_settings(config: RobotConfig) -> dict[str, Any]:
    tools = tools_settings(config)
    active = str(tools.get("active", "gripper"))
    presets = tools.get("presets", {})
    preset = deepcopy(presets.get(active, presets.get("gripper", {})))
    preset["active"] = active
    preset["tools"] = tools
    return preset


def tools_settings(config: RobotConfig) -> dict[str, Any]:
    defaults: dict[str, Any] = {
        "active": "gripper",
        "presets": {
            "gripper": {
                "type": "servo_gripper",
                "label": "Gripper",
                "open_value": 0.0,
                "closed_value": 1.0,
                "tcp_offset_mm": {"x": 0.0, "y": 0.0, "z": 30.0},
                "io": {
                    "pwm_pin": 9,
                    "pulse_min_us": 500,
                    "pulse_max_us": 2500,
                    "pwm_frequency_hz": 50,
                },
            },
            "magnet": {
                "type": "electromagnet",
                "label": "Magnet",
                "tcp_offset_mm": {"x": 0.0, "y": 0.0, "z": 18.0},
                "io": {
                    "pin": -1,
                    "active_high": True,
                },
            },
        },
    }
    legacy = config.raw.get("tool")
    if isinstance(legacy, dict):
        defaults["active"] = "gripper" if legacy.get("type", "servo_gripper") != "electromagnet" else "magnet"
        defaults["presets"]["gripper"].update(deepcopy(legacy))
    raw = config.raw.get("tools")
    if isinstance(raw, dict):
        if "active" in raw:
            defaults["active"] = str(raw["active"])
        if isinstance(raw.get("presets"), dict):
            for name, preset in raw["presets"].items():
                if isinstance(preset, dict):
                    base = defaults["presets"].get(name, {})
                    merged = deepcopy(base)
                    merged.update(deepcopy(preset))
                    defaults["presets"][name] = merged
    return defaults


def encoder_settings(config: RobotConfig) -> dict[str, Any]:
    defaults: dict[str, Any] = {
        "enabled": True,
        "closed_loop_mode": "settle_correction",
        "settle_tolerance_deg": 1.0,
        "fault_tolerance_deg": 5.0,
        "max_correction_attempts": 2,
        "axes": [
            {"joint": 1, "name": "base", "cs_pin": 5, "zero_offset_deg": 0.0, "direction_sign": 1, "enabled": True},
            {"joint": 2, "name": "shoulder", "cs_pin": 7, "zero_offset_deg": 0.0, "direction_sign": 1, "enabled": True},
            {"joint": 3, "name": "elbow", "cs_pin": -1, "zero_offset_deg": 0.0, "direction_sign": 1, "enabled": False},
            {"joint": 4, "name": "wrist", "cs_pin": -1, "zero_offset_deg": 0.0, "direction_sign": 1, "enabled": False},
        ],
    }
    raw = config.raw.get("encoders")
    if isinstance(raw, dict):
        defaults.update({key: deepcopy(value) for key, value in raw.items() if key != "axes"})
        if isinstance(raw.get("axes"), list):
            axes = deepcopy(defaults["axes"])
            for index, patch in enumerate(raw["axes"]):
                if index < len(axes) and isinstance(patch, dict):
                    axes[index].update(deepcopy(patch))
            defaults["axes"] = axes
    return defaults


def calibration_settings(config: RobotConfig) -> dict[str, Any]:
    defaults: dict[str, Any] = {
        "movement_tolerance_deg": 0.2,
        "tool_dimensions_validated": False,
        "last_validation": "",
    }
    raw = config.raw.get("calibration")
    if isinstance(raw, dict):
        defaults.update(deepcopy(raw))
    return defaults


def geometry_settings(config: RobotConfig) -> dict[str, Any]:
    defaults = deepcopy(DEFAULT_GEOMETRY_CONFIG)
    raw = config.raw.get("geometry")
    if not isinstance(raw, dict):
        return defaults
    if "active_preset" in raw:
        defaults["active_preset"] = str(raw["active_preset"])
    if isinstance(raw.get("presets"), dict):
        for name, preset in raw["presets"].items():
            if isinstance(preset, dict):
                merged = deepcopy(defaults["presets"].get(name, {}))
                for key, value in preset.items():
                    if isinstance(value, dict) and isinstance(merged.get(key), dict):
                        nested = deepcopy(merged[key])
                        nested.update(deepcopy(value))
                        merged[key] = nested
                    else:
                        merged[key] = deepcopy(value)
                defaults["presets"][name] = merged
    return defaults


def task_defaults(config: RobotConfig) -> dict[str, Any]:
    defaults = {
        "safe_position": "safe",
        "approach_height_mm": 80.0,
        "pickup_height_mm": 25.0,
        "dropoff_height_mm": 45.0,
        "default_drop_zone": "dropoff_a",
    }
    raw = config.raw.get("task_defaults")
    if isinstance(raw, dict):
        defaults.update(deepcopy(raw))
    return defaults


def color_sorting_task_defaults(config: RobotConfig) -> dict[str, Any]:
    """Normalized defaults for the current color-sorting task workflow.

    Working assumption: task Z values are active-TCP robot-frame coordinates.
    Vision detections only supply table X/Y; pickup/drop heights and phi are
    resolved here so perception and task execution remain loosely coupled.
    """

    legacy = task_defaults(config)
    pickup_z = float(legacy.get("pickup_z_mm", legacy.get("pickup_height_mm", 25.0)))
    dropoff_z = float(legacy.get("dropoff_z_mm", legacy.get("dropoff_height_mm", 45.0)))
    legacy_approach_height = float(legacy.get("approach_height_mm", max(pickup_z, dropoff_z) + 55.0))
    approach_clearance = max(0.0, legacy_approach_height - pickup_z)
    drop_clearance = max(0.0, legacy_approach_height - dropoff_z)
    defaults: dict[str, Any] = {
        "execution_strategy": "closed_loop",
        "max_objects": 10,
        "filters": {
            "min_confidence": 0.0,
            "min_area_px": 0.0,
            "include_colors": [],
            "require_robot_coordinates": True,
        },
        "ordering": {
            "policy": "nearest_to_safe",
            "color_priority": [],
        },
        "safe_position": str(legacy.get("safe_position", "safe")),
        "camera_clear_position": str(legacy.get("camera_clear_position", legacy.get("safe_position", "safe"))),
        "pickup_z_mm": pickup_z,
        "dropoff_z_mm": dropoff_z,
        "approach_clearance_mm": approach_clearance,
        "drop_approach_clearance_mm": drop_clearance,
        "pickup_phi_deg": 0.0,
        "drop_phi_deg": 0.0,
        "downward_phi_deg": -90.0,
        "pickup_preferred_phi_deg": -90.0,
        "drop_preferred_phi_deg": -90.0,
        "orientation_policy": "prefer_downward",
        "motion_modes": {
            "transfer": "joint",
            "pickup_approach": "linear",
            "pickup_descent": "linear",
            "lift": "linear",
            "drop_approach": "linear",
            "drop_descent": "linear",
        },
        "missing_drop_zone_policy": "error",
        "unknown_color_policy": "ignore",
        "placement_policy": "fixed",
        "capture_settle_ms": 250,
        "tool_settle_ms": 150,
        "tool_action_delay_ms": 150,
        "object_profiles": {},
        "default_drop_zone": str(legacy.get("default_drop_zone", "dropoff_a")),
    }
    tasks = config.raw.get("tasks")
    if isinstance(tasks, dict) and isinstance(tasks.get("color_sorting"), dict):
        _merge_task_settings(defaults, tasks["color_sorting"])
    raw = config.raw.get("color_sorting")
    if isinstance(raw, dict):
        _merge_task_settings(defaults, raw)
    return defaults


def _merge_task_settings(target: dict[str, Any], source: dict[str, Any]) -> None:
    for key, value in source.items():
        if value is None:
            continue
        if key == "strategy":
            target["execution_strategy"] = deepcopy(value)
        elif key == "limits" and isinstance(value, dict):
            if value.get("max_objects") is not None:
                target["max_objects"] = deepcopy(value["max_objects"])
        elif key == "filters" and isinstance(value, dict):
            target["filters"].update(deepcopy(value))
        elif key == "ordering" and isinstance(value, dict):
            target["ordering"].update(deepcopy(value))
        elif key in {"motion", "motion_modes"} and isinstance(value, dict):
            target["motion_modes"].update(deepcopy(value))
        elif key in {"approach_height_mm", "pickup_height_mm", "dropoff_height_mm"}:
            target[key] = deepcopy(value)
        elif isinstance(value, dict) and isinstance(target.get(key), dict):
            target[key].update(deepcopy(value))
        else:
            target[key] = deepcopy(value)


def validate_named_position(config: RobotConfig, name: str, position: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    kind = str(position.get("type") or position.get("kind") or "joint").lower()
    if kind == "joint":
        angles = position.get("angles_deg")
        if not isinstance(angles, list):
            return [f"{name} is missing angles_deg"]
        result = validate_joint_targets(config, [float(value) for value in angles])
        if not result.ok:
            errors.append(result.reason)
        return errors

    target = position.get("target") if isinstance(position.get("target"), dict) else position
    if not isinstance(target, dict):
        return [f"{name} is missing target"]
    raw_phi = target.get("phi_deg", target.get("phi"))
    pose = {
        "x_mm": float(target.get("x_mm", target.get("x", 0.0))),
        "y_mm": float(target.get("y_mm", target.get("y", 0.0))),
        "z_mm": float(target.get("z_mm", target.get("z", 0.0))),
    }
    if bool(target.get("phi_auto", False)) or raw_phi is None:
        pose["phi_auto"] = True
    else:
        pose["phi_deg"] = float(raw_phi)
    ik = inverse_kinematics(pose, config.links, config.joints, config.home_pose)
    if not ik["ok"]:
        errors.append(f"{name} has no valid IK solution")
    return errors


def model_validation_warnings(config: RobotConfig, tolerance_mm: float = 0.05, tolerance_deg: float = 0.05) -> list[str]:
    warnings: list[str] = []
    geometry = geometry_settings(config)
    active_name = str(geometry.get("active_preset", ""))
    preset = geometry.get("presets", {}).get(active_name)
    if not isinstance(preset, dict):
        return [f"active geometry preset {active_name or '-'} is missing"]

    try:
        expected_rows = matlab_geometry_to_dh_rows(preset)
    except Exception as exc:
        return [f"active geometry preset cannot derive DH rows: {exc}"]

    actual_rows = config.kinematics.dh_rows
    if len(actual_rows) != len(expected_rows):
        warnings.append(f"DH row count {len(actual_rows)} does not match geometry preset row count {len(expected_rows)}")
        return warnings

    for index, (actual, expected) in enumerate(zip(actual_rows, expected_rows, strict=True), start=1):
        if actual.joint_index != expected.joint_index:
            warnings.append(f"DH row {index} joint index differs from active geometry preset")
        for field, tolerance, unit in [
            ("theta_offset_deg", tolerance_deg, "deg"),
            ("d_mm", tolerance_mm, "mm"),
            ("a_mm", tolerance_mm, "mm"),
            ("alpha_deg", tolerance_deg, "deg"),
        ]:
            actual_value = float(getattr(actual, field))
            expected_value = float(getattr(expected, field))
            if abs(actual_value - expected_value) > tolerance:
                warnings.append(
                    f"DH row {index} {field}={actual_value:.2f} differs from geometry-derived {expected_value:.2f} {unit}"
                )

    dimensions = preset.get("dimensions_mm", {}) if isinstance(preset.get("dimensions_mm"), dict) else {}
    if "L_2" in dimensions:
        expected_side = float(dimensions["L_2"])
        actual_side = float(config.links.base_side_offset_mm)
        if abs(actual_side - expected_side) > tolerance_mm:
            warnings.append(
                f"base_side_offset={actual_side:.2f} differs from geometry L_2={expected_side:.2f} mm"
            )
    return warnings


def named_position_errors(config: RobotConfig) -> dict[str, list[str]]:
    return {
        name: messages
        for name, position in named_positions(config).items()
        if (messages := validate_named_position(config, name, position))
    }
