from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
from math import isfinite
from typing import Any

from .config import RobotConfig
from .demo_settings import calibration_settings, geometry_settings, tools_settings
from .kinematics import forward_kinematics


TRANSFORM_CHAIN: list[dict[str, Any]] = [
    {
        "id": "actuator",
        "label": "Actuator sensor/steps",
        "settings": ["joints[].hardware", "joints[].zero_offset_deg", "joints[].direction_sign"],
        "notes": "Controller-side mapping from motor/servo units into logical joint angle estimates.",
    },
    {
        "id": "logical_joint",
        "label": "Logical joint coordinate q",
        "settings": ["reported_angles_deg", "target_angles_deg", "joints[].home_deg"],
        "notes": "UI/API joint angles in degrees. This is the motion contract between PC and controller.",
    },
    {
        "id": "dh_theta",
        "label": "DH model joint theta",
        "settings": ["kinematics.dh_rows[].theta_offset_deg", "kinematics.dh_rows[].zero_offset_deg", "kinematics.dh_rows[].direction_sign"],
        "equation": "theta = q * dh.direction_sign + dh.zero_offset_deg + dh.theta_offset_deg",
        "notes": "Model-only mapping used by FK, IK, previews, and Cartesian planning.",
    },
    {
        "id": "flange",
        "label": "DH frame 4 / wrist-flange frame",
        "settings": ["geometry.presets.*.dimensions_mm", "links_mm", "kinematics.dh_rows"],
        "notes": "Final DH frame before the active tool TCP offset.",
    },
    {
        "id": "tcp",
        "label": "Active tool TCP frame",
        "settings": ["tools.active", "tools.presets.*.tcp_offset_mm"],
        "notes": "Tool offset is expressed in the active tool frame. Tool +Z is forward and maps to local DH +X.",
    },
    {
        "id": "command_correction",
        "label": "Optional Cartesian command correction",
        "settings": ["kinematics_calibration"],
        "notes": "Compensation layer applied to Cartesian command targets before IK. It does not change FK or DH truth.",
    },
]


FRAME_DEFINITIONS: list[dict[str, str]] = [
    {
        "id": "robot_base",
        "label": "Robot base",
        "definition": "Origin is the declared base mounting reference. +X sideways, +Y forward, +Z up. The work plate may have a non-zero Z in this frame.",
    },
    {
        "id": "dh_frame_4",
        "label": "DH frame 4 / wrist-flange",
        "definition": "Final Standard-DH frame after joint 4 and all configured DH link offsets.",
    },
    {
        "id": "tool_frame",
        "label": "Tool frame",
        "definition": "Origin is the flange origin. Tool +Z is the configured forward TCP direction; tool +X/+Y are transverse axes.",
    },
    {
        "id": "tcp",
        "label": "TCP",
        "definition": "Point used by Cartesian FK/IK, path previews, task targets, and command correction samples.",
    },
    {
        "id": "workspace_plane",
        "label": "Workspace plane",
        "definition": "Planar robot-frame X/Y map used by camera detections, with Z declared separately by calibration.measurement_reference.workspace_plane_z_mm.",
    },
    {
        "id": "camera_image",
        "label": "Camera/image",
        "definition": "Pixel frame transformed into workspace X/Y by saved planar calibration.",
    },
]


Z_AUDIT_ORDER: list[str] = [
    "Record the measurement reference: workspace Z=0 surface, contact point, and instrument.",
    "Verify which physical point is called TCP: magnet face, gripper tip, flange, marker, or another contact point.",
    "Measure active tool tcp_offset_mm directly from the flange/tool frame and confirm the sign of tool +Z.",
    "Check each joint reference zero and positive direction before changing DH lengths.",
    "Verify base/shoulder axis heights, side offsets, and final wrist/flange link length.",
    "Compare FK against measured flange and TCP positions at left/right, near/far, and high/low poses.",
    "Only after the physical model is accepted, fit Cartesian command correction for repeatable residual error.",
]


KNOWN_POSE_COLUMNS: list[dict[str, str]] = [
    {"key": "id", "label": "Pose ID", "unit": ""},
    {"key": "tool", "label": "Tool", "unit": ""},
    {"key": "angles_deg", "label": "Reported joints", "unit": "deg"},
    {"key": "reference_condition", "label": "Reference condition", "unit": ""},
    {"key": "measured_flange_mm", "label": "Measured flange XYZ", "unit": "mm"},
    {"key": "measured_tcp_mm", "label": "Measured TCP XYZ", "unit": "mm"},
    {"key": "measurement_method", "label": "Measurement method", "unit": ""},
    {"key": "expected_fk_tcp_mm", "label": "Expected FK TCP", "unit": "mm"},
    {"key": "residual_tcp_mm", "label": "Measured - FK TCP", "unit": "mm"},
]


RENDERER_PARITY_POSES: list[dict[str, Any]] = [
    {"id": "home", "label": "Configured home pose", "source": "config.home_pose"},
    {"id": "left_near", "label": "Left/near regression pose", "angles_deg": [-35.0, 35.0, -30.0, -45.0]},
    {"id": "right_high", "label": "Right/high regression pose", "angles_deg": [45.0, 60.0, 25.0, -25.0]},
]


def _point3(point: dict[str, Any] | None) -> dict[str, float] | None:
    if not isinstance(point, dict):
        return None
    keys = [
        ("x_mm", "x"),
        ("y_mm", "y"),
        ("z_mm", "z"),
    ]
    values: dict[str, float] = {}
    for canonical, alias in keys:
        value = point.get(canonical, point.get(alias))
        if value is None:
            return None
        number = float(value)
        if not isfinite(number):
            return None
        values[canonical] = number
    return values


def _residual(measured: dict[str, float] | None, expected: dict[str, Any]) -> dict[str, float] | None:
    if measured is None:
        return None
    return {
        "x_mm": measured["x_mm"] - float(expected["x_mm"]),
        "y_mm": measured["y_mm"] - float(expected["y_mm"]),
        "z_mm": measured["z_mm"] - float(expected["z_mm"]),
    }


def _active_tool(config: RobotConfig) -> dict[str, Any]:
    tools = tools_settings(config)
    active = str(tools.get("active", "gripper"))
    presets = tools.get("presets") if isinstance(tools.get("presets"), dict) else {}
    preset = deepcopy(presets.get(active, {})) if isinstance(presets.get(active), dict) else {}
    tcp = preset.get("tcp_offset_mm") if isinstance(preset.get("tcp_offset_mm"), dict) else {}
    return {
        "name": active,
        "label": preset.get("label", active),
        "type": preset.get("type", "generic"),
        "tcp_offset_mm": {
            "x": float(tcp.get("x", tcp.get("x_mm", 0.0))),
            "y": float(tcp.get("y", tcp.get("y_mm", 0.0))),
            "z": float(tcp.get("z", tcp.get("z_mm", 0.0))),
        },
        "tcp_axis_mapping": {
            "tool_x": "local DH +Y",
            "tool_y": "local DH +Z",
            "tool_z": "local DH +X / tool-forward",
        },
        "dimensions_validated": bool(calibration_settings(config).get("tool_dimensions_validated", False)),
    }


def _dh_row_by_joint(config: RobotConfig) -> dict[int, Any]:
    return {int(row.joint_index): row for row in config.kinematics.dh_rows}


def joint_convention_table(config: RobotConfig) -> list[dict[str, Any]]:
    rows = _dh_row_by_joint(config)
    table: list[dict[str, Any]] = []
    for index, joint in enumerate(config.joints):
        row = rows.get(index)
        table.append(
            {
                "joint": index + 1,
                "name": joint.name,
                "actuator": joint.actuator,
                "logical_limits_deg": {"min": joint.min_deg, "max": joint.max_deg},
                "mechanical_home_deg": joint.home_deg,
                "actuator_mapping": {
                    "zero_offset_deg": joint.zero_offset_deg,
                    "direction_sign": joint.direction_sign,
                    "role": "controller actuator-to-logical-joint mapping",
                },
                "dh_model_mapping": {
                    "theta_offset_deg": row.theta_offset_deg if row else None,
                    "zero_offset_deg": row.zero_offset_deg if row else None,
                    "direction_sign": row.direction_sign if row else None,
                    "role": "logical joint q to Standard-DH theta mapping",
                    "equation": "theta = q * direction_sign + zero_offset_deg + theta_offset_deg",
                },
            }
        )
    return table


def renderer_parity_cases(config: RobotConfig) -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    for case in RENDERER_PARITY_POSES:
        angles = [float(value) for value in case.get("angles_deg", config.home_pose)]
        fk = forward_kinematics(angles, config.links)
        cases.append(
            {
                "id": case["id"],
                "label": case["label"],
                "angles_deg": angles,
                "tcp": {
                    "x_mm": fk["x_mm"],
                    "y_mm": fk["y_mm"],
                    "z_mm": fk["z_mm"],
                },
                "flange": fk["flange_frame"]["origin"],
                "tcp_axes": fk["tcp_frame"]["axes"],
            }
        )
    return cases


def model_truth_summary(config: RobotConfig, fk: dict[str, Any] | None = None) -> dict[str, Any]:
    geometry = geometry_settings(config)
    active_geometry = str(geometry.get("active_preset", ""))
    current_fk = None
    if fk:
        current_fk = {
            "tcp": {
                "x_mm": fk.get("x_mm"),
                "y_mm": fk.get("y_mm"),
                "z_mm": fk.get("z_mm"),
                "tool_phi_deg": fk.get("tool_phi_deg", fk.get("tool_pitch_deg")),
            },
            "flange_frame": fk.get("flange_frame"),
            "tool_frame": fk.get("tool_frame"),
            "tcp_frame": fk.get("tcp_frame"),
            "tool_tcp_offset_mm": fk.get("tool_tcp_offset_mm"),
        }
    return {
        "units": {"length": "mm", "angle": "deg"},
        "active_geometry_preset": active_geometry,
        "transform_chain": deepcopy(TRANSFORM_CHAIN),
        "frames": deepcopy(FRAME_DEFINITIONS),
        "active_tool": _active_tool(config),
        "measurement_reference": deepcopy(calibration_settings(config).get("measurement_reference", {})),
        "joint_conventions": joint_convention_table(config),
        "dh_rows": [asdict(row) for row in config.kinematics.dh_rows],
        "calibration_layers": [
            {"id": "geometry", "label": "Robot geometry", "source": "geometry.presets and kinematics.dh_rows"},
            {"id": "joint_reference", "label": "Joint reference/home", "source": "joints[].home_deg and logical set-pose workflow"},
            {"id": "actuator_mapping", "label": "Actuator zero/sign/gearing", "source": "joints[].zero_offset_deg, direction_sign, hardware"},
            {"id": "tool_tcp", "label": "Tool TCP dimensions", "source": "tools.presets.*.tcp_offset_mm"},
            {"id": "workspace", "label": "Camera/workspace map", "source": "camera.calibration.workspace_aruco"},
            {"id": "measurement_reference", "label": "Workspace/TCP measurement reference", "source": "calibration.measurement_reference"},
            {"id": "command_correction", "label": "Cartesian command correction", "source": "kinematics_calibration"},
        ],
        "z_audit_order": list(Z_AUDIT_ORDER),
        "known_pose_columns": deepcopy(KNOWN_POSE_COLUMNS),
        "renderer_parity_cases": renderer_parity_cases(config),
        "current_fk": current_fk,
    }


def calibration_pose_report(config: RobotConfig, measurements: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for index, pose in enumerate(measurements or [], start=1):
        angles = pose.get("angles_deg", pose.get("reported_angles_deg", pose.get("joint_angles_deg")))
        if not isinstance(angles, list) or len(angles) != len(config.joints):
            rows.append(
                {
                    "id": str(pose.get("id", f"pose_{index}")),
                    "ok": False,
                    "error": f"expected {len(config.joints)} joint angles in angles_deg",
                }
            )
            continue
        normalized_angles = [float(value) for value in angles]
        fk = forward_kinematics(normalized_angles, config.links)
        measured_tcp = _point3(pose.get("measured_tcp_mm") or pose.get("measured_tcp"))
        measured_flange = _point3(pose.get("measured_flange_mm") or pose.get("measured_flange"))
        rows.append(
            {
                "id": str(pose.get("id", f"pose_{index}")),
                "ok": True,
                "tool": str(pose.get("tool", _active_tool(config)["name"])),
                "angles_deg": normalized_angles,
                "reference_condition": str(pose.get("reference_condition", "")),
                "measurement_method": str(pose.get("measurement_method", "")),
                "notes": str(pose.get("notes", "")),
                "expected_fk_tcp_mm": {
                    "x_mm": fk["x_mm"],
                    "y_mm": fk["y_mm"],
                    "z_mm": fk["z_mm"],
                },
                "expected_fk_flange_mm": fk["flange_frame"]["origin"],
                "measured_tcp_mm": measured_tcp,
                "measured_flange_mm": measured_flange,
                "residual_tcp_mm": _residual(measured_tcp, fk),
                "residual_flange_mm": _residual(measured_flange, fk["flange_frame"]["origin"]),
            }
        )
    return {
        "pose_count": len(rows),
        "columns": deepcopy(KNOWN_POSE_COLUMNS),
        "rows": rows,
    }
