from dataclasses import replace

import cv2
import numpy as np

from app.config import load_config
from app.demo_settings import color_profiles, drop_zones, model_validation_warnings, named_positions, validate_named_position
from app.tasks import build_pick_and_place_sequence, build_sorting_sequence
from app.vision import apply_planar_transform, detect_configured_colors, planar_transform_from_points


def test_named_positions_are_valid_or_report_reasons():
    config = load_config()
    positions = named_positions(config)

    assert "safe" in positions
    assert isinstance(validate_named_position(config, "home", positions["home"]), list)


def test_named_home_uses_joint_home_pose_even_when_raw_config_is_stale():
    config = load_config()
    raw = {
        **config.raw,
        "named_positions": {
            **config.raw.get("named_positions", {}),
            "home": {"type": "joint", "angles_deg": [10_000.0, 10_000.0, 10_000.0, 10_000.0]},
        },
    }
    patched = replace(config, raw=raw)

    assert named_positions(patched)["home"]["angles_deg"] == config.home_pose


def test_named_position_validation_reports_unreachable_cartesian_target():
    config = load_config()

    errors = validate_named_position(
        config,
        "too_far",
        {"type": "cartesian", "target": {"x_mm": 5000.0, "y_mm": 0.0, "z_mm": 5000.0, "phi_deg": 0.0}},
    )

    assert errors == ["too_far has no valid IK solution"]


def test_named_position_validation_accepts_auto_phi_cartesian_target():
    config = load_config()

    errors = validate_named_position(
        config,
        "auto_phi_target",
        {"type": "cartesian", "target": {"x_mm": 250.0, "y_mm": 0.0, "z_mm": 120.0, "phi_auto": True}},
    )

    assert errors == []


def test_drop_zones_preserve_auto_phi_targets():
    config = load_config()
    raw = {
        **config.raw,
        "drop_zones": {
            "dropoff_a": {"x_mm": -120.0, "y_mm": 180.0, "z_mm": 45.0, "phi_auto": True},
        },
    }
    patched = replace(config, raw=raw)

    zones = drop_zones(patched)

    assert zones["dropoff_a"]["phi_auto"] is True
    assert "phi_deg" not in zones["dropoff_a"]


def test_model_validation_returns_warning_list():
    config = load_config()

    warnings = model_validation_warnings(config)

    assert isinstance(warnings, list)


def test_pick_and_place_sequence_contains_tool_actions():
    config = load_config()
    sequence = build_pick_and_place_sequence(
        config,
        {"x_mm": 0.0, "y_mm": 180.0, "z_mm": 30.0, "phi_deg": 0.0},
        "dropoff_a",
    )

    assert sequence["ok"]
    assert [step["kind"] for step in sequence["steps"]].count("tool") == 3
    assert len(sequence["waypoints"]) >= 5


def test_pick_and_place_sequence_uses_active_magnet_actions():
    config = load_config()
    raw = {
        **config.raw,
        "tools": {
            **config.raw.get("tools", {}),
            "active": "magnet",
        },
    }
    patched = replace(config, raw=raw)

    sequence = build_pick_and_place_sequence(
        patched,
        {"x_mm": 0.0, "y_mm": 180.0, "z_mm": 30.0, "phi_deg": 0.0},
        "dropoff_a",
    )

    assert sequence["ok"]
    assert [step["action"] for step in sequence["steps"] if step["kind"] == "tool"] == ["off", "on", "off"]


def test_sorting_sequence_uses_color_drop_zone():
    config = load_config()
    profiles = color_profiles(config)
    sequence = build_sorting_sequence(
        config,
        {"color": "red", "robot": {"x_mm": 0.0, "y_mm": 180.0, "z_mm": 30.0, "phi_deg": 0.0}},
        profiles,
    )

    assert sequence["ok"]
    assert sequence["task"] == "sorting"
    assert sequence["drop_zone"] == profiles["red"]["drop_zone"]


def test_planar_transform_maps_synthetic_points():
    transform = planar_transform_from_points(
        [[0, 0], [100, 0], [100, 100], [0, 100]],
        [[-50, -50], [50, -50], [50, 50], [-50, 50]],
    )

    mapped = apply_planar_transform(transform, [50, 50])

    assert mapped["x_mm"] == 0.0
    assert mapped["y_mm"] == 0.0


def test_color_blob_detector_finds_synthetic_red_object():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.circle(image, (80, 60), 18, (0, 0, 255), -1)
    profiles = {
        "red": {
            "enabled": True,
            "hsv_min": [0, 80, 60],
            "hsv_max": [12, 255, 255],
            "min_area_px": 50,
            "drop_zone": "dropoff_a",
        }
    }

    detections = detect_configured_colors(image, profiles)

    assert detections[0]["ok"]
    assert detections[0]["color"] == "red"
    assert abs(detections[0]["center_px"]["x"] - 80) < 1
