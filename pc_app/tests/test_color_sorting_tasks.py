from dataclasses import replace

import pytest

from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.demo_settings import color_profiles
from app.motion import build_program_trajectory
from app.tasks import (
    build_color_sorting_plan,
    filter_sorting_detections,
    normalize_color_sorting_settings,
)


def detection(color: str, x: float, y: float, *, detection_id: str | None = None, area: float = 500.0, confidence: float = 0.9):
    return {
        "id": detection_id or f"{color}-{x}-{y}",
        "ok": True,
        "color": color,
        "label": color,
        "confidence": confidence,
        "area_px": area,
        "robot": {"x_mm": x, "y_mm": y, "z_mm": 999.0},
    }


def test_color_sorting_settings_normalize_legacy_height_aliases():
    config = load_config(EXAMPLE_CONFIG_PATH)

    settings = normalize_color_sorting_settings(
        config,
        {
            "strategy": "batch_once",
            "pickup_height_mm": 30.0,
            "dropoff_height_mm": 50.0,
            "approach_height_mm": 90.0,
            "selection_order": "largest",
        },
    )

    assert settings["execution_strategy"] == "batch_once"
    assert settings["pickup_z_mm"] == 30.0
    assert settings["dropoff_z_mm"] == 50.0
    assert settings["approach_clearance_mm"] == 60.0
    assert settings["drop_approach_clearance_mm"] == 40.0
    assert settings["ordering"]["policy"] == "largest"


def test_color_sorting_defaults_prefer_downward_orientation():
    config = load_config(EXAMPLE_CONFIG_PATH)

    settings = normalize_color_sorting_settings(config)

    assert settings["orientation_policy"] == "prefer_downward"
    assert settings["pickup_preferred_phi_deg"] == -90.0
    assert settings["drop_preferred_phi_deg"] == -90.0


def test_filtering_reports_structured_ignored_reasons():
    config = load_config(EXAMPLE_CONFIG_PATH)
    profiles = color_profiles(config)
    settings = normalize_color_sorting_settings(
        config,
        {
            "filters": {"min_confidence": 0.5, "include_colors": ["red"]},
            "unknown_color_policy": "ignore",
        },
    )
    detections = [
        detection("red", 0, 180, detection_id="ok"),
        {**detection("red", 10, 180, detection_id="low"), "confidence": 0.2},
        detection("blue", 20, 180, detection_id="filtered"),
        {**detection("red", 30, 180, detection_id="unprojected"), "robot": None},
        detection("green", 40, 180, detection_id="unknown"),
    ]

    result = filter_sorting_detections(config, detections, profiles, settings)

    assert [candidate["detection_id"] for candidate in result["candidates"]] == ["ok"]
    assert {item["reason_code"] for item in result["ignored"]} == {
        "low_confidence",
        "color_filtered",
        "no_robot_coordinates",
        "color_filtered",
    }


def test_plan_uses_configured_tcp_z_phi_and_linear_near_object_modes():
    config = load_config(EXAMPLE_CONFIG_PATH)
    plan = build_color_sorting_plan(
        config,
        [detection("red", 0, 180, detection_id="r1")],
        color_profiles(config),
        task_settings={
            "execution_strategy": "batch_once",
            "orientation_policy": "fixed",
            "pickup_z_mm": 31.0,
            "dropoff_z_mm": 47.0,
            "approach_clearance_mm": 12.0,
            "drop_approach_clearance_mm": 9.0,
            "pickup_phi_deg": 5.0,
            "drop_phi_deg": -6.0,
        },
    )

    assert plan["ok"]
    targets = {
        step["label"]: step["waypoint"]["target"]
        for step in plan["steps"]
        if step["kind"] == "move" and step["waypoint"]["type"] == "cartesian"
    }
    modes = {
        step["label"]: step["waypoint"]["mode"]
        for step in plan["steps"]
        if step["kind"] == "move" and step["waypoint"]["type"] == "cartesian"
    }
    assert targets["above pickup"]["z_mm"] == 43.0
    assert targets["pickup"]["z_mm"] == 31.0
    assert targets["pickup"]["phi_deg"] == 5.0
    assert targets["above dropoff"]["z_mm"] == 56.0
    assert targets["dropoff"]["z_mm"] == 47.0
    assert targets["dropoff"]["phi_deg"] == -6.0
    assert modes["above pickup"] == "joint"
    assert modes["pickup"] == "linear"
    assert modes["lift"] == "linear"
    assert modes["above dropoff"] == "joint"
    assert modes["dropoff"] == "linear"


def test_every_generated_move_exposes_frame_mode_height_and_recovery_phase():
    config = load_config(EXAMPLE_CONFIG_PATH)
    plan = build_color_sorting_plan(
        config,
        [detection("red", 0, 180, detection_id="r1")],
        color_profiles(config),
        task_settings={"execution_strategy": "batch_once"},
    )

    assert plan["ok"]
    moves = [step for step in plan["steps"] if step["kind"] == "move"]
    assert moves
    for step in moves:
        assert step["target_frame"] == "robot_base"
        assert step["movement_mode"] in {"joint", "linear"}
        assert isinstance(step["height_mm"], float)
        assert step["phase"]
        assert isinstance(step["safe_retreat_available"], bool)


def test_per_color_object_profile_overrides_z_and_phi():
    config = load_config(EXAMPLE_CONFIG_PATH)
    plan = build_color_sorting_plan(
        config,
        [detection("red", -120, 150, detection_id="r1")],
        color_profiles(config),
        task_settings={
            "execution_strategy": "batch_once",
            "orientation_policy": "fixed",
            "object_profiles": {
                "red": {
                    "pickup_z_mm": 33.0,
                    "dropoff_z_mm": 52.0,
                    "pickup_phi_deg": 11.0,
                    "drop_phi_deg": -12.0,
                }
            },
        },
    )

    assert plan["ok"]
    obj = plan["objects"][0]
    assert obj["object_target"]["z_mm"] == 33.0
    assert obj["object_target"]["phi_deg"] == 11.0
    assert obj["drop_target"]["z_mm"] == 52.0
    assert obj["drop_target"]["phi_deg"] == -12.0


def test_prefer_downward_targets_use_auto_phi_with_preference():
    config = load_config(EXAMPLE_CONFIG_PATH)
    plan = build_color_sorting_plan(
        config,
        [detection("red", -120, 150, detection_id="r1")],
        color_profiles(config),
        task_settings={"execution_strategy": "batch_once", "orientation_policy": "prefer_downward"},
    )

    assert plan["ok"]
    obj = plan["objects"][0]
    assert obj["object_target"]["phi_auto"] is True
    assert obj["object_target"]["preferred_phi_deg"] == -90.0
    assert obj["drop_target"]["phi_auto"] is True
    assert obj["drop_target"]["preferred_phi_deg"] == -90.0


def test_missing_drop_zone_errors_only_for_detected_relevant_color():
    config = load_config(EXAMPLE_CONFIG_PATH)
    profiles = {
        **color_profiles(config),
        "green": {"enabled": True, "drop_zone": "missing_zone"},
    }

    red_only = build_color_sorting_plan(
        config,
        [detection("red", 0, 180)],
        profiles,
        task_settings={"execution_strategy": "batch_once"},
    )
    with_missing = build_color_sorting_plan(
        config,
        [detection("green", 0, 180)],
        profiles,
        task_settings={"execution_strategy": "batch_once", "missing_drop_zone_policy": "error"},
    )
    ignore_missing = build_color_sorting_plan(
        config,
        [detection("red", 0, 180), detection("green", 10, 180)],
        profiles,
        task_settings={"execution_strategy": "batch_once", "missing_drop_zone_policy": "ignore"},
    )

    assert red_only["ok"]
    assert not with_missing["ok"]
    assert "missing drop zone" in with_missing["error"]
    assert ignore_missing["ok"]
    assert any(item["reason_code"] == "missing_drop_zone" for item in ignore_missing["ignored_detections"])


def test_fixed_and_grid_assignment_capacity_are_deterministic():
    config = load_config(EXAMPLE_CONFIG_PATH)
    profiles = color_profiles(config)
    fixed = build_color_sorting_plan(
        config,
        [detection("red", 0, 180, detection_id="a"), detection("red", 20, 180, detection_id="b")],
        profiles,
        task_settings={"execution_strategy": "batch_once", "ordering": {"policy": "left_to_right"}},
    )
    assert fixed["ok"]
    assert [item["detection_id"] for item in fixed["objects"]] == ["a", "b"]
    assert fixed["objects"][0]["drop_target"]["x_mm"] == fixed["objects"][1]["drop_target"]["x_mm"]

    raw = {
        **config.raw,
        "task_destinations": {
            **config.raw["task_destinations"],
            "destinations": {
                **config.raw["task_destinations"]["destinations"],
                "dropoff_a": {
                    "x_mm": -120.0,
                    "y_mm": 180.0,
                    "z_mm": 45.0,
                    "phi_deg": 0.0,
                    "grid": {"rows": 1, "columns": 1, "x_spacing_mm": 10.0, "y_spacing_mm": 10.0},
                },
            },
        },
    }
    patched = replace(config, raw=raw)
    grid_overflow = build_color_sorting_plan(
        patched,
        [detection("red", 0, 180, detection_id="a"), detection("red", 20, 180, detection_id="b")],
        profiles,
        task_settings={"execution_strategy": "batch_once", "placement_policy": "grid"},
    )

    assert not grid_overflow["ok"]
    assert "capacity exceeded" in grid_overflow["error"]


def test_manual_selection_pauses_until_detection_id_is_selected():
    config = load_config(EXAMPLE_CONFIG_PATH)
    detections = [
        detection("red", 0, 180, detection_id="r1"),
        detection("blue", 20, 180, detection_id="b1"),
    ]
    waiting = build_color_sorting_plan(
        config,
        detections,
        color_profiles(config),
        task_settings={"execution_strategy": "batch_once", "ordering": {"policy": "manual"}},
    )
    selected = build_color_sorting_plan(
        config,
        detections,
        color_profiles(config),
        task_settings={"execution_strategy": "batch_once", "ordering": {"policy": "manual"}},
        selected_detection_ids=["b1"],
    )

    assert waiting["selection_required"] is True
    assert waiting["task_preview"]["candidate_objects"][0]["detection_id"] == "r1"
    assert selected["ok"]
    assert [item["detection_id"] for item in selected["objects"]] == ["b1"]


def test_closed_loop_preview_contains_one_reusable_cycle():
    config = load_config(EXAMPLE_CONFIG_PATH)
    plan = build_color_sorting_plan(
        config,
        [detection("red", 0, 180, detection_id="r1"), detection("blue", 20, 180, detection_id="b1")],
        color_profiles(config),
        task_settings={"execution_strategy": "closed_loop", "ordering": {"policy": "left_to_right"}, "max_objects": 5},
    )

    assert plan["ok"]
    assert plan["object_count"] == 1
    assert plan["task_preview"]["strategy"] == "closed_loop"
    assert plan["task_preview"]["next_object"]["detection_id"] == "r1"
    assert len(plan["task_preview"]["candidate_objects"]) == 2


def test_program_preview_error_names_failing_task_waypoint():
    config = load_config(EXAMPLE_CONFIG_PATH)
    trajectory = build_program_trajectory(
        config.home_pose,
        [
            {
                "type": "cartesian",
                "mode": "joint",
                "label": "above pickup",
                "target": {"x_mm": 0.0, "y_mm": 180.0, "z_mm": 80.0, "phi_auto": True, "preferred_phi_deg": -90.0},
            }
        ],
        config.links,
        config.joints,
        {"planner_type": "s_curve", "global_speed_deg_s": 25.0, "global_accel_deg_s2": 120.0},
        "auto",
    )

    assert not trajectory["ok"]
    assert "above pickup" in trajectory["errors"][0]
    assert "x 0.0, y 180.0, z 80.0" in trajectory["errors"][0]


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"max_objects": 0}, "max_objects must be at least 1"),
        ({"pickup_z_mm": -1}, "pickup_z_mm must be zero or greater"),
        ({"dropoff_z_mm": "bad"}, "dropoff_z_mm must be a finite number"),
        ({"filters": {"min_confidence": 1.1}}, "filters.min_confidence must be between 0 and 1"),
        ({"ordering": {"policy": "random"}}, "ordering.policy must be one of"),
        ({"motion_modes": {"transfer": "spline"}}, "motion_modes.transfer must be one of"),
        ({"object_profiles": []}, "object_profiles must be a JSON object"),
    ],
)
def test_invalid_task_settings_are_rejected_instead_of_silently_coerced(overrides, message):
    config = load_config(EXAMPLE_CONFIG_PATH)

    with pytest.raises(ValueError, match=message):
        normalize_color_sorting_settings(config, overrides)


def test_missing_safe_position_is_a_planning_error():
    config = load_config(EXAMPLE_CONFIG_PATH)

    plan = build_color_sorting_plan(
        config,
        [detection("red", -120, 150)],
        color_profiles(config),
        task_settings={"execution_strategy": "batch_once", "safe_position": "missing"},
    )

    assert not plan["ok"]
    assert plan["error"] == "safe position missing is missing or invalid"


def test_missing_active_tool_preset_is_a_planning_error():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = {
        **config.raw,
        "tools": {
            "active": "vacuum",
            "presets": config.raw["tools"]["presets"],
        },
    }

    plan = build_color_sorting_plan(
        replace(config, raw=raw),
        [detection("red", -120, 150)],
        color_profiles(config),
        task_settings={"execution_strategy": "batch_once"},
    )

    assert not plan["ok"]
    assert "active tool vacuum is missing" in plan["error"]


def test_closed_loop_only_requires_mappings_for_colors_in_the_fresh_capture():
    config = load_config(EXAMPLE_CONFIG_PATH)
    profiles = {
        **color_profiles(config),
        "green": {"enabled": True, "drop_zone": "missing_zone"},
    }

    plan = build_color_sorting_plan(
        config,
        [detection("red", -120, 150)],
        profiles,
        task_settings={"execution_strategy": "closed_loop", "missing_drop_zone_policy": "error"},
    )

    assert plan["ok"]
    assert plan["objects"][0]["color"] == "red"


def test_batch_mode_only_requires_drop_zones_for_relevant_detections():
    config = load_config(EXAMPLE_CONFIG_PATH)
    profiles = {
        **color_profiles(config),
        "green": {"enabled": True, "drop_zone": "missing_zone"},
    }

    plan = build_color_sorting_plan(
        config,
        [detection("red", -120, 150)],
        profiles,
        task_settings={"execution_strategy": "batch_once", "missing_drop_zone_policy": "error"},
    )

    assert plan["ok"]


def test_preview_warns_when_tool_dimensions_are_not_hardware_validated():
    config = load_config(EXAMPLE_CONFIG_PATH)

    plan = build_color_sorting_plan(
        config,
        [detection("red", -120, 150)],
        color_profiles(config),
        task_settings={"execution_strategy": "batch_once"},
    )

    assert plan["ok"]
    assert "active tool dimensions are not validated for hardware" in plan["task_preview"]["warnings"]


def test_nonnumeric_detection_coordinates_are_rejected():
    config = load_config(EXAMPLE_CONFIG_PATH)
    bad = detection("red", -120, 150)
    bad["robot"]["x_mm"] = "not-a-number"

    plan = build_color_sorting_plan(
        config,
        [bad],
        color_profiles(config),
        task_settings={"execution_strategy": "batch_once"},
    )

    assert not plan["ok"]
    assert plan["task_preview"]["ignored_detections"][0]["reason_code"] == "no_robot_coordinates"


def test_invalid_drop_zone_coordinates_are_reported_instead_of_becoming_zero():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = {
        **config.raw,
        "task_destinations": {
            **config.raw["task_destinations"],
            "destinations": {
                **config.raw["task_destinations"]["destinations"],
                "dropoff_a": {
                    "label": "Dropoff A",
                    "x_mm": "not-a-number",
                    "y_mm": 180.0,
                    "z_mm": 45.0,
                    "phi_deg": 0.0,
                },
            },
        },
    }

    plan = build_color_sorting_plan(
        replace(config, raw=raw),
        [detection("red", -120, 150)],
        color_profiles(config),
        task_settings={"execution_strategy": "batch_once"},
    )

    assert not plan["ok"]
    assert "task destination dropoff_a x_mm must be a finite number" in plan["error"]
