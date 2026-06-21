from __future__ import annotations

from math import hypot

import pytest

from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.motion import build_program_trajectory
from app.program_library import (
    ProgramLibraryError,
    all_programs,
    built_in_programs,
    copy_program_to_user,
    delete_user_program,
    find_program,
    load_user_programs,
    program_motion_fingerprint,
    save_user_program,
    save_user_program_cached_plan,
)


def reference_config():
    return load_config(EXAMPLE_CONFIG_PATH)


def path_settings(config):
    return {
        "global_speed_deg_s": 30.0,
        "global_accel_deg_s2": 120.0,
        "tcp_speed_mm_s": 45.0,
        "phi_speed_deg_s": 80.0,
        "tcp_accel_mm_s2": 160.0,
        "phi_accel_deg_s2": 240.0,
        "waypoint_rate_hz": 30.0,
        "cartesian_step_mm": 5.0,
        "planner_type": "s_curve",
        "jerk_percent": 45.0,
        "blend_percent": 0.0,
        "per_joint_speed_deg_s": [joint.max_speed_deg_s for joint in config.joints],
        "per_joint_accel_deg_s2": [joint.max_accel_deg_s2 for joint in config.joints],
    }


def test_user_program_crud_persists_across_reload(tmp_path):
    config = reference_config()
    store = tmp_path / "programs.local.json"
    saved = save_user_program(
        config,
        {
            "name": "Inspection Move",
            "description": "Small persisted program",
            "steps": [
                {
                    "id": "step-1",
                    "label": "Home",
                    "type": "joint",
                    "mode": "joint",
                    "angles_deg": config.home_pose,
                }
            ],
        },
        store,
    )

    reloaded = load_user_programs(config, store)
    assert reloaded[saved["id"]]["name"] == "Inspection Move"
    assert reloaded[saved["id"]]["steps"][0]["angles_deg"] == config.home_pose

    updated = save_user_program(
        config,
        {
            **saved,
            "name": "Inspection Move Revised",
        },
        store,
    )
    assert updated["id"] == saved["id"]
    assert load_user_programs(config, store)[saved["id"]]["name"].endswith("Revised")

    assert delete_user_program(config, saved["id"], store)
    assert load_user_programs(config, store) == {}


def test_built_ins_are_read_only_and_copy_to_editable_user_program(tmp_path):
    config = reference_config()
    store = tmp_path / "programs.local.json"
    built_in = find_program(config, "demo-air-square", store)

    assert built_in is not None
    assert built_in["read_only"]
    assert built_in["template"]
    with pytest.raises(ProgramLibraryError, match="read-only"):
        save_user_program(config, {**built_in, "name": "Mutated Demo"}, store)
    with pytest.raises(ProgramLibraryError, match="cannot be deleted"):
        delete_user_program(config, built_in["id"], store)

    copied = copy_program_to_user(config, built_in["id"], path=store)
    assert not copied["read_only"]
    assert not copied["template"]
    assert copied["metadata"]["copied_from"] == built_in["id"]
    assert copied["id"] in load_user_programs(config, store)


def test_built_in_demo_programs_adapt_and_preview_against_reference_robot():
    config = reference_config()
    demos = {program["id"]: program for program in built_in_programs(config)}

    assert set(demos) == {
        "demo-air-square",
        "demo-air-circle",
        "demo-kinematic-showcase",
    }
    for demo in demos.values():
        trajectory = build_program_trajectory(
            config.home_pose,
            demo["steps"],
            config.links,
            config.joints,
            path_settings(config),
            "auto",
        )
        assert trajectory["ok"], trajectory.get("errors")
        assert trajectory["duration_s"] > 0
        assert trajectory["waypoint_count"] > 1

    circle_targets = [
        step["target"]
        for step in demos["demo-air-circle"]["steps"]
        if step["type"] == "cartesian"
    ]
    center_x = sum(target["x_mm"] for target in circle_targets[:-1]) / (len(circle_targets) - 1)
    center_z = sum(target["z_mm"] for target in circle_targets[:-1]) / (len(circle_targets) - 1)
    radii = [hypot(target["x_mm"] - center_x, target["z_mm"] - center_z) for target in circle_targets]
    assert max(radii) - min(radii) < 0.01
    assert circle_targets[0] == circle_targets[-1]

    square_targets = [
        step["target"]
        for step in demos["demo-air-square"]["steps"]
        if step["type"] == "cartesian"
    ]
    assert square_targets[0] == square_targets[-1]
    assert len({target["x_mm"] for target in square_targets}) == 2
    assert len({target["z_mm"] for target in square_targets}) == 2


def test_program_listing_keeps_built_ins_separate_from_user_records(tmp_path):
    config = reference_config()
    store = tmp_path / "programs.local.json"
    save_user_program(
        config,
        {
            "name": "User Move",
            "steps": [
                {
                    "label": "Home",
                    "type": "joint",
                    "angles_deg": config.home_pose,
                }
            ],
        },
        store,
    )

    programs = all_programs(config, store)
    assert len([program for program in programs if program["read_only"]]) == 3
    assert len([program for program in programs if not program["read_only"]]) == 1


def test_program_library_preserves_motion_overrides_and_tool_steps(tmp_path):
    config = reference_config()
    store = tmp_path / "programs.local.json"

    saved = save_user_program(
        config,
        {
            "name": "Pick sequence",
            "steps": [
                {
                    "label": "Slow approach",
                    "type": "cartesian",
                    "mode": "linear",
                    "target": {"x_mm": 0.0, "y_mm": 250.0, "z_mm": 120.0, "phi_auto": True},
                    "settings": {
                        "tcp_speed_mm_s": 20.0,
                        "tcp_accel_mm_s2": 80.0,
                    },
                },
                {
                    "label": "Close",
                    "type": "tool",
                    "tool": "gripper",
                    "action": "close",
                    "settle_ms": 200,
                },
            ],
        },
        store,
    )

    assert saved["steps"][0]["settings"] == {
        "tcp_speed_mm_s": 20.0,
        "tcp_accel_mm_s2": 80.0,
    }
    assert saved["steps"][1]["type"] == "tool"
    assert saved["steps"][1]["action"] == "close"
    assert saved["steps"][1]["settle_ms"] == 200.0


def test_user_program_persists_compiled_plan_with_definition_fingerprint(tmp_path):
    config = reference_config()
    store = tmp_path / "programs.local.json"
    saved = save_user_program(
        config,
        {
            "name": "Reusable move",
            "steps": [
                {
                    "label": "Home",
                    "type": "joint",
                    "angles_deg": config.home_pose,
                }
            ],
        },
        store,
    )

    with_plan = save_user_program_cached_plan(
        config,
        saved["id"],
        {
            "backend_build_id": "test-build",
            "config_id": "test-config",
            "model_fingerprint": "test-model",
            "start_reported_angles_deg": config.home_pose,
            "preview": {
                "mode": "program",
                "trajectory": {
                    "duration_s": 1.0,
                    "waypoint_count": 2,
                    "waypoints": [config.home_pose, config.home_pose],
                },
            },
        },
        store,
    )

    reloaded = load_user_programs(config, store)[saved["id"]]
    assert reloaded["cached_plan"]["backend_build_id"] == "test-build"
    assert reloaded["cached_plan"]["program_fingerprint"] == program_motion_fingerprint(with_plan)
    assert reloaded["cached_plan"]["preview"]["trajectory"]["waypoint_count"] == 2

    updated = save_user_program(config, {**saved, "description": "changed"}, store)
    assert "cached_plan" not in updated
