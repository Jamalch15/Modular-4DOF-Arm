from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from shutil import copyfile

import pytest
from fastapi.testclient import TestClient
from pytest import approx

from app import main
from app.cartesian_calibration import (
    calibration_context,
    calibration_settings,
    correct_cartesian_target,
    create_sample,
    fit_profile,
)
from app.config import EXAMPLE_CONFIG_PATH, load_config, save_calibration_updates
from app.kinematics import forward_kinematics


def fitted_constant_settings(
    *,
    enabled: bool = True,
    offset: tuple[float, float, float] = (10.0, -5.0, 2.0),
) -> dict:
    return {
        "schema_version": 2,
        "enabled": enabled,
        "active_profile": "gripper",
        "default_model": "constant_xyz",
        "profiles": {
            "gripper": {
                "tool": "gripper",
                "enabled": enabled,
                "model_type": "constant_xyz",
                "activation": {"eligible": True, "reasons": []},
                "samples": [],
                "result": {
                    "id": "test-result",
                    "model_type": "constant_xyz",
                    "coefficients": {
                        "xy_matrix": [[1.0, 0.0], [0.0, 1.0]],
                        "xy_offset_mm": [offset[0], offset[1]],
                        "z_offset_mm": offset[2],
                    },
                    "fit": {
                        "status": "pass",
                        "after_model": {
                            "count": 4,
                            "xy_rmse_mm": 0.5,
                            "xy_max_mm": 1.0,
                            "z_rmse_mm": 0.2,
                            "z_max_abs_mm": 0.4,
                        },
                    },
                    "validation": {"status": "pass", "landing_status": "pass"},
                },
            }
        },
    }


def config_with_calibration(tmp_path: Path, settings: dict):
    target = tmp_path / "robot.yaml"
    copyfile(EXAMPLE_CONFIG_PATH, target)
    base_config = load_config(target)
    settings = deepcopy(settings)
    for key, profile in settings.get("profiles", {}).items():
        profile["context"] = calibration_context(base_config, key)
    save_calibration_updates(target, {"kinematics_calibration": settings})
    return load_config(target), target


def calibration_sample(
    sample_id: str,
    expected: tuple[float, float, float],
    measured: tuple[float, float, float],
    *,
    role: str = "fit",
    context: dict | None = None,
) -> dict:
    return {
        "id": sample_id,
        "role": role,
        "quality": 1.0,
        "context": deepcopy(context),
        "fk_predicted": {
            "x_mm": expected[0],
            "y_mm": expected[1],
            "z_mm": expected[2],
        },
        "measured": {
            "x_mm": measured[0],
            "y_mm": measured[1],
            "z_mm": measured[2],
        },
        "intended_target": {
            "x_mm": expected[0],
            "y_mm": expected[1],
            "z_mm": expected[2],
        },
        "residuals": {
            "model_mm": {
                "x": measured[0] - expected[0],
                "y": measured[1] - expected[1],
                "z": measured[2] - expected[2],
            },
            "ik_target_mm": {"xyz": 0.0},
        },
    }


def test_disabled_or_absent_calibration_is_a_noop(tmp_path):
    config = load_config(EXAMPLE_CONFIG_PATH)
    target = {"x_mm": 100.0, "y_mm": 200.0, "z_mm": 40.0, "phi_deg": -90.0}

    absent_command, absent_metadata = correct_cartesian_target(target, config)
    disabled_config, _ = config_with_calibration(
        tmp_path,
        fitted_constant_settings(enabled=False),
    )
    disabled_command, disabled_metadata = correct_cartesian_target(target, disabled_config)

    assert absent_command == target
    assert absent_metadata["applied"] is False
    assert disabled_command == target
    assert disabled_metadata["applied"] is False
    assert disabled_metadata["reason"] == "disabled"


def test_enabled_calibration_inverse_shifts_cartesian_command(tmp_path):
    config, _ = config_with_calibration(tmp_path, fitted_constant_settings())
    intended = {"x_mm": 100.0, "y_mm": 200.0, "z_mm": 40.0, "phi_deg": -90.0}

    command, metadata = correct_cartesian_target(intended, config)

    assert metadata["applied"] is True
    assert command["x_mm"] == approx(90.0)
    assert command["y_mm"] == approx(205.0)
    assert command["z_mm"] == approx(38.0)
    assert command["phi_deg"] == intended["phi_deg"]


def test_stale_model_signature_blocks_correction(tmp_path):
    config, target_path = config_with_calibration(tmp_path, fitted_constant_settings())
    data = deepcopy(config.raw)
    data["kinematics"]["dh_rows"][0]["d_mm"] += 5.0
    save_calibration_updates(
        target_path,
        {"kinematics": {"dh_rows": data["kinematics"]["dh_rows"]}},
    )
    changed = load_config(target_path)

    intended = {"x_mm": 100.0, "y_mm": 200.0, "z_mm": 40.0, "phi_deg": -90.0}
    command, metadata = correct_cartesian_target(intended, changed)

    assert command == intended
    assert metadata["applied"] is False
    assert metadata["reason"] == "stale_profile"
    assert any("geometry" in warning for warning in metadata["warnings"])


def test_unvalidated_profile_cannot_apply_normal_commands_but_can_run_validation_trial(tmp_path):
    settings = fitted_constant_settings()
    settings["profiles"]["gripper"]["activation"] = {
        "eligible": False,
        "reasons": ["held-out validation required"],
    }
    config, _ = config_with_calibration(tmp_path, settings)
    intended = {"x_mm": 100.0, "y_mm": 200.0, "z_mm": 40.0, "phi_deg": -90.0}

    normal, normal_metadata = correct_cartesian_target(intended, config)
    trial, trial_metadata = correct_cartesian_target(intended, config, validation_trial=True)

    assert normal == intended
    assert normal_metadata["reason"] == "validation_required"
    assert trial["x_mm"] == approx(90.0)
    assert trial_metadata["applied"] is True
    assert trial_metadata["reason"] == "validation_trial"


def test_calibration_persists_through_config_reload(tmp_path):
    config, target = config_with_calibration(tmp_path, fitted_constant_settings())

    reloaded = load_config(target)
    settings = calibration_settings(reloaded)

    assert config.raw["kinematics_calibration"]["enabled"] is True
    assert settings["enabled"] is True
    assert settings["profiles"]["gripper"]["result"]["id"] == "test-result"


def test_invalid_or_incomplete_samples_are_rejected():
    config = load_config(EXAMPLE_CONFIG_PATH)
    fk = forward_kinematics(config.home_pose, config.links)

    with pytest.raises(ValueError, match="measured is required"):
        create_sample(
            {
                "intended_target": {
                    "x_mm": fk["x_mm"],
                    "y_mm": fk["y_mm"],
                    "z_mm": fk["z_mm"],
                }
            },
            config,
            config.home_pose,
            fk,
        )

    with pytest.raises(ValueError, match="quality"):
        create_sample(
            {
                "intended_target": {
                    "x_mm": fk["x_mm"],
                    "y_mm": fk["y_mm"],
                    "z_mm": fk["z_mm"],
                },
                "measured": {
                    "x_mm": fk["x_mm"],
                    "y_mm": fk["y_mm"],
                    "z_mm": fk["z_mm"],
                },
                "quality": 0.05,
            },
            config,
            config.home_pose,
            fk,
        )


def test_affine_fit_rejects_outlier_and_reports_validation_metrics():
    config = load_config(EXAMPLE_CONFIG_PATH)
    settings = calibration_settings(config)
    context = calibration_context(config)
    transform = lambda x, y, z: (1.02 * x + 0.01 * y + 8.0, -0.02 * x + 0.98 * y - 4.0, z + 3.0)
    fit_points = [
        (-150.0, 120.0, 35.0),
        (0.0, 120.0, 35.0),
        (150.0, 120.0, 35.0),
        (-150.0, 300.0, 55.0),
        (0.0, 300.0, 55.0),
        (150.0, 300.0, 55.0),
    ]
    samples = [
        calibration_sample(f"fit-{index}", point, transform(*point), context=context)
        for index, point in enumerate(fit_points)
    ]
    samples.append(
        calibration_sample(
            "outlier",
            (30.0, 220.0, 45.0),
            (180.0, 20.0, 130.0),
            context=context,
        )
    )
    validation_point = (80.0, 210.0, 45.0)
    samples.append(
        calibration_sample(
            "validation-1",
            validation_point,
            transform(*validation_point),
            role="validation",
            context=context,
        )
    )
    settings["profiles"] = {
        "gripper": {
            "tool": "gripper",
            "enabled": False,
            "model_type": "affine_xy_z_offset",
            "samples": samples,
        }
    }
    settings["active_profile"] = "gripper"

    updated, result = fit_profile(settings, config, model_type="affine_xy_z_offset")

    assert "outlier" in result["rejected_sample_ids"]
    assert result["fit"]["status"] == "pass"
    assert result["validation"]["status"] == "pass"
    coefficients = updated["profiles"]["gripper"]["result"]["coefficients"]
    assert coefficients["xy_matrix"][0] == approx([1.02, 0.01], abs=1e-6)
    assert coefficients["xy_matrix"][1] == approx([-0.02, 0.98], abs=1e-6)
    assert coefficients["xy_offset_mm"] == approx([8.0, -4.0], abs=1e-6)
    assert coefficients["z_offset_mm"] == approx(3.0)


def test_preview_applies_calibration_but_keeps_requested_target_visible(monkeypatch, tmp_path):
    config, _ = config_with_calibration(
        tmp_path,
        fitted_constant_settings(offset=(4.0, -3.0, 1.5)),
    )
    monkeypatch.setattr(main, "config", config)
    main.state.reported_angles_deg = [-45.0, 0.0, -45.0, -45.0]
    raw_fk = forward_kinematics([-40.0, 5.0, -35.0, -40.0], config.links)
    intended = {
        "x_mm": raw_fk["x_mm"],
        "y_mm": raw_fk["y_mm"],
        "z_mm": raw_fk["z_mm"],
        "phi_deg": raw_fk["tool_phi_deg"],
    }

    preview = main.build_preview(
        mode="joint",
        target=intended,
        waypoint_program=None,
        links=config.links,
        settings=main.request_settings(None),
        branch="auto",
    )

    assert preview["ok"], preview.get("error")
    assert preview["preview"]["target"]["x_mm"] == approx(intended["x_mm"])
    assert preview["preview"]["command_target"]["x_mm"] == approx(intended["x_mm"] - 4.0)
    assert preview["preview"]["command_target"]["y_mm"] == approx(intended["y_mm"] + 3.0)
    assert preview["preview"]["calibration"]["applied"] is True


def test_program_calibrates_cartesian_waypoints_without_changing_joint_waypoints(monkeypatch, tmp_path):
    config, _ = config_with_calibration(
        tmp_path,
        fitted_constant_settings(offset=(3.0, 2.0, -1.0)),
    )
    monkeypatch.setattr(main, "config", config)
    start = [-45.0, 0.0, -45.0, -45.0]
    main.state.reported_angles_deg = list(start)
    joint_target = [-40.0, 5.0, -40.0, -40.0]
    cartesian_fk = forward_kinematics([-35.0, 8.0, -32.0, -38.0], config.links)
    intended = {
        "x_mm": cartesian_fk["x_mm"],
        "y_mm": cartesian_fk["y_mm"],
        "z_mm": cartesian_fk["z_mm"],
        "phi_deg": cartesian_fk["tool_phi_deg"],
    }

    preview = main.build_preview(
        mode="program",
        target=None,
        waypoint_program=[
            {"type": "joint", "mode": "joint", "angles_deg": joint_target},
            {"type": "cartesian", "mode": "joint", "target": intended},
        ],
        links=config.links,
        settings=main.request_settings(None),
        branch="auto",
    )

    assert preview["ok"], preview.get("error")
    correction = preview["preview"]["calibration"]
    assert len(correction) == 1
    assert correction[0]["applied"] is True
    assert correction[0]["command_target"]["x_mm"] == approx(intended["x_mm"] - 3.0)
    assert preview["preview"]["trajectory"]["physical_cartesian_waypoints"]
    first_segment_end = preview["preview"]["trajectory"]["segments"][0]
    assert first_segment_end["type"] == "joint"


def test_automatic_targets_choose_z_and_phi_without_running_ik(monkeypatch, tmp_path):
    settings = fitted_constant_settings()
    config, _ = config_with_calibration(tmp_path, settings)
    monkeypatch.setattr(main, "config", config)
    main.state.reported_angles_deg = list(config.home_pose)
    monkeypatch.setattr(
        main,
        "inverse_kinematics",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("target generation must not run IK")),
    )
    client = TestClient(main.app)

    payload = client.post(
        "/api/kinematics-calibration/targets",
        json={
            "count": 12,
            "validation_stride": 4,
        },
    ).json()

    assert payload["ok"] is True
    assert payload["fit_quality"]["status"] == "pass"
    assert len(payload["points"]) == 12
    assert payload["reachability"] == {"reachable_count": 12, "unreachable_count": 0}
    assert payload["strategy"]["id"] == "model_aware_joint_pose_sampling"
    assert payload["strategy"]["coverage"]["z_span_mm"] >= 15.0
    assert payload["strategy"]["coverage"]["phi_span_deg"] >= 20.0
    assert {point["recommended_role"] for point in payload["points"]} == {"fit", "validation"}
    assert all(point["reachable"] for point in payload["points"])


def test_sample_capture_requires_one_executed_calibration_preview(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.path_previews.clear()
    final_angles = list(config.home_pose)
    main.state.reported_angles_deg = list(final_angles)
    intended_fk = forward_kinematics(final_angles, config.links)
    target = {
        "x_mm": intended_fk["x_mm"],
        "y_mm": intended_fk["y_mm"],
        "z_mm": intended_fk["z_mm"],
        "phi_deg": intended_fk["tool_phi_deg"],
    }
    request = main.KinematicsCalibrationSampleRequest(
        intended_target=target,
        command_target=target,
        measured=target,
        role="fit",
        preview_id="bound-preview",
    )

    with pytest.raises(ValueError, match="expired"):
        main._calibration_capture_contract(request)

    main.path_previews["bound-preview"] = {
        "id": "bound-preview",
        "source": "kinematics_calibration_fit",
        "target": target,
        "command_target": target,
        "trajectory": {"waypoints": [final_angles]},
        "execution_started_at": 123.0,
        "execution_count": 1,
        "model_fingerprint": main.robot_model_fingerprint(),
        "config_id": main.RUNNING_CONFIG_ID,
    }

    capture = main._calibration_capture_contract(request)

    assert capture["preview_id"] == "bound-preview"
    assert capture["reported_pose_is_estimated"] is True
