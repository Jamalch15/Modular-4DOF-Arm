from __future__ import annotations

from copy import deepcopy

from pytest import approx

from app.cartesian_calibration import calibration_context, calibration_settings
from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.kinematics import forward_kinematics
from app.physical_model_calibration import (
    _links_with_deltas,
    fit_physical_model,
    physical_model_updates,
)


FIT_POSES = [
    [-70.0, 20.0, -80.0, 40.0],
    [-45.0, 35.0, -50.0, 20.0],
    [-20.0, 55.0, -35.0, -30.0],
    [10.0, 25.0, 20.0, -65.0],
    [35.0, 45.0, 10.0, 20.0],
    [65.0, 60.0, -45.0, 45.0],
    [-80.0, 75.0, -20.0, -35.0],
    [-35.0, 95.0, -60.0, 10.0],
    [20.0, 80.0, 25.0, -50.0],
    [55.0, 30.0, 45.0, -70.0],
]

VALIDATION_POSES = [
    [-55.0, 65.0, -15.0, -20.0],
    [0.0, 40.0, -70.0, 55.0],
    [45.0, 90.0, -30.0, -45.0],
]


def _sample(config, true_links, pose, role, index):
    predicted = forward_kinematics(pose, config.links)
    measured = forward_kinematics(pose, true_links)
    return {
        "id": f"{role}-{index}",
        "role": role,
        "quality": 1.0,
        "tool": "gripper",
        "context": calibration_context(config),
        "reported_joints_deg": list(pose),
        "fk_predicted": {
            "x_mm": predicted["x_mm"],
            "y_mm": predicted["y_mm"],
            "z_mm": predicted["z_mm"],
            "phi_deg": predicted["tool_phi_deg"],
        },
        "measured": {
            "x_mm": measured["x_mm"],
            "y_mm": measured["y_mm"],
            "z_mm": measured["z_mm"],
        },
        "intended_target": {
            "x_mm": predicted["x_mm"],
            "y_mm": predicted["y_mm"],
            "z_mm": predicted["z_mm"],
        },
        "residuals": {
            "model_mm": {
                "x": measured["x_mm"] - predicted["x_mm"],
                "y": measured["y_mm"] - predicted["y_mm"],
                "z": measured["z_mm"] - predicted["z_mm"],
            },
            "ik_target_mm": {"xyz": 0.0},
        },
    }


def test_joint_zero_fit_recovers_synthetic_model_and_passes_validation():
    config = load_config(EXAMPLE_CONFIG_PATH)
    names = ["joint_1_zero_deg", "joint_2_zero_deg", "joint_3_zero_deg", "joint_4_zero_deg"]
    actual = [0.8, -1.2, 0.9, -0.6]
    true_links = _links_with_deltas(config, names, actual)
    samples = [
        *[_sample(config, true_links, pose, "fit", index) for index, pose in enumerate(FIT_POSES)],
        *[
            _sample(config, true_links, pose, "validation", index)
            for index, pose in enumerate(VALIDATION_POSES)
        ],
    ]
    profile = {
        "tool": "gripper",
        "context": calibration_context(config),
        "samples": samples,
    }

    result = fit_physical_model(
        profile,
        config,
        parameter_group="joint_zeros",
        thresholds=calibration_settings(config)["thresholds"],
    )

    fitted = {item["name"]: item["delta"] for item in result["parameters"]}
    assert fitted["joint_1_zero_deg"] == approx(actual[0], abs=0.05)
    assert fitted["joint_2_zero_deg"] == approx(actual[1], abs=0.05)
    assert fitted["joint_3_zero_deg"] == approx(actual[2], abs=0.05)
    assert fitted["joint_4_zero_deg"] == approx(actual[3], abs=0.05)
    assert result["validation"]["after"]["xyz_rmse_mm"] < 0.2
    assert result["safe_to_apply"] is True


def test_physical_model_updates_existing_dh_and_geometry_sources():
    config = load_config(EXAMPLE_CONFIG_PATH)
    result = {
        "id": "candidate",
        "safe_to_apply": True,
        "parameter_group": "joint_zeros_geometry",
        "parameters": [
            {"name": "joint_2_zero_deg", "delta": 1.5, "unit": "deg"},
            {"name": "base_height_mm", "delta": 4.0, "unit": "mm"},
            {"name": "base_side_offset_mm", "delta": -2.0, "unit": "mm"},
            {"name": "upper_arm_mm", "delta": 3.0, "unit": "mm"},
            {"name": "forearm_mm", "delta": -1.0, "unit": "mm"},
        ],
    }

    updates = physical_model_updates(config, deepcopy(result))

    assert updates["kinematics"]["dh_rows"][1]["zero_offset_deg"] == approx(1.5)
    assert updates["kinematics"]["dh_rows"][0]["d_mm"] == approx(
        config.kinematics.dh_rows[0].d_mm + 4.0
    )
    dimensions = updates["geometry"]["presets"]["matlab_prototype"]["dimensions_mm"]
    assert dimensions["L_1"] == approx(97.45)
    assert dimensions["L_2"] == approx(21.2)
    assert dimensions["L_5"] == approx(163.15)
    assert dimensions["L_7"] == approx(141.55)
    assert updates["calibration"]["physical_model_history"][-1]["result_id"] == "candidate"
