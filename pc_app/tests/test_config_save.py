from pathlib import Path
from shutil import copyfile

import yaml

from app.config import load_config, save_calibration_updates
from app.demo_settings import geometry_settings


def test_save_calibration_updates_values_and_preserves_comments(tmp_path):
    source = Path(__file__).resolve().parents[1] / "config" / "robot.example.yaml"
    target = tmp_path / "robot.yaml"
    copyfile(source, target)

    save_calibration_updates(
        target,
        {
            "links_mm": {"base_height": 81.5},
            "joints": [
                {
                    "limits_deg": {"min": -150.0, "max": 150.0},
                    "home_deg": 1.0,
                    "max_speed_deg_s": 44.0,
                    "max_accel_deg_s2": 111.0,
                    "zero_offset_deg": 2.0,
                    "direction_sign": -1,
                    "hardware": {
                        "stepper": {
                            "enabled": True,
                            "step_pin": 17,
                            "dir_pin": 16,
                            "enable_pin": -1,
                            "enable_active_low": True,
                            "m0_pin": 3,
                            "m1_pin": 8,
                            "m2_pin": 18,
                            "driver_model": "DRV8825",
                            "motor_full_steps_per_rev": 200,
                            "microsteps": 32,
                            "gear_ratio": 4.5,
                        }
                    },
                }
            ],
            "motion": {"command_rate_limit_hz": 14.0, "acceleration_deg_s2": 130.0},
            "path_defaults": {
                "global_speed_deg_s": 22.0,
                "cartesian_step_mm": 8.0,
                "planner_type": "trapezoid",
            },
            "camera": {
                "enabled": True,
                "source_index": 1,
                "resolution": {"width": 1280, "height": 720},
                "intrinsics": {
                    "source": "test",
                    "fx_px": 900.0,
                    "fy_px": 901.0,
                    "cx_px": 640.0,
                    "cy_px": 360.0,
                    "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0],
                },
                "calibration": {
                    "image_points": [],
                    "robot_points": [],
                    "apriltag": {
                        "dictionary": "DICT_APRILTAG_36H11",
                        "tag_size_mm": 40.0,
                        "result": {"accepted": True, "id": "test-pose"},
                    },
                },
            },
            "geometry": {
                "active_preset": "matlab_prototype",
                "presets": {
                    "matlab_prototype": {
                        "label": "MATLAB prototype",
                        "status": "working_assumption",
                        "source": "jacobian_ik_robotarm_analytic_seed.m",
                        "units": {"length": "mm", "angle": "deg"},
                        "dimensions_mm": {"L_1": 93.45, "L_2": 23.2, "L_3": 64.5},
                        "signs": {"s4": -1, "s6": -1, "s8": 1},
                    }
                },
            },
        },
    )

    text = target.read_text(encoding="utf-8")
    saved = load_config(target)

    assert "# Derived compatibility values" in text
    assert saved.links.base_height_mm == 81.5
    assert saved.joints[0].min_deg == -150.0
    assert saved.joints[0].max_deg == 150.0
    assert saved.joints[0].home_deg == 1.0
    assert saved.joints[0].max_speed_deg_s == 44.0
    assert saved.joints[0].max_accel_deg_s2 == 111.0
    assert saved.joints[0].zero_offset_deg == 2.0
    assert saved.joints[0].direction_sign == -1
    assert saved.raw["named_positions"]["home"]["angles_deg"] == [1.0, 20.0, 20.0, 0.0]
    assert saved.joints[0].hardware.stepper.enabled is True
    assert saved.joints[0].hardware.stepper.step_pin == 17
    assert saved.joints[0].hardware.stepper.dir_pin == 16
    assert saved.joints[0].hardware.stepper.microsteps == 32
    assert saved.joints[0].hardware.stepper.gear_ratio == 4.5
    assert saved.motion.command_rate_limit_hz == 14.0
    assert saved.motion.acceleration_deg_s2 == 130.0
    assert saved.raw["path_defaults"]["global_speed_deg_s"] == 22.0
    assert saved.raw["path_defaults"]["planner_type"] == "trapezoid"
    assert saved.raw["camera"]["source_index"] == 1
    assert saved.raw["camera"]["intrinsics"]["fx_px"] == 900.0
    assert saved.raw["camera"]["calibration"]["apriltag"]["result"]["id"] == "test-pose"
    geometry = geometry_settings(saved)
    assert geometry["presets"]["matlab_prototype"]["dimensions_mm"]["L_2"] == 23.2
    assert geometry["presets"]["matlab_prototype"]["signs"]["s4"] == -1


def test_load_config_keeps_joint_index_zero_based(tmp_path):
    source = Path(__file__).resolve().parents[1] / "config" / "robot.example.yaml"
    target = tmp_path / "robot.yaml"
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    data["kinematics"]["dh_rows"] = [
        {"joint_index": 0, "theta_offset_deg": 0.0, "d_mm": 1.0, "a_mm": 0.0, "alpha_deg": 90.0},
        {"joint_index": 1, "theta_offset_deg": 0.0, "d_mm": 2.0, "a_mm": 10.0, "alpha_deg": 0.0},
        {"joint_index": 2, "theta_offset_deg": 0.0, "d_mm": 3.0, "a_mm": 20.0, "alpha_deg": 0.0},
        {"joint_index": 3, "theta_offset_deg": 0.0, "d_mm": 4.0, "a_mm": 30.0, "alpha_deg": 0.0},
    ]
    target.write_text(yaml.safe_dump(data), encoding="utf-8")

    loaded = load_config(target)

    assert [row.joint_index for row in loaded.kinematics.dh_rows] == [0, 1, 2, 3]
