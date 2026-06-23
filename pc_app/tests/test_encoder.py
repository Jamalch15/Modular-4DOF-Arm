from copy import deepcopy

import pytest

from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.encoder import (
    calibrated_joint_angle,
    default_encoder_settings,
    evidence_from_status,
    normalize_encoder_settings,
    validate_encoder_settings,
    wrapped_delta_deg,
)


def test_wrapped_delta_crosses_zero_without_a_full_turn_jump():
    assert wrapped_delta_deg(2.0, 358.0) == pytest.approx(4.0)
    assert wrapped_delta_deg(358.0, 2.0) == pytest.approx(-4.0)


def test_calibrated_joint_angle_applies_reference_sign_and_scale():
    axis = default_encoder_settings()["axes"][0]
    axis.update(
        {
            "reference_raw_deg": 350.0,
            "reference_joint_deg": 20.0,
            "direction_sign": -1,
            "sensor_turns_per_joint_turn": 2.0,
        }
    )

    assert calibrated_joint_angle(10.0, axis) == pytest.approx(10.0)


def test_calibrated_joint_angle_uses_piecewise_map():
    axis = default_encoder_settings()["axes"][0]
    axis.update(
        {
            "reference_raw_deg": 100.0,
            "reference_joint_deg": 90.0,
            "calibration_model": "piecewise_linear",
            "calibration_map": [
                {"raw_delta_deg": -20.0, "joint_deg": 110.0},
                {"raw_delta_deg": 0.0, "joint_deg": 90.0},
                {"raw_delta_deg": 20.0, "joint_deg": 65.0},
            ],
        }
    )

    assert calibrated_joint_angle(110.0, axis) == pytest.approx(77.5)


def test_piecewise_map_refuses_uncalibrated_range():
    axis = default_encoder_settings()["axes"][0]
    axis.update(
        {
            "reference_raw_deg": 100.0,
            "calibration_model": "piecewise_linear",
            "calibration_map": [
                {"raw_delta_deg": -10.0, "joint_deg": 100.0},
                {"raw_delta_deg": 10.0, "joint_deg": 80.0},
            ],
            "calibration_map_extrapolate_deg": 1.0,
        }
    )

    with pytest.raises(ValueError, match="outside the calibrated map range"):
        calibrated_joint_angle(120.0, axis)


def test_default_encoder_settings_use_recommended_esp32_s3_pins():
    settings = default_encoder_settings()

    assert settings["bus"]["sck_pin"] == 12
    assert settings["bus"]["miso_pin"] == 13
    assert settings["bus"]["mosi_pin"] == 14
    assert settings["axes"][0]["cs_pin"] == 15


def test_legacy_settle_correction_is_migrated_to_safe_diagnostic_mode():
    settings = normalize_encoder_settings(
        {
            "enabled": True,
            "closed_loop_mode": "settle_correction",
            "axes": [{"joint": 2, "enabled": True, "cs_pin": 12}],
        }
    )

    assert settings["schema_version"] == 2
    assert settings["mode"] == "diagnostic"
    assert settings["correction"]["enabled"] is False


def test_disabled_encoder_bus_removes_bounded_correction_authority():
    settings = normalize_encoder_settings(
        {
            "schema_version": 2,
            "enabled": False,
            "mode": "bounded_correction",
            "axes": [
                {
                    "joint": 2,
                    "enabled": True,
                    "cs_pin": 12,
                    "calibration_validated": True,
                    "calibration_id": "fixture-a",
                }
            ],
            "correction": {
                "enabled": True,
                "validation_id": "validated-bench-run",
            },
        }
    )

    assert settings["mode"] == "diagnostic"
    assert settings["correction"]["enabled"] is False


def test_disabled_correction_does_not_block_save_with_stale_limits():
    config = load_config(EXAMPLE_CONFIG_PATH)
    settings = default_encoder_settings()
    settings["correction"].update(
        {
            "enabled": False,
            "max_delta_deg": "not-a-number",
            "joint_limit_margin_deg": "not-a-number",
            "speed_deg_s": "not-a-number",
            "accel_deg_s2": "not-a-number",
            "max_attempts": "not-a-number",
        }
    )

    errors = validate_encoder_settings(config, settings)

    assert not any("encoder correction" in error for error in errors)


def test_encoder_validation_rejects_non_shoulder_enabled_legacy_axis():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw.get("encoders", {}))
    raw["enabled"] = True
    raw["axes"] = [
        {"joint": 1, "enabled": True, "cs_pin": 12},
        {"joint": 2, "enabled": False, "cs_pin": -1},
    ]
    patched = deepcopy(config.raw)
    patched["encoders"] = raw
    config = type(config)(**{**config.__dict__, "raw": patched})

    errors = validate_encoder_settings(config, normalize_encoder_settings(config))

    assert any("only the shoulder encoder" in error for error in errors)


def test_encoder_validation_rejects_esp32_s3_gpio_holes_when_enabled():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw.get("encoders", {}))
    raw["enabled"] = True
    raw["bus"]["mosi_pin"] = 23
    raw["axes"][0].update({"enabled": True, "cs_pin": 12})
    patched = deepcopy(config.raw)
    patched["encoders"] = raw
    config = type(config)(**{**config.__dict__, "raw": patched})

    errors = validate_encoder_settings(config, normalize_encoder_settings(config))

    assert any("encoders.bus.mosi_pin" in error and "valid ESP32-S3 GPIO" in error for error in errors)


def test_encoder_validation_rejects_unsorted_piecewise_map():
    config = load_config(EXAMPLE_CONFIG_PATH)
    settings = default_encoder_settings()
    settings["axes"][0].update(
        {
            "calibration_model": "piecewise_linear",
            "calibration_map": [
                {"raw_delta_deg": 5.0, "joint_deg": 10.0},
                {"raw_delta_deg": 5.0, "joint_deg": 12.0},
            ],
        }
    )

    errors = validate_encoder_settings(config, settings)

    assert any("calibration_map raw_delta_deg values must be strictly increasing" in error for error in errors)


def test_encoder_evidence_becomes_stale_without_changing_the_measurement():
    evidence = evidence_from_status(
        joint_number=2,
        name="shoulder",
        raw_count=1024,
        raw_angle_deg=22.5,
        measured_angle_deg=10.0,
        valid=True,
        age_ms=600,
        noise_deg=0.1,
        flags=[],
        freshness_timeout_ms=500,
        estimated_angle_deg=9.5,
    )

    assert evidence["measured_angle_deg"] == 10.0
    assert evidence["fresh"] is False
    assert evidence["valid"] is True
    assert evidence["health"] == "stale"
    assert evidence["mismatch_deg"] is None
