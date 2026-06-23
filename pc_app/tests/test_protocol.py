import pytest

from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.demo_settings import encoder_settings, tools_settings
from app.protocol import (
    format_arm,
    format_config_lines,
    format_correctj,
    format_jog_stop,
    format_jogj,
    format_jogv,
    format_movej,
    format_servoj,
    format_setpose,
    format_tool,
    format_traj_begin,
    format_traj_clear,
    format_traj_point,
    format_traj_start,
    parse_hello_capabilities,
    parse_status,
)


def test_format_movej_line():
    command = format_movej([1.0, 2.0, 3.0, 4.0], speed=25.0, accel=100.0)

    assert command == "MOVEJ 1.000 2.000 3.000 4.000 25.000 100.000"


def test_format_jog_lines():
    command = format_jogj([1.0, 2.0, 3.0, 4.0], speed=25.0, accel=100.0)

    assert command == "JOGJ 1.000 2.000 3.000 4.000 25.000 100.000"
    assert format_jogv([1.0, -2.0, 3.5, 0.0], accel=100.0) == "JOGV 1.000 -2.000 3.500 0.000 100.000"
    assert format_servoj([1.0, -2.0, 3.5, 0.0], duration_s=1.0 / 30.0) == (
        "SERVOJ 1.000 -2.000 3.500 0.000 0.0333"
    )
    assert format_jog_stop() == "JOG STOP"


def test_format_trajectory_lines():
    assert format_traj_begin(3, duration_s=1.25, speed=25.0, accel=100.0) == (
        "TRAJ BEGIN count=3 duration=1.250 speed=25.000 accel=100.000"
    )
    assert format_traj_point(1, 0.5, [1.0, 2.0, 3.0, 4.0]) == (
        "TRAJ POINT index=1 t=0.500 j1=1.000 j2=2.000 j3=3.000 j4=4.000"
    )
    assert format_traj_start() == "TRAJ START"
    assert format_traj_clear() == "TRAJ CLEAR"


def test_parse_status_line():
    status = parse_status("STATUS state=idle homed=1 j1=1.5 j2=2 j3=-3 j4=4 fault=OK")

    assert status.state == "idle"
    assert status.homed is True
    assert status.joints_deg == [1.5, 2.0, -3.0, 4.0]
    assert status.fault == "OK"


def test_parse_hello_capabilities_detects_encoder_config_support():
    legacy = parse_hello_capabilities("HELLO name=esp32s3-arm firmware=arm_controller protocol=3 config=1")
    current = parse_hello_capabilities("HELLO name=esp32s3-arm firmware=arm_controller protocol=4 config=1 encoder=1")

    assert legacy["protocol"] == 3
    assert legacy["config"] is True
    assert legacy["encoder_config"] is False
    assert current["protocol"] == 4
    assert current["encoder"] is True
    assert current["encoder_config"] is True


def test_parse_extended_status_line():
    status = parse_status(
        "STATUS state=idle homed=1 armed=1 hw=mixed enabled=1100 known=1 "
        "pose_source=mixed enc=1100 e1=12.5 e2=-4.25 j1=1 j2=2 j3=3 j4=4 "
        "closed_loop=readback tool_type=servo_gripper tool=open tool_value=0.250 fault=OK"
    )

    assert status.armed is True
    assert status.hardware_mode == "mixed"
    assert status.enabled_axes == "1100"
    assert status.known_pose is True
    assert status.pose_source == "mixed"
    assert status.encoder_available == "1100"
    assert status.encoder_angles_deg[:2] == [12.5, -4.25]
    assert status.closed_loop_mode == "readback"
    assert status.tool_type == "servo_gripper"
    assert status.tool_state == "open"
    assert status.tool_value == 0.25


def test_parse_protocol_v4_encoder_evidence_without_changing_joint_estimates():
    status = parse_status(
        "STATUS state=idle homed=0 known=0 known_mask=0100 pose_source=open_loop_estimate "
        "armed=1 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        "er2=8192 ea2=180.0 em2=22.5 eage2=40 enoise2=0.08 evalidn2=4 ef2=OK "
        "j1=1 j2=20 j3=3 j4=4 closed_loop=diagnostic correction=completed "
        "correction_id=tx-1 correction_delta=-0.25 correction_steps=-18 "
        "correction_attempts=1 cb1=0 cb2=0.25 cb3=0 cb4=0 fault=OK"
    )

    assert status.joints_deg == [1.0, 20.0, 3.0, 4.0]
    assert status.known_pose is False
    assert status.known_mask == "0100"
    assert status.encoder_valid == "0100"
    assert status.encoder_raw_counts[1] == 8192
    assert status.encoder_raw_angles_deg[1] == pytest.approx(180.0)
    assert status.encoder_measured_angles_deg[1] == pytest.approx(22.5)
    assert status.encoder_age_ms[1] == 40
    assert status.encoder_noise_deg[1] == pytest.approx(0.08)
    assert status.encoder_consecutive_valid_samples[1] == 4
    assert status.correction_bias_deg[1] == pytest.approx(0.25)
    assert status.correction_state == "completed"
    assert status.correction_transaction_id == "tx-1"
    assert status.correction_requested_delta_deg == pytest.approx(-0.25)
    assert status.correction_emitted_steps == -18
    assert status.correction_attempts == 1


def test_format_hardware_config_lines_from_config():
    config = load_config(EXAMPLE_CONFIG_PATH)
    lines = format_config_lines(
        config.joints,
        tools_settings(config),
        encoder_settings(config),
    )
    base_stepper = config.joints[0].hardware.stepper
    elbow_servo = config.joints[2].hardware.servo

    assert lines[0] == "CONFIG BEGIN axes=4"
    assert lines[-1] == "CONFIG END"
    assert "CONFIG JOINT index=1 name=base actuator=stepper" in lines[1]
    expected_enabled = 1 if base_stepper.enabled else 0
    assert f"enabled={expected_enabled}" in lines[1]
    assert f"step={base_stepper.step_pin}" in lines[1]
    assert f"full_steps={base_stepper.motor_full_steps_per_rev}" in lines[1]
    assert f"microsteps={base_stepper.microsteps}" in lines[1]
    assert "m0=" not in lines[1]
    assert "m1=" not in lines[1]
    assert "m2=" not in lines[1]
    assert "CONFIG JOINT index=3 name=elbow actuator=servo" in lines[3]
    assert f"servo_range={elbow_servo.servo_range_deg:.3f}" in lines[3]
    assert f"min_us={elbow_servo.pulse_min_us}" in lines[3]
    assert f"max_us={elbow_servo.pulse_max_us}" in lines[3]
    assert any(line.startswith("CONFIG TOOL name=gripper active=") for line in lines)
    assert any("type=electromagnet" in line for line in lines)
    assert any(line.startswith("CONFIG ENCODER_BUS ") for line in lines)
    assert any(line.startswith("CONFIG ENCODER joint=2 ") for line in lines)
    assert any(line.startswith("CONFIG ENCODER_POLICY ") for line in lines)
    assert any("limit_margin=2.000000" in line for line in lines)


def test_disabled_encoder_config_formatting_tolerates_stale_local_values():
    config = load_config(EXAMPLE_CONFIG_PATH)
    encoders = encoder_settings(config)
    encoders["enabled"] = False
    encoders["bus"].update(
        {
            "sck_pin": "bad",
            "miso_pin": "bad",
            "mosi_pin": "bad",
            "clock_hz": "bad",
            "sample_interval_ms": "bad",
        }
    )
    encoders["axes"][0].update(
        {
            "enabled": False,
            "cs_pin": "bad",
            "reference_raw_deg": "bad",
            "reference_joint_deg": "bad",
            "direction_sign": "bad",
            "wrap_period_deg": "bad",
            "sensor_turns_per_joint_turn": "bad",
            "freshness_timeout_ms": "bad",
            "max_noise_deg": "bad",
        }
    )
    encoders["correction"].update(
        {
            "enabled": False,
            "max_delta_deg": "bad",
            "joint_limit_margin_deg": "bad",
            "speed_deg_s": "bad",
            "accel_deg_s2": "bad",
            "max_attempts": "bad",
        }
    )

    lines = format_config_lines(config.joints, tools_settings(config), encoders)

    assert any(
        line == "CONFIG ENCODER_BUS enabled=0 type=spi sck=-1 miso=-1 mosi=-1 clock=1000000 sample_ms=100"
        for line in lines
    )
    assert any("CONFIG ENCODER joint=2" in line and "enabled=0" in line and "cs=-1" in line for line in lines)
    assert any("CONFIG ENCODER_POLICY" in line and "correction=0" in line for line in lines)


def test_format_arm_and_setpose():
    assert format_arm(True) == "ARM 1"
    assert format_arm(False) == "ARM 0"
    assert format_setpose([0, 1, 2, 3]) == "SETPOSE 0.000 1.000 2.000 3.000"
    assert format_correctj(2, -0.25, 2.0, 10.0, "test transaction") == (
        "CORRECTJ joint=2 delta=-0.250000 speed=2.000000 accel=10.000000 id=test_transaction"
    )


def test_format_tool_commands():
    assert format_tool("open") == "TOOL OPEN"
    assert format_tool("close") == "TOOL CLOSE"
    assert format_tool("set", 0.3456) == "TOOL SET value=0.346"
    assert format_tool("on") == "TOOL ON"
    assert format_tool("off") == "TOOL OFF"
