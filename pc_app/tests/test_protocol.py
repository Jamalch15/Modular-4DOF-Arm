from app.config import load_config
from app.demo_settings import tools_settings
from app.protocol import (
    format_arm,
    format_config_lines,
    format_jog_stop,
    format_jogj,
    format_jogv,
    format_movej,
    format_setpose,
    format_tool,
    format_traj_begin,
    format_traj_clear,
    format_traj_point,
    format_traj_start,
    parse_status,
)


def test_format_movej_line():
    command = format_movej([1.0, 2.0, 3.0, 4.0], speed=25.0, accel=100.0)

    assert command == "MOVEJ 1.000 2.000 3.000 4.000 25.000 100.000"


def test_format_jog_lines():
    command = format_jogj([1.0, 2.0, 3.0, 4.0], speed=25.0, accel=100.0)

    assert command == "JOGJ 1.000 2.000 3.000 4.000 25.000 100.000"
    assert format_jogv([1.0, -2.0, 3.5, 0.0], accel=100.0) == "JOGV 1.000 -2.000 3.500 0.000 100.000"
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


def test_format_hardware_config_lines_from_config():
    config = load_config()
    lines = format_config_lines(config.joints, tools_settings(config))
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


def test_format_arm_and_setpose():
    assert format_arm(True) == "ARM 1"
    assert format_arm(False) == "ARM 0"
    assert format_setpose([0, 1, 2, 3]) == "SETPOSE 0.000 1.000 2.000 3.000"


def test_format_tool_commands():
    assert format_tool("open") == "TOOL OPEN"
    assert format_tool("close") == "TOOL CLOSE"
    assert format_tool("set", 0.3456) == "TOOL SET value=0.346"
    assert format_tool("on") == "TOOL ON"
    assert format_tool("off") == "TOOL OFF"
