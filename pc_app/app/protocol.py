from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .config import JointConfig


def format_hello() -> str:
    return "HELLO"


def format_status() -> str:
    return "STATUS"


def format_movej(joints_deg: list[float], speed: float, accel: float) -> str:
    if len(joints_deg) != 4:
        raise ValueError("MOVEJ requires exactly four joint angles")
    values = " ".join(f"{angle:.3f}" for angle in joints_deg)
    return f"MOVEJ {values} {speed:.3f} {accel:.3f}"


def format_jogj(joints_deg: list[float], speed: float, accel: float) -> str:
    if len(joints_deg) != 4:
        raise ValueError("JOGJ requires exactly four joint angles")
    if speed <= 0 or accel <= 0:
        raise ValueError("JOGJ requires positive speed and acceleration")
    values = " ".join(f"{angle:.3f}" for angle in joints_deg)
    return f"JOGJ {values} {speed:.3f} {accel:.3f}"


def format_jogv(joint_velocity_deg_s: list[float], accel: float) -> str:
    if len(joint_velocity_deg_s) != 4:
        raise ValueError("JOGV requires exactly four joint velocities")
    if accel <= 0:
        raise ValueError("JOGV requires positive acceleration")
    values = " ".join(f"{velocity:.3f}" for velocity in joint_velocity_deg_s)
    return f"JOGV {values} {accel:.3f}"


def format_jog_stop() -> str:
    return "JOG STOP"


def format_traj_begin(point_count: int, duration_s: float, speed: float, accel: float) -> str:
    if point_count < 2:
        raise ValueError("TRAJ BEGIN requires at least two points")
    if duration_s <= 0:
        raise ValueError("TRAJ BEGIN requires a positive duration")
    if speed <= 0 or accel <= 0:
        raise ValueError("TRAJ BEGIN requires positive speed and acceleration")
    return f"TRAJ BEGIN count={point_count} duration={duration_s:.3f} speed={speed:.3f} accel={accel:.3f}"


def format_traj_point(index: int, time_s: float, joints_deg: list[float]) -> str:
    if index < 0:
        raise ValueError("TRAJ POINT index must be non-negative")
    if time_s < 0:
        raise ValueError("TRAJ POINT time must be non-negative")
    if len(joints_deg) != 4:
        raise ValueError("TRAJ POINT requires exactly four joint angles")
    values = " ".join(f"j{joint_index}={angle:.3f}" for joint_index, angle in enumerate(joints_deg, start=1))
    return f"TRAJ POINT index={index} t={time_s:.3f} {values}"


def format_traj_start() -> str:
    return "TRAJ START"


def format_traj_clear() -> str:
    return "TRAJ CLEAR"


def format_stop() -> str:
    return "STOP"


def format_estop() -> str:
    return "ESTOP"


def format_home() -> str:
    return "HOME"


def format_arm(armed: bool) -> str:
    return f"ARM {1 if armed else 0}"


def format_setpose(joints_deg: list[float]) -> str:
    if len(joints_deg) != 4:
        raise ValueError("SETPOSE requires exactly four joint angles")
    values = " ".join(f"{angle:.3f}" for angle in joints_deg)
    return f"SETPOSE {values}"


def format_tool(action: str, value: float | None = None) -> str:
    normalized = action.strip().upper()
    if normalized in {"OPEN", "CLOSE", "ON", "OFF"}:
        return f"TOOL {normalized}"
    if normalized == "SET":
        if value is None:
            raise ValueError("TOOL SET requires a value")
        clamped = max(0.0, min(1.0, float(value)))
        return f"TOOL SET value={clamped:.3f}"
    raise ValueError(f"unsupported TOOL action {action}")


def _tool_config_lines(tools: dict[str, Any] | None) -> list[str]:
    if not isinstance(tools, dict):
        return []
    active = str(tools.get("active", "gripper")).replace(" ", "_")
    presets = tools.get("presets")
    if not isinstance(presets, dict):
        return []

    lines: list[str] = []
    for name, preset in presets.items():
        if not isinstance(preset, dict):
            continue
        tool_type = str(preset.get("type", "generic")).replace(" ", "_")
        safe_name = str(name).replace(" ", "_")
        tcp = preset.get("tcp_offset_mm") if isinstance(preset.get("tcp_offset_mm"), dict) else {}
        common = (
            "CONFIG TOOL "
            f"name={safe_name} active={1 if safe_name == active else 0} type={tool_type} "
            f"tcp_x={float(tcp.get('x', 0.0)):.3f} "
            f"tcp_y={float(tcp.get('y', 0.0)):.3f} "
            f"tcp_z={float(tcp.get('z', 0.0)):.3f}"
        )
        io = preset.get("io") if isinstance(preset.get("io"), dict) else {}
        if tool_type == "servo_gripper":
            lines.append(
                common
                + f" open={float(preset.get('open_value', 0.0)):.3f}"
                + f" close={float(preset.get('closed_value', 1.0)):.3f}"
                + f" pwm={int(io.get('pwm_pin', -1))}"
                + f" min_us={int(io.get('pulse_min_us', 500))}"
                + f" max_us={int(io.get('pulse_max_us', 2500))}"
                + f" freq={int(io.get('pwm_frequency_hz', 50))}"
            )
        elif tool_type == "electromagnet":
            lines.append(
                common
                + f" pin={int(io.get('pin', -1))}"
                + f" active_high={1 if bool(io.get('active_high', True)) else 0}"
            )
        else:
            lines.append(common)
    return lines


def format_config_lines(joints: list[JointConfig], tools: dict[str, Any] | None = None) -> list[str]:
    if len(joints) != 4:
        raise ValueError("hardware CONFIG requires exactly four joints")
    lines = ["CONFIG BEGIN axes=4"]
    for index, joint in enumerate(joints, start=1):
        common = (
            f"index={index} name={joint.name} actuator={joint.actuator} "
            f"zero={joint.zero_offset_deg:.3f} sign={joint.direction_sign} "
            f"min={joint.min_deg:.3f} max={joint.max_deg:.3f} home={joint.home_deg:.3f} "
            f"max_speed={joint.max_speed_deg_s:.3f} max_accel={joint.max_accel_deg_s2:.3f}"
        )
        if joint.actuator == "servo" and joint.hardware.servo:
            servo = joint.hardware.servo
            lines.append(
                "CONFIG JOINT "
                f"{common} enabled={1 if servo.enabled else 0} "
                f"pwm={servo.pwm_pin} min_us={servo.pulse_min_us} max_us={servo.pulse_max_us} "
                f"freq={servo.pwm_frequency_hz} servo_range={servo.servo_range_deg:.3f} "
                f"neutral={servo.neutral_deg:.3f} gear={servo.gear_ratio:.6f}"
            )
        else:
            stepper = joint.hardware.stepper
            if stepper is None:
                raise ValueError(f"{joint.name} has no stepper hardware config")
            driver_model = stepper.driver_model.replace(" ", "_")
            lines.append(
                "CONFIG JOINT "
                f"{common} enabled={1 if stepper.enabled else 0} "
                f"step={stepper.step_pin} dir={stepper.dir_pin} enable={stepper.enable_pin} "
                f"enable_low={1 if stepper.enable_active_low else 0} "
                f"driver={driver_model} full_steps={stepper.motor_full_steps_per_rev} "
                f"microsteps={stepper.microsteps} gear={stepper.gear_ratio:.6f}"
            )
    lines.extend(_tool_config_lines(tools))
    lines.append("CONFIG END")
    return lines


@dataclass(frozen=True)
class ControllerStatus:
    state: str
    homed: bool
    joints_deg: list[float]
    fault: str
    armed: bool = False
    hardware_mode: str = "unknown"
    enabled_axes: str = "0000"
    known_pose: bool = False
    pose_source: str = "unknown"
    encoder_available: str = "0000"
    encoder_angles_deg: list[float | None] | None = None
    closed_loop_mode: str = "off"
    tool_type: str = "unknown"
    tool_state: str = "unknown"
    tool_value: float | None = None


def parse_status(line: str) -> ControllerStatus:
    parts = line.strip().split()
    if not parts or parts[0] != "STATUS":
        raise ValueError("status line must start with STATUS")

    values: dict[str, str] = {}
    for token in parts[1:]:
        if "=" in token:
            key, value = token.split("=", 1)
            values[key] = value

    joints = [float(values.get(f"j{index}", "0.0")) for index in range(1, 5)]
    encoder_angles: list[float | None] = []
    for index in range(1, 5):
        key = f"e{index}"
        encoder_angles.append(float(values[key]) if key in values else None)
    homed = values.get("homed", "0") in {"1", "true", "True"}
    return ControllerStatus(
        state=values.get("state", "unknown"),
        homed=homed,
        joints_deg=joints,
        fault=values.get("fault", "UNKNOWN"),
        armed=values.get("armed", "0") in {"1", "true", "True"},
        hardware_mode=values.get("hw", values.get("hardware", "unknown")),
        enabled_axes=values.get("enabled", "0000"),
        known_pose=values.get("known", "1" if homed else "0") in {"1", "true", "True"},
        pose_source=values.get("pose_source", "home" if homed else "unknown"),
        encoder_available=values.get("enc", "0000"),
        encoder_angles_deg=encoder_angles,
        closed_loop_mode=values.get("closed_loop", "off"),
        tool_type=values.get("tool_type", "unknown"),
        tool_state=values.get("tool", "unknown"),
        tool_value=float(values["tool_value"]) if "tool_value" in values else None,
    )
