from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha1
from math import isfinite
from typing import Any

from .config import JointConfig


def format_hello() -> str:
    return "HELLO"


def format_status() -> str:
    return "STATUS"


def parse_key_value_line(line: str) -> dict[str, str]:
    parts = line.strip().split()
    values: dict[str, str] = {}
    for token in parts[1:]:
        if "=" in token:
            key, value = token.split("=", 1)
            values[key] = value
    return values


def parse_hello_capabilities(line: str) -> dict[str, Any]:
    parts = line.strip().split()
    if not parts or parts[0] != "HELLO":
        return {}
    values = parse_key_value_line(line)

    def int_value(key: str, fallback: int = 0) -> int:
        try:
            return int(values.get(key, fallback))
        except (TypeError, ValueError):
            return fallback

    protocol = int_value("protocol", 1)
    encoder = values.get("encoder", "0") in {"1", "true", "True"}
    return {
        "name": values.get("name", ""),
        "firmware": values.get("firmware", ""),
        "protocol": protocol,
        "config": values.get("config", "0") in {"1", "true", "True"},
        "encoder": encoder,
        "encoder_config": protocol >= 4 and encoder,
        "alignj": values.get("alignj", "0") in {"1", "true", "True"},
        "raw": line,
    }


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


def format_servoj(joints_deg: list[float], duration_s: float) -> str:
    if len(joints_deg) != 4:
        raise ValueError("SERVOJ requires exactly four joint angles")
    if duration_s <= 0:
        raise ValueError("SERVOJ requires a positive duration")
    values = " ".join(f"{angle:.3f}" for angle in joints_deg)
    return f"SERVOJ {values} {duration_s:.4f}"


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


def format_correctj(joint: int, delta_deg: float, speed: float, accel: float, transaction_id: str) -> str:
    if joint != 2:
        raise ValueError("CORRECTJ currently supports only joint 2")
    if speed <= 0 or accel <= 0:
        raise ValueError("CORRECTJ requires positive speed and acceleration")
    if not transaction_id.strip():
        raise ValueError("CORRECTJ requires a transaction ID")
    return (
        f"CORRECTJ joint={joint} delta={float(delta_deg):.6f} "
        f"speed={float(speed):.6f} accel={float(accel):.6f} "
        f"id={_safe_token(transaction_id)}"
    )


def format_alignj(
    joint: int,
    delta_deg: float,
    speed: float,
    accel: float,
    transaction_id: str,
    *,
    hold: bool = True,
) -> str:
    if joint != 2:
        raise ValueError("ALIGNJ currently supports only joint 2")
    if speed <= 0 or accel <= 0:
        raise ValueError("ALIGNJ requires positive speed and acceleration")
    if not transaction_id.strip():
        raise ValueError("ALIGNJ requires a transaction ID")
    return (
        f"ALIGNJ joint={joint} delta={float(delta_deg):.6f} "
        f"speed={float(speed):.6f} accel={float(accel):.6f} "
        f"id={_safe_token(transaction_id)} hold={1 if hold else 0}"
    )


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


def _safe_token(value: Any, fallback: str = "none") -> str:
    text = str(value if value not in {None, ""} else fallback).strip()
    return text.replace(" ", "_")


def _short_firmware_id_token(value: Any, fallback: str = "none") -> str:
    text = _safe_token(value)
    if text == "none":
        return fallback
    # The controller only needs a compact identity/validation marker. Keep full
    # UUIDs in the PC config, not in bounded serial command lines.
    return f"v{sha1(text.encode('ascii', errors='ignore')).hexdigest()[:10]}"


def _short_validation_token(value: Any, *, enabled: bool) -> str:
    if not enabled:
        return "none"
    return _short_firmware_id_token(value)


def _safe_int(value: Any, fallback: int) -> int:
    try:
        if isinstance(value, bool):
            raise ValueError
        return int(value)
    except (TypeError, ValueError):
        return int(fallback)


def _safe_float(value: Any, fallback: float) -> float:
    try:
        if isinstance(value, bool):
            raise ValueError
        converted = float(value)
    except (TypeError, ValueError):
        return float(fallback)
    return converted if isfinite(converted) else float(fallback)


def _encoder_config_lines(encoders: dict[str, Any] | None) -> list[str]:
    if not isinstance(encoders, dict):
        return []
    bus = encoders.get("bus") if isinstance(encoders.get("bus"), dict) else {}
    lines = [
        "CONFIG ENCODER_BUS "
        f"enabled={1 if bool(encoders.get('enabled')) else 0} "
        f"type={_safe_token(bus.get('type'), 'spi')} "
        f"sck={_safe_int(bus.get('sck_pin', -1), -1)} "
        f"miso={_safe_int(bus.get('miso_pin', -1), -1)} "
        f"mosi={_safe_int(bus.get('mosi_pin', -1), -1)} "
        f"clock={_safe_int(bus.get('clock_hz', 1_000_000), 1_000_000)} "
        f"sample_ms={_safe_int(bus.get('sample_interval_ms', 100), 100)}"
    ]
    for axis in encoders.get("axes", []):
        if not isinstance(axis, dict):
            continue
        lines.append(
            "CONFIG ENCODER "
            f"joint={_safe_int(axis.get('joint', 0), 0)} "
            f"name={_safe_token(axis.get('name'), 'encoder')} "
            f"sensor={_safe_token(axis.get('sensor'), 'as5048a')} "
            f"enabled={1 if bool(axis.get('enabled')) else 0} "
            f"cs={_safe_int(axis.get('cs_pin', -1), -1)} "
            f"reference_raw={_safe_float(axis.get('reference_raw_deg', 0.0), 0.0):.6f} "
            f"reference_joint={_safe_float(axis.get('reference_joint_deg', 0.0), 0.0):.6f} "
            f"sign={-1 if _safe_int(axis.get('direction_sign', 1), 1) < 0 else 1} "
            f"wrap={_safe_float(axis.get('wrap_period_deg', 360.0), 360.0):.6f} "
            f"turns={_safe_float(axis.get('sensor_turns_per_joint_turn', 1.0), 1.0):.6f} "
            f"mounting={_safe_token(axis.get('mounting_location'), 'joint_output')} "
            f"freshness_ms={_safe_int(axis.get('freshness_timeout_ms', 500), 500)} "
            f"max_noise={_safe_float(axis.get('max_noise_deg', 0.5), 0.5):.6f} "
            f"calibrated={1 if bool(axis.get('calibration_validated')) else 0} "
            f"calibration_id={_short_firmware_id_token(axis.get('calibration_id'))}"
        )
    verification = encoders.get("verification") if isinstance(encoders.get("verification"), dict) else {}
    correction = encoders.get("correction") if isinstance(encoders.get("correction"), dict) else {}
    correction_enabled = bool(correction.get("enabled"))
    auto_max_delta = _safe_float(correction.get("max_delta_deg", 1.0), 1.0)
    align_max_delta = _safe_float(correction.get("align_max_delta_deg", auto_max_delta), auto_max_delta)
    firmware_max_delta = max(auto_max_delta, align_max_delta)
    lines.append(
        "CONFIG ENCODER_POLICY "
        f"mode={_safe_token(encoders.get('mode'), 'diagnostic')} "
        f"policy={_safe_token(verification.get('policy'), 'diagnostic')} "
        f"settle_ms={_safe_int(verification.get('settle_delay_ms', 300), 300)} "
        f"samples={_safe_int(verification.get('required_stable_samples', 3), 3)} "
        f"warn={_safe_float(verification.get('warning_tolerance_deg', 2.0), 2.0):.6f} "
        f"fault={_safe_float(verification.get('fault_tolerance_deg', 5.0), 5.0):.6f} "
        f"hysteresis={_safe_float(verification.get('hysteresis_deg', 0.25), 0.25):.6f} "
        f"require={1 if bool(verification.get('require_encoder')) else 0} "
        f"correction={1 if correction_enabled else 0} "
        f"validation_id={_short_validation_token(correction.get('validation_id'), enabled=correction_enabled)} "
        f"max_delta={firmware_max_delta:.6f} "
        f"limit_margin={_safe_float(correction.get('joint_limit_margin_deg', 2.0), 2.0):.6f} "
        f"correction_speed={_safe_float(correction.get('speed_deg_s', 2.0), 2.0):.6f} "
        f"correction_accel={_safe_float(correction.get('accel_deg_s2', 10.0), 10.0):.6f} "
        f"attempts={_safe_int(correction.get('max_attempts', 2), 2)}"
    )
    return lines


def format_config_lines(
    joints: list[JointConfig],
    tools: dict[str, Any] | None = None,
    encoders: dict[str, Any] | None = None,
) -> list[str]:
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
    lines.extend(_encoder_config_lines(encoders))
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
    known_mask: str = "0000"
    pose_source: str = "unknown"
    encoder_available: str = "0000"
    encoder_angles_deg: list[float | None] | None = None
    encoder_valid: str = "0000"
    encoder_raw_counts: list[int | None] | None = None
    encoder_raw_angles_deg: list[float | None] | None = None
    encoder_measured_angles_deg: list[float | None] | None = None
    encoder_age_ms: list[int | None] | None = None
    encoder_noise_deg: list[float | None] | None = None
    encoder_consecutive_valid_samples: list[int | None] | None = None
    encoder_flags: list[list[str]] | None = None
    closed_loop_mode: str = "off"
    correction_bias_deg: list[float | None] | None = None
    correction_state: str = "idle"
    correction_transaction_id: str = "none"
    correction_requested_delta_deg: float | None = None
    correction_emitted_steps: int | None = None
    correction_attempts: int = 0
    tool_type: str = "unknown"
    tool_state: str = "unknown"
    tool_value: float | None = None


def parse_status(line: str) -> ControllerStatus:
    parts = line.strip().split()
    if not parts or parts[0] != "STATUS":
        raise ValueError("status line must start with STATUS")

    values = parse_key_value_line(line)

    joints = [float(values.get(f"j{index}", "0.0")) for index in range(1, 5)]
    encoder_angles: list[float | None] = []
    raw_counts: list[int | None] = []
    raw_angles: list[float | None] = []
    measured_angles: list[float | None] = []
    ages: list[int | None] = []
    noise: list[float | None] = []
    consecutive_valid: list[int | None] = []
    flags: list[list[str]] = []
    correction_bias: list[float | None] = []
    for index in range(1, 5):
        legacy_key = f"e{index}"
        raw_key = f"er{index}"
        raw_angle_key = f"ea{index}"
        measured_key = f"em{index}"
        age_key = f"eage{index}"
        noise_key = f"enoise{index}"
        valid_count_key = f"evalidn{index}"
        flags_key = f"ef{index}"
        bias_key = f"cb{index}"
        legacy_angle = float(values[legacy_key]) if legacy_key in values else None
        encoder_angles.append(legacy_angle)
        raw_counts.append(int(values[raw_key]) if raw_key in values else None)
        raw_angles.append(
            float(values[raw_angle_key])
            if raw_angle_key in values
            else legacy_angle
        )
        measured_angles.append(float(values[measured_key]) if measured_key in values else None)
        ages.append(int(values[age_key]) if age_key in values else None)
        noise.append(float(values[noise_key]) if noise_key in values else None)
        consecutive_valid.append(
            int(values[valid_count_key])
            if valid_count_key in values
            else None
        )
        flags.append(
            []
            if flags_key not in values or values[flags_key] in {"", "OK", "none"}
            else [item for item in values[flags_key].split(",") if item]
        )
        correction_bias.append(float(values[bias_key]) if bias_key in values else None)
    homed = values.get("homed", "0") in {"1", "true", "True"}
    known_pose = values.get("known", "1" if homed else "0") in {"1", "true", "True"}
    return ControllerStatus(
        state=values.get("state", "unknown"),
        homed=homed,
        joints_deg=joints,
        fault=values.get("fault", "UNKNOWN"),
        armed=values.get("armed", "0") in {"1", "true", "True"},
        hardware_mode=values.get("hw", values.get("hardware", "unknown")),
        enabled_axes=values.get("enabled", "0000"),
        known_pose=known_pose,
        known_mask=values.get("known_mask", ("1111" if known_pose else "0000")),
        pose_source=values.get("pose_source", "home" if homed else "unknown"),
        encoder_available=values.get("enc", "0000"),
        encoder_angles_deg=encoder_angles,
        encoder_valid=values.get("enc_valid", values.get("enc", "0000")),
        encoder_raw_counts=raw_counts,
        encoder_raw_angles_deg=raw_angles,
        encoder_measured_angles_deg=measured_angles,
        encoder_age_ms=ages,
        encoder_noise_deg=noise,
        encoder_consecutive_valid_samples=consecutive_valid,
        encoder_flags=flags,
        closed_loop_mode=values.get("closed_loop", "off"),
        correction_bias_deg=correction_bias,
        correction_state=values.get("correction", "idle"),
        correction_transaction_id=values.get("correction_id", "none"),
        correction_requested_delta_deg=(
            float(values["correction_delta"])
            if "correction_delta" in values
            else None
        ),
        correction_emitted_steps=(
            int(values["correction_steps"])
            if "correction_steps" in values
            else None
        ),
        correction_attempts=int(values.get("correction_attempts", "0")),
        tool_type=values.get("tool_type", "unknown"),
        tool_state=values.get("tool", "unknown"),
        tool_value=float(values["tool_value"]) if "tool_value" in values else None,
    )
