from __future__ import annotations

import asyncio
import shutil
import tempfile
from dataclasses import asdict
from pathlib import Path
from time import monotonic, time
from typing import Any
from uuid import uuid4

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .apriltag_calibration import (
    AprilTagCalibrationSession,
    annotate_apriltag_frame,
    april_tag_settings,
    configured_tag_ids,
    detect_apriltags,
    estimate_camera_pose,
)
from .config import LinkConfig, RobotConfig, ensure_local_config, load_config, save_calibration_updates
from .demo_settings import (
    camera_settings,
    calibration_settings,
    color_profiles,
    drop_zones,
    encoder_settings,
    geometry_settings,
    model_validation_warnings,
    named_position_errors,
    named_positions,
    task_defaults,
    tool_settings,
    tools_settings,
    validate_named_position,
)
from .event_log import EventLog
from .kinematics import COORDINATE_FRAME, differential_ik_step, forward_kinematics, inverse_kinematics
from .motion import (
    RateLimitedMotion,
    build_joint_trajectory,
    build_linear_cartesian_trajectory,
    build_program_trajectory,
    has_reached_target,
)
from .protocol import (
    format_arm,
    format_config_lines,
    format_estop,
    format_hello,
    format_home,
    format_jog_stop,
    format_jogj,
    format_jogv,
    format_movej,
    format_setpose,
    format_status,
    format_stop,
    format_tool,
    format_traj_begin,
    format_traj_point,
    format_traj_start,
    parse_status,
)
from .robot_state import MotionState, RobotState
from .safety import validate_can_move, validate_joint_targets
from .serial_client import SerialClient, SerialClientError
from .simulator import apply_simulation_step
from .tasks import build_batch_sorting_sequence, build_pick_and_place_sequence, build_sorting_sequence
from .vision import annotated_detection_frame, decode_image_b64, detect_configured_colors, encode_image_b64


APP_DIR = Path(__file__).resolve().parent
STATIC_DIR = APP_DIR / "static"

config: RobotConfig = load_config()
state = RobotState(
    joint_names=config.joint_names,
    target_angles_deg=config.home_pose,
    reported_angles_deg=config.home_pose,
    connected=config.simulation_default,
    simulation=config.simulation_default,
    serial_port=None,
    known_pose=config.simulation_default,
)
state.active_tool = str(tools_settings(config).get("active", "gripper"))
state.tool_type = str(tool_settings(config).get("type", "servo_gripper"))
state.closed_loop_mode = str(encoder_settings(config).get("closed_loop_mode", "off"))
state.fk = forward_kinematics(state.reported_angles_deg, config.links)
limiter = RateLimitedMotion(config, state.reported_angles_deg.copy(), state.target_angles_deg.copy())
serial_client = SerialClient(config.serial)
websockets: set[WebSocket] = set()
path_previews: dict[str, dict[str, Any]] = {}
task_previews: dict[str, dict[str, Any]] = {}
path_task: asyncio.Task[None] | None = None
path_task_source: str | None = None
live_task: asyncio.Task[None] | None = None
task_task: asyncio.Task[None] | None = None
event_log = EventLog()
active_motion_run_id: str | None = None
simulation_trajectory_active = False
MAX_TRAJECTORY_UPLOAD_POINTS = 220
CARTESIAN_JOG_STALE_S = 0.35
CARTESIAN_JOG_MIN_DT_S = 0.01
CARTESIAN_JOG_MAX_DT_S = 0.08
cartesian_jog_runtime: dict[str, Any] = {
    "active": False,
    "last_update": 0.0,
    "last_status_poll": 0.0,
    "seed_deg": None,
    "joint_velocity_deg_s": [0.0, 0.0, 0.0, 0.0],
    "run_id": None,
}
april_tag_session = AprilTagCalibrationSession(camera_settings(config))

app = FastAPI(title="4DOF Robot Arm Control Dashboard")
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


class ConnectRequest(BaseModel):
    port: str | None = None
    baud_rate: int | None = None
    simulation: bool | None = None


class JointTargetRequest(BaseModel):
    index: int
    angle_deg: float
    settings: dict[str, Any] | None = None


class AllTargetsRequest(BaseModel):
    angles_deg: list[float]
    settings: dict[str, Any] | None = None


class ArmRequest(BaseModel):
    armed: bool


class IkTargetRequest(BaseModel):
    x_mm: float
    y_mm: float
    z_mm: float
    phi_deg: float | None = None
    phi_auto: bool = False


class PathSettingsRequest(BaseModel):
    global_speed_deg_s: float | None = None
    global_accel_deg_s2: float | None = None
    waypoint_rate_hz: float | None = None
    cartesian_step_mm: float | None = None
    planner_type: str | None = None
    jerk_percent: float | None = None
    blend_percent: float | None = None
    per_joint_speed_deg_s: list[float] | None = None
    per_joint_accel_deg_s2: list[float] | None = None


class IkSolveRequest(BaseModel):
    target: IkTargetRequest
    links_mm: dict[str, float] | None = None
    branch: str = "auto"


class PathPreviewRequest(BaseModel):
    target: IkTargetRequest | None = None
    mode: str = "joint"
    links_mm: dict[str, float] | None = None
    branch: str = "auto"
    settings: PathSettingsRequest | None = None
    waypoints: list[dict[str, Any]] | None = None


class PathExecuteRequest(BaseModel):
    preview_id: str


class LiveMotionRequest(BaseModel):
    enabled: bool


class LiveTargetRequest(BaseModel):
    angles_deg: list[float] | None = None
    target: IkTargetRequest | None = None
    mode: str = "joint"
    branch: str = "auto"
    settings: PathSettingsRequest | None = None


class CartesianJogRequest(BaseModel):
    vx_mm_s: float = 0.0
    vy_mm_s: float = 0.0
    vz_mm_s: float = 0.0
    vphi_deg_s: float = 0.0
    dt_s: float | None = None
    tcp_speed_mm_s: float | None = None
    phi_speed_deg_s: float | None = None
    settings: PathSettingsRequest | None = None


class CalibrationRequest(BaseModel):
    links_mm: dict[str, float] | None = None
    kinematics: dict[str, Any] | None = None
    joints: list[dict[str, Any]] | None = None
    motion: dict[str, Any] | None = None
    serial: dict[str, Any] | None = None
    named_positions: dict[str, dict[str, Any]] | None = None
    camera: dict[str, Any] | None = None
    color_profiles: dict[str, dict[str, Any]] | None = None
    drop_zones: dict[str, dict[str, Any]] | None = None
    task_defaults: dict[str, Any] | None = None
    path_defaults: dict[str, Any] | None = None
    tool: dict[str, Any] | None = None
    tools: dict[str, Any] | None = None
    encoders: dict[str, Any] | None = None
    calibration: dict[str, Any] | None = None
    geometry: dict[str, Any] | None = None


class SetPoseRequest(BaseModel):
    angles_deg: list[float]


class ToolRequest(BaseModel):
    action: str
    value: float | None = None
    tool: str | None = None


class ToolsRequest(BaseModel):
    active: str
    presets: dict[str, dict[str, Any]] | None = None


class NamedPositionsRequest(BaseModel):
    positions: dict[str, dict[str, Any]]


class VisionSettingsRequest(BaseModel):
    camera: dict[str, Any] | None = None
    color_profiles: dict[str, dict[str, Any]] | None = None
    drop_zones: dict[str, dict[str, Any]] | None = None


class VisionDetectRequest(BaseModel):
    image_b64: str | None = None
    profile_names: list[str] | None = None


class AprilTagCaptureRequest(BaseModel):
    image_b64: str | None = None
    sample_count: int = 1
    sample_interval_ms: int = 80
    accumulate: bool = True


class AprilTagSaveRequest(BaseModel):
    require_all_tags: bool = True


class TaskPreviewRequest(BaseModel):
    task: str = "pick_and_place"
    object_target: dict[str, Any] | None = None
    detections: list[dict[str, Any]] | None = None
    drop_zone: str | dict[str, Any] | None = None
    detection: dict[str, Any] | None = None
    settings: PathSettingsRequest | None = None
    branch: str = "auto"


class TaskExecuteRequest(BaseModel):
    preview_id: str


def public_config() -> dict[str, Any]:
    return {
        "joints": [asdict(joint) for joint in config.joints],
        "links_mm": asdict(config.links),
        "kinematics": asdict(config.kinematics),
        "motion": asdict(config.motion),
        "serial": asdict(config.serial),
        "simulation_default": config.simulation_default,
        "coordinate_frame": COORDINATE_FRAME,
        "coordinate_frame_notes": config.coordinate_frame_notes,
        "config_source": str(config.source_path),
        "named_positions": named_positions(config),
        "camera": camera_settings(config),
        "color_profiles": color_profiles(config),
        "drop_zones": drop_zones(config),
        "task_defaults": task_defaults(config),
        "path_defaults": default_path_settings(),
        "tool": tool_settings(config),
        "tools": tools_settings(config),
        "encoders": encoder_settings(config),
        "calibration": calibration_settings(config),
        "geometry": geometry_settings(config),
        "validation": {
            "model_warnings": model_validation_warnings(config),
            "named_position_errors": named_position_errors(config),
        },
    }


def evaluate_hardware_config() -> dict[str, Any]:
    axis_states: list[str] = []
    enabled_bits: list[str] = []
    errors: list[str] = []
    for index, joint in enumerate(config.joints, start=1):
        state_name = "simulated"
        enabled = False
        if joint.actuator == "stepper" and joint.hardware.stepper:
            hardware = joint.hardware.stepper
            enabled = hardware.enabled
            if enabled:
                missing = []
                if hardware.step_pin < 0:
                    missing.append("step_pin")
                if hardware.dir_pin < 0:
                    missing.append("dir_pin")
                if hardware.motor_full_steps_per_rev <= 0:
                    missing.append("motor_full_steps_per_rev")
                if hardware.microsteps <= 0:
                    missing.append("microsteps")
                if hardware.gear_ratio <= 0:
                    missing.append("gear_ratio")
                if missing:
                    state_name = "invalid"
                    errors.append(f"{joint.name}: missing/invalid {', '.join(missing)}")
                else:
                    state_name = "hardware"
        elif joint.actuator == "servo" and joint.hardware.servo:
            hardware = joint.hardware.servo
            enabled = hardware.enabled
            if enabled:
                missing = []
                if hardware.pwm_pin < 0:
                    missing.append("pwm_pin")
                if hardware.pulse_min_us >= hardware.pulse_max_us:
                    missing.append("pulse_min_us/pulse_max_us")
                if hardware.pwm_frequency_hz <= 0:
                    missing.append("pwm_frequency_hz")
                if hardware.servo_range_deg <= 0:
                    missing.append("servo_range_deg")
                if hardware.gear_ratio <= 0:
                    missing.append("gear_ratio")
                if missing:
                    state_name = "invalid"
                    errors.append(f"{joint.name}: missing/invalid {', '.join(missing)}")
                else:
                    state_name = "hardware"
        else:
            if joint.actuator not in {"stepper", "servo"}:
                errors.append(f"{joint.name}: unsupported actuator {joint.actuator}")
                state_name = "invalid"
        axis_states.append(state_name)
        enabled_bits.append("1" if enabled and state_name == "hardware" else "0")

    errors.extend(active_tool_hardware_errors())
    if any(axis == "invalid" for axis in axis_states):
        mode = "invalid"
    elif errors:
        mode = "invalid"
    elif all(axis == "hardware" for axis in axis_states):
        mode = "hardware"
    elif any(axis == "hardware" for axis in axis_states):
        mode = "mixed"
    else:
        mode = "simulated"
    return {
        "mode": mode,
        "axis_states": axis_states,
        "enabled_axes": "".join(enabled_bits),
        "errors": errors,
    }


def apply_hardware_evaluation(sync_status: str | None = None, message: str | None = None) -> dict[str, Any]:
    evaluation = evaluate_hardware_config()
    state.hardware_mode = evaluation["mode"]
    state.hardware_axis_states = evaluation["axis_states"]
    state.hardware_enabled_axes = evaluation["enabled_axes"]
    if sync_status is not None:
        state.config_sync_status = sync_status
    if message is not None:
        state.config_sync_message = message
    return evaluation


def hardware_ready_for_motion() -> tuple[bool, str]:
    evaluation = apply_hardware_evaluation()
    if evaluation["mode"] == "invalid":
        return False, "; ".join(evaluation["errors"]) or "hardware config is invalid"
    if evaluation["mode"] == "simulated":
        return False, "no hardware axes are enabled"
    if state.config_sync_status != "synced":
        return False, f"hardware config is not synced ({state.config_sync_status})"
    return True, ""


def _tool_pin(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be an integer GPIO or -1 for unknown")
    pin = int(value)
    if pin < -1 or pin > 48:
        raise ValueError(f"{name} must be between -1 and 48")
    return pin


def validate_tools_payload(tools: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    active = str(tools.get("active", "gripper"))
    presets = tools.get("presets")
    if not isinstance(presets, dict) or not presets:
        return ["tools.presets must define at least one tool"]
    if active not in presets:
        errors.append(f"active tool {active} is missing from presets")

    for name, preset in presets.items():
        if not isinstance(preset, dict):
            errors.append(f"tool {name} must be a mapping")
            continue
        tool_type = str(preset.get("type", "generic"))
        if tool_type not in {"servo_gripper", "electromagnet", "generic"}:
            errors.append(f"tool {name} has unsupported type {tool_type}")
        tcp = preset.get("tcp_offset_mm")
        if not isinstance(tcp, dict):
            errors.append(f"tool {name} must define tcp_offset_mm")
            tcp = {}
        for axis in ["x", "y", "z"]:
            try:
                float(tcp.get(axis, 0.0))
            except (TypeError, ValueError):
                errors.append(f"tool {name} tcp_offset_mm.{axis} must be numeric")

        io = preset.get("io") if isinstance(preset.get("io"), dict) else {}
        try:
            if tool_type == "servo_gripper":
                open_value = float(preset.get("open_value", 0.0))
                closed_value = float(preset.get("closed_value", 1.0))
                if not 0.0 <= open_value <= 1.0 or not 0.0 <= closed_value <= 1.0:
                    errors.append(f"tool {name} open_value and closed_value must be in 0.0..1.0")
                _tool_pin(io.get("pwm_pin", -1), f"tool {name} pwm_pin")
                pulse_min = int(io.get("pulse_min_us", 500))
                pulse_max = int(io.get("pulse_max_us", 2500))
                frequency = int(io.get("pwm_frequency_hz", 50))
                if pulse_min <= 0 or pulse_max <= pulse_min:
                    errors.append(f"tool {name} pulse_min_us must be positive and below pulse_max_us")
                if frequency <= 0:
                    errors.append(f"tool {name} pwm_frequency_hz must be positive")
            elif tool_type == "electromagnet":
                _tool_pin(io.get("pin", -1), f"tool {name} pin")
        except (TypeError, ValueError) as exc:
            errors.append(str(exc))
    return errors


def active_tool_hardware_errors() -> list[str]:
    tools = tools_settings(config)
    errors = validate_tools_payload(tools)
    active = str(tools.get("active", "gripper"))
    preset = tools.get("presets", {}).get(active, {})
    tool_type = str(preset.get("type", "generic")) if isinstance(preset, dict) else "generic"
    io = preset.get("io") if isinstance(preset.get("io"), dict) else {}
    try:
        if tool_type == "servo_gripper" and _tool_pin(io.get("pwm_pin", -1), f"tool {active} pwm_pin") < 0:
            errors.append(f"active tool {active} is missing gripper PWM pin")
        if tool_type == "electromagnet" and _tool_pin(io.get("pin", -1), f"tool {active} pin") < 0:
            errors.append(f"active tool {active} is missing magnet GPIO pin")
    except ValueError as exc:
        errors.append(str(exc))
    return errors


def config_sync_ready() -> tuple[bool, str]:
    if state.motion_state == MotionState.MOVING:
        return False, "stop motion before saving or syncing config"
    if state.live_motion_enabled:
        return False, "turn off live motion before saving or syncing config"
    if any(task is not None and not task.done() for task in [path_task, live_task, task_task]):
        return False, "wait for active motion or task execution to finish before syncing config"
    return True, ""


def links_from_override(links_mm: dict[str, float] | None) -> LinkConfig:
    values = config.links.__dict__.copy()
    if links_mm:
        aliases = {
            "l1": "base_height_mm",
            "l2": "upper_arm_mm",
            "l3": "forearm_mm",
            "l4": "tool_total_mm",
            "base_height": "base_height_mm",
            "upper_arm": "upper_arm_mm",
            "forearm": "forearm_mm",
            "wrist": "wrist_mm",
            "tool": "tool_mm",
            "base_side_offset": "base_side_offset_mm",
            "side_offset": "base_side_offset_mm",
        }
        for key, value in links_mm.items():
            normalized_key = aliases.get(key, key)
            if normalized_key == "tool_total_mm":
                values["wrist_mm"] = float(value)
                values["tool_mm"] = 0.0
            elif normalized_key in values:
                values[normalized_key] = float(value)
    return LinkConfig(**values)


def default_path_settings() -> dict[str, Any]:
    defaults = {
        "global_speed_deg_s": min(joint.max_speed_deg_s for joint in config.joints),
        "global_accel_deg_s2": config.motion.acceleration_deg_s2,
        "waypoint_rate_hz": config.motion.command_rate_limit_hz,
        "cartesian_step_mm": 10.0,
        "planner_type": "s_curve",
        "jerk_percent": 25.0,
        "blend_percent": 0.0,
        "per_joint_speed_deg_s": [joint.max_speed_deg_s for joint in config.joints],
        "per_joint_accel_deg_s2": [joint.max_accel_deg_s2 for joint in config.joints],
    }
    stored = config.raw.get("path_defaults")
    if isinstance(stored, dict):
        defaults.update({key: value for key, value in stored.items() if value is not None})
    return defaults


def request_settings(settings: PathSettingsRequest | dict[str, Any] | None) -> dict[str, Any]:
    merged = default_path_settings()
    if settings is None:
        return merged
    values = settings if isinstance(settings, dict) else settings.__dict__
    merged.update({key: value for key, value in values.items() if value is not None})
    return merged


def reset_cartesian_jog_runtime() -> None:
    cartesian_jog_runtime.update(
        {
            "active": False,
            "last_update": 0.0,
            "last_status_poll": 0.0,
            "seed_deg": None,
            "joint_velocity_deg_s": [0.0 for _ in config.joints],
            "run_id": None,
        }
    )


def _clamp_scalar(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def _cartesian_jog_dt(requested_dt_s: float | None) -> float:
    if requested_dt_s is None:
        now = monotonic()
        previous = float(cartesian_jog_runtime.get("last_update") or 0.0)
        if previous > 0.0:
            return _clamp_scalar(now - previous, CARTESIAN_JOG_MIN_DT_S, CARTESIAN_JOG_MAX_DT_S)
        return 1.0 / max(config.motion.command_rate_limit_hz, 20.0)
    return _clamp_scalar(float(requested_dt_s), CARTESIAN_JOG_MIN_DT_S, CARTESIAN_JOG_MAX_DT_S)


def _cartesian_jog_seed(now_s: float) -> list[float]:
    seed = cartesian_jog_runtime.get("seed_deg")
    last_update = float(cartesian_jog_runtime.get("last_update") or 0.0)
    if not cartesian_jog_runtime.get("active") or seed is None or now_s - last_update > CARTESIAN_JOG_STALE_S:
        cartesian_jog_runtime["joint_velocity_deg_s"] = [0.0 for _ in config.joints]
        return [float(value) for value in state.reported_angles_deg]
    if state.simulation:
        return [float(value) for value in state.reported_angles_deg]
    return [float(value) for value in seed]


def _cartesian_jog_task_delta(
    vx_mm_s: float,
    vy_mm_s: float,
    vz_mm_s: float,
    vphi_deg_s: float,
    dt_s: float,
) -> dict[str, float]:
    """Convert one velocity sample into a local resolved-rate IK step.

    Do not accumulate an endpoint goal here. A rejected local step must not
    remain hidden in the next request, otherwise reversing the fader continues
    chasing the stale unreachable goal until the operator releases it.
    """
    return {
        "x_mm": float(vx_mm_s) * dt_s,
        "y_mm": float(vy_mm_s) * dt_s,
        "z_mm": float(vz_mm_s) * dt_s,
        "phi_deg": float(vphi_deg_s) * dt_s,
    }


def _joint_limits_from_settings(settings: dict[str, Any]) -> tuple[list[float], list[float]]:
    speed_limits = []
    accel_limits = []
    per_speed = settings.get("per_joint_speed_deg_s")
    per_accel = settings.get("per_joint_accel_deg_s2")
    for index, joint in enumerate(config.joints):
        speed = float(settings.get("global_speed_deg_s") or joint.max_speed_deg_s)
        accel = float(settings.get("global_accel_deg_s2") or config.motion.acceleration_deg_s2)
        if isinstance(per_speed, list) and index < len(per_speed) and per_speed[index] is not None:
            speed = min(speed, float(per_speed[index]))
        if isinstance(per_accel, list) and index < len(per_accel) and per_accel[index] is not None:
            accel = min(accel, float(per_accel[index]))
        speed_limits.append(max(0.01, min(joint.max_speed_deg_s, speed)))
        accel_limits.append(max(0.01, min(joint.max_accel_deg_s2, accel)))
    return speed_limits, accel_limits


def _limit_cartesian_jog_target(
    seed_deg: list[float],
    requested_target_deg: list[float],
    dt_s: float,
    speed_limits_deg_s: list[float],
    accel_limits_deg_s2: list[float],
) -> tuple[list[float], list[float], list[str]]:
    previous_velocity = [
        float(value)
        for value in (cartesian_jog_runtime.get("joint_velocity_deg_s") or [0.0 for _ in config.joints])
    ]
    desired_velocity = [
        (float(requested) - float(seed)) / max(dt_s, 1e-6)
        for seed, requested in zip(seed_deg, requested_target_deg, strict=True)
    ]
    speed_scale = 1.0
    for velocity, limit in zip(desired_velocity, speed_limits_deg_s, strict=True):
        if abs(velocity) > limit:
            speed_scale = min(speed_scale, limit / max(abs(velocity), 1e-9))
    desired_velocity = [velocity * speed_scale for velocity in desired_velocity]

    velocity_delta = [
        desired - previous
        for desired, previous in zip(desired_velocity, previous_velocity, strict=True)
    ]
    accel_scale = 1.0
    for delta_v, limit in zip(velocity_delta, accel_limits_deg_s2, strict=True):
        max_velocity_delta = limit * dt_s
        if abs(delta_v) > max_velocity_delta:
            accel_scale = min(accel_scale, max_velocity_delta / max(abs(delta_v), 1e-9))
    limited_velocity = [
        previous + delta_v * accel_scale
        for previous, delta_v in zip(previous_velocity, velocity_delta, strict=True)
    ]

    target: list[float] = []
    velocities: list[float] = []
    notes: list[str] = []
    for index, (seed, velocity, joint) in enumerate(zip(seed_deg, limited_velocity, config.joints, strict=True)):
        next_angle = float(seed) + velocity * dt_s
        clamped = _clamp_scalar(next_angle, joint.min_deg, joint.max_deg)
        if abs(clamped - next_angle) > 1e-6:
            velocity = (clamped - float(seed)) / max(dt_s, 1e-6)
            notes.append(f"{joint.name} limit")
        target.append(clamped)
        velocities.append(velocity)
    return target, velocities, notes


def cancel_task(task: asyncio.Task[None] | None) -> None:
    if task is not None and not task.done():
        task.cancel()


def cancel_motion_tasks() -> None:
    cancel_task(path_task)
    cancel_task(live_task)
    cancel_task(task_task)
    reset_cartesian_jog_runtime()


def disable_live_motion(command: str | None = None) -> None:
    state.live_motion_enabled = False
    reset_cartesian_jog_runtime()
    if command:
        state.last_command = command


def log_event(source: str, message: str, **data: Any) -> None:
    event_log.add(source, message, **data)


def list_serial_ports() -> list[dict[str, Any]]:
    try:
        from serial.tools import list_ports
    except Exception:
        return []
    ports = []
    for port in list_ports.comports():
        ports.append(
            {
                "device": port.device,
                "name": port.name,
                "description": port.description,
                "hwid": port.hwid,
                "manufacturer": port.manufacturer,
            }
        )
    return ports


def update_encoder_verification() -> None:
    settings = encoder_settings(config)
    fault_tolerance = float(settings.get("fault_tolerance_deg", 5.0))
    errors: list[float | None] = []
    fault = False
    for index, encoder_angle in enumerate(state.encoder_angles_deg):
        available = index < len(state.encoder_available) and state.encoder_available[index] == "1"
        if available and encoder_angle is not None:
            error = float(encoder_angle) - float(state.target_angles_deg[index])
            errors.append(error)
            if abs(error) > fault_tolerance:
                fault = True
        else:
            errors.append(None)
    state.encoder_errors_deg = errors
    state.encoder_fault = fault
    if fault:
        state.set_error("encoder verification exceeded fault tolerance", fault=True)


def read_serial_until_any(prefixes: tuple[str, ...], timeout_s: float = 2.0) -> str:
    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        line = serial_client.read_line()
        if any(line.startswith(prefix) for prefix in prefixes):
            return line
    raise SerialClientError(f"timed out waiting for {'/'.join(prefixes)}")


def joint_errors_deg(target_deg: list[float], reported_deg: list[float]) -> list[float]:
    return [
        float(reported) - float(target)
        for target, reported in zip(target_deg, reported_deg, strict=True)
    ]


def tcp_sample_from_state() -> dict[str, float]:
    fk = state.fk or forward_kinematics(state.reported_angles_deg, config.links)
    return {
        "x_mm": float(fk.get("x_mm", 0.0)),
        "y_mm": float(fk.get("y_mm", 0.0)),
        "z_mm": float(fk.get("z_mm", 0.0)),
        "ts": time(),
    }


def _tcp_distance_mm(a: dict[str, Any], b: dict[str, Any]) -> float:
    return (
        (float(a.get("x_mm", 0.0)) - float(b.get("x_mm", 0.0))) ** 2
        + (float(a.get("y_mm", 0.0)) - float(b.get("y_mm", 0.0))) ** 2
        + (float(a.get("z_mm", 0.0)) - float(b.get("z_mm", 0.0))) ** 2
    ) ** 0.5


def _joint_progress_ratio(start_deg: list[float], target_deg: list[float], current_deg: list[float]) -> float:
    total = sum((target - start) ** 2 for start, target in zip(start_deg, target_deg, strict=True)) ** 0.5
    if total <= 1e-9:
        return 1.0
    remaining = sum((target - current) ** 2 for target, current in zip(target_deg, current_deg, strict=True)) ** 0.5
    return max(0.0, min(1.0, 1.0 - remaining / total))


def _motion_run_matches(run_id: str | None) -> bool:
    if run_id is None:
        return True
    return state.motion_diagnostics.get("run_id") == run_id


def axis_modes() -> list[str]:
    evaluation = evaluate_hardware_config()
    return list(evaluation.get("axis_states", ["unknown"] * len(config.joints)))


def start_motion_diagnostics(
    *,
    source: str,
    mode: str,
    target_deg: list[float],
    expected_duration_s: float,
    waypoint_count: int,
    step_label: str = "",
    step_index: int = 0,
    step_total: int = 0,
) -> str:
    global active_motion_run_id
    run_id = str(uuid4())
    active_motion_run_id = run_id
    initial_sample = tcp_sample_from_state()
    state.motion_execution_state = "queued"
    state.motion_diagnostics = {
        "run_id": run_id,
        "source": source,
        "mode": mode,
        "execution_state": "queued",
        "requested_target_deg": [float(value) for value in target_deg],
        "start_reported_deg": [float(value) for value in state.reported_angles_deg],
        "expected_duration_s": float(expected_duration_s),
        "elapsed_s": 0.0,
        "waypoint_count": int(waypoint_count),
        "current_waypoint_index": 0,
        "current_waypoint_total": int(waypoint_count),
        "active_step_label": step_label,
        "active_step_index": int(step_index),
        "active_step_total": int(step_total),
        "progress_ratio": 0.0,
        "axis_modes": axis_modes(),
        "started_at": time(),
        "controller_response": "",
        "last_status_line": state.last_status_line,
        "final_reported_deg": None,
        "final_error_deg": None,
        "actual_tcp_path": [initial_sample],
        "last_tcp_sample": initial_sample,
        "actual_duration_s": None,
        "result": "queued",
    }
    return run_id


def update_motion_diagnostics(run_id: str | None = None, **updates: Any) -> None:
    if not _motion_run_matches(run_id):
        return
    diagnostics = dict(state.motion_diagnostics or {})
    diagnostics.update(updates)
    state.motion_diagnostics = diagnostics
    if "execution_state" in updates:
        state.motion_execution_state = str(updates["execution_state"])


def record_motion_sample(run_id: str | None = None) -> None:
    if not state.motion_diagnostics or not _motion_run_matches(run_id):
        return
    if state.motion_diagnostics.get("result") not in {"queued", "executing"}:
        return
    diagnostics = dict(state.motion_diagnostics)
    sample = tcp_sample_from_state()
    path = list(diagnostics.get("actual_tcp_path") or [])
    last_sample = diagnostics.get("last_tcp_sample") or (path[-1] if path else None)
    elapsed_since_last = sample["ts"] - float((last_sample or {}).get("ts", 0.0))
    moved_mm = _tcp_distance_mm(sample, last_sample) if last_sample else 0.0
    if not path or moved_mm >= 1.0 or elapsed_since_last >= 0.25:
        path.append(sample)
        if len(path) > 360:
            path = path[-360:]
        diagnostics["actual_tcp_path"] = path
        diagnostics["last_tcp_sample"] = sample

    start = [float(value) for value in diagnostics.get("start_reported_deg", state.reported_angles_deg)]
    target = [float(value) for value in diagnostics.get("requested_target_deg", state.target_angles_deg)]
    diagnostics["elapsed_s"] = max(0.0, time() - float(diagnostics.get("started_at", time())))
    diagnostics["progress_ratio"] = _joint_progress_ratio(start, target, state.reported_angles_deg)
    state.motion_diagnostics = diagnostics


def maybe_finish_reached_motion() -> None:
    if not active_motion_run_id or not state.motion_diagnostics:
        return
    if state.motion_diagnostics.get("result") not in {"queued", "executing"}:
        return
    target = [float(value) for value in state.motion_diagnostics.get("requested_target_deg", state.target_angles_deg)]
    tolerance = float(calibration_settings(config).get("movement_tolerance_deg", 0.2))
    if state.motion_state == MotionState.IDLE and all(abs(error) <= tolerance for error in joint_errors_deg(target, state.reported_angles_deg)):
        finish_motion_diagnostics("reached", run_id=active_motion_run_id)


def finish_motion_diagnostics(result: str, error: str | None = None, run_id: str | None = None) -> None:
    global active_motion_run_id
    if not _motion_run_matches(run_id):
        return
    record_motion_sample(run_id)
    diagnostics = dict(state.motion_diagnostics or {})
    started_at = float(diagnostics.get("started_at", time()))
    target = diagnostics.get("requested_target_deg") or state.target_angles_deg
    final_error = joint_errors_deg([float(value) for value in target], state.reported_angles_deg)
    diagnostics.update(
        {
            "result": result,
            "execution_state": result,
            "progress_ratio": 1.0 if result == "reached" else float(diagnostics.get("progress_ratio", 0.0)),
            "error": error or "",
            "final_reported_deg": [float(value) for value in state.reported_angles_deg],
            "final_error_deg": final_error,
            "actual_duration_s": max(0.0, time() - started_at),
            "elapsed_s": max(0.0, time() - started_at),
            "controller_response": state.last_controller_response,
            "last_status_line": state.last_status_line,
        }
    )
    state.motion_diagnostics = diagnostics
    state.motion_execution_state = result
    if run_id is None or active_motion_run_id == run_id:
        active_motion_run_id = None


def send_movej_and_read_response(command: str) -> str:
    serial_client.clear_input()
    serial_client.send_line(command)
    response = read_serial_until_any(("OK command=MOVEJ", "ERR"), timeout_s=1.0)
    state.last_controller_response = response
    update_motion_diagnostics(controller_response=response, execution_state="accepted" if response.startswith("OK") else "failed")
    if response.startswith("ERR"):
        raise SerialClientError(response)
    return response


def send_jogj_and_read_response(command: str) -> str:
    serial_client.clear_input()
    serial_client.send_line(command)
    response = read_serial_until_any(("OK command=JOGJ", "ERR"), timeout_s=0.75)
    state.last_controller_response = response
    if response.startswith("ERR"):
        raise SerialClientError(response)
    return response


def send_jogv_and_read_response(command: str) -> str:
    serial_client.clear_input()
    serial_client.send_line(command)
    response = read_serial_until_any(("OK command=JOGV", "ERR"), timeout_s=0.75)
    state.last_controller_response = response
    if response.startswith("ERR"):
        raise SerialClientError(response)
    return response


def send_jog_stop_and_read_response() -> str:
    serial_client.clear_input()
    serial_client.send_line(format_jog_stop())
    response = read_serial_until_any(("OK command=JOG_STOP", "ERR"), timeout_s=0.75)
    state.last_controller_response = response
    if response.startswith("ERR"):
        raise SerialClientError(response)
    return response


def _trajectory_times_s(trajectory: dict[str, Any], waypoint_count: int) -> list[float]:
    raw_times = trajectory.get("time_from_start_s")
    if isinstance(raw_times, list) and len(raw_times) == waypoint_count:
        times = [max(0.0, float(value)) for value in raw_times]
    else:
        elapsed = 0.0
        times = []
        durations = trajectory.get("segment_durations_s", [])
        for index in range(waypoint_count):
            if index < len(durations):
                elapsed += max(0.0, float(durations[index]))
            elif waypoint_count > 1:
                elapsed = float(trajectory.get("duration_s", elapsed)) * index / (waypoint_count - 1)
            times.append(elapsed)

    if not times:
        return []
    first_time = times[0]
    if first_time != 0.0:
        times = [max(0.0, value - first_time) for value in times]
    for index in range(1, len(times)):
        if times[index] <= times[index - 1]:
            times[index] = times[index - 1] + 0.001
    return times


def _interpolate_timed_waypoint(
    waypoints: list[list[float]],
    times_s: list[float],
    sample_time_s: float,
) -> list[float]:
    if sample_time_s <= times_s[0]:
        return waypoints[0].copy()
    if sample_time_s >= times_s[-1]:
        return waypoints[-1].copy()
    for index in range(len(times_s) - 1):
        t0 = times_s[index]
        t1 = times_s[index + 1]
        if sample_time_s <= t1:
            fraction = (sample_time_s - t0) / max(t1 - t0, 1e-9)
            return [
                float(start) + (float(end) - float(start)) * fraction
                for start, end in zip(waypoints[index], waypoints[index + 1], strict=True)
            ]
    return waypoints[-1].copy()


def trajectory_upload_points(
    trajectory: dict[str, Any],
    max_points: int = MAX_TRAJECTORY_UPLOAD_POINTS,
) -> list[tuple[float, list[float]]]:
    waypoints = [[float(value) for value in waypoint] for waypoint in trajectory.get("waypoints", [])]
    if len(waypoints) < 2:
        return [(0.0, waypoints[0])] if waypoints else []
    times = _trajectory_times_s(trajectory, len(waypoints))
    if len(times) != len(waypoints):
        return []
    if len(waypoints) <= max_points:
        return list(zip(times, waypoints, strict=True))

    duration_s = times[-1]
    if duration_s <= 0.0:
        return [(0.0, waypoints[0]), (0.001, waypoints[-1])]
    sample_count = max(2, max_points)
    samples: list[tuple[float, list[float]]] = []
    for index in range(sample_count):
        sample_time = duration_s * index / (sample_count - 1)
        samples.append((sample_time, _interpolate_timed_waypoint(waypoints, times, sample_time)))
    return samples


def send_trajectory_and_read_response(
    trajectory: dict[str, Any],
    speed_deg_s: float,
    accel_deg_s2: float,
) -> dict[str, Any]:
    points = trajectory_upload_points(trajectory)
    if len(points) < 2:
        raise SerialClientError("trajectory upload requires at least two timed points")
    duration_s = points[-1][0]
    if duration_s <= 0.0:
        raise SerialClientError("trajectory upload duration must be positive")

    serial_client.clear_input()
    begin_line = format_traj_begin(len(points), duration_s, speed_deg_s, accel_deg_s2)
    serial_client.send_line(begin_line)
    response = read_serial_until_any(("OK command=TRAJ_BEGIN", "ERR"), timeout_s=1.0)
    if response.startswith("ERR"):
        if "UNKNOWN" in response or "unknown" in response:
            raise SerialClientError("controller firmware does not support TRAJ; upload the updated ESP firmware")
        raise SerialClientError(response)

    for index, (time_s, joints_deg) in enumerate(points):
        serial_client.send_line(format_traj_point(index, time_s, joints_deg))
        response = read_serial_until_any(("OK command=TRAJ_POINT", "ERR"), timeout_s=1.0)
        if response.startswith("ERR"):
            raise SerialClientError(response)

    serial_client.send_line(format_traj_start())
    response = read_serial_until_any(("OK command=TRAJ_START", "ERR"), timeout_s=1.0)
    state.last_controller_response = response
    update_motion_diagnostics(
        controller_response=response,
        execution_state="accepted" if response.startswith("OK") else "failed",
        uploaded_waypoint_count=len(points),
        uploaded_duration_s=duration_s,
    )
    if response.startswith("ERR"):
        raise SerialClientError(response)
    return {"response": response, "point_count": len(points), "duration_s": duration_s}


async def wait_for_hardware_target(
    target_deg: list[float],
    timeout_s: float,
    tolerance_deg: float = 0.15,
    poll_interval_s: float = 0.08,
) -> tuple[bool, str]:
    deadline = monotonic() + max(timeout_s, poll_interval_s)
    last_error: list[float] = []
    while monotonic() < deadline:
        if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
            return False, f"motion stopped while waiting for hardware ({state.motion_state.value})"
        try:
            refresh_serial_status()
        except SerialClientError as exc:
            return False, str(exc)
        last_error = joint_errors_deg(target_deg, state.reported_angles_deg)
        if state.motion_state == MotionState.IDLE and all(abs(error) <= tolerance_deg for error in last_error):
            return True, "target reached"
        await asyncio.sleep(poll_interval_s)
    error_text = ", ".join(f"{value:.2f}" for value in last_error) if last_error else "unknown"
    return False, f"hardware target timeout after {timeout_s:.2f}s; joint errors deg=[{error_text}]"


def sync_hardware_config() -> dict[str, Any]:
    evaluation = apply_hardware_evaluation()
    ready, reason = config_sync_ready()
    if not ready:
        state.config_sync_status = "blocked"
        state.config_sync_message = reason
        return {"ok": False, "status": state.config_sync_status, "evaluation": evaluation, "message": reason}
    if not serial_client.is_connected:
        state.config_sync_status = "not_connected"
        state.config_sync_message = "serial port is not connected"
        return {"ok": False, "status": state.config_sync_status, "evaluation": evaluation, "message": state.config_sync_message}
    if evaluation["mode"] == "invalid":
        state.config_sync_status = "invalid"
        state.config_sync_message = "; ".join(evaluation["errors"]) or "hardware config is invalid"
        return {"ok": False, "status": state.config_sync_status, "evaluation": evaluation, "message": state.config_sync_message}

    try:
        serial_client.clear_input()
        for line in format_config_lines(config.joints, tools_settings(config)):
            serial_client.send_line(line)
        response = read_serial_until_any(("OK command=CONFIG", "ERR"), timeout_s=2.0)
    except SerialClientError as exc:
        state.config_sync_status = "failed"
        state.config_sync_message = str(exc)
        return {"ok": False, "status": state.config_sync_status, "evaluation": evaluation, "message": state.config_sync_message}

    if response.startswith("OK command=CONFIG"):
        state.config_sync_status = "synced"
        state.config_sync_message = response
        state.last_command = response
        state.clear_error()
        log_event("controller", response)
        return {"ok": True, "status": state.config_sync_status, "evaluation": evaluation, "response": response}

    if "UNKNOWN" in response or "unknown" in response:
        state.config_sync_status = "unsupported"
        state.config_sync_message = response
    else:
        state.config_sync_status = "failed"
        state.config_sync_message = response
    return {"ok": False, "status": state.config_sync_status, "evaluation": evaluation, "response": response}


def build_preview(
    *,
    mode: str,
    target: dict[str, Any] | None,
    waypoint_program: list[dict[str, Any]] | None,
    links: LinkConfig,
    settings: dict[str, Any],
    branch: str,
    source: str = "preview",
) -> dict[str, Any]:
    mode = mode.lower()
    ik_result: dict[str, Any] | None = None

    if waypoint_program:
        trajectory = build_program_trajectory(
            state.reported_angles_deg,
            waypoint_program,
            links,
            config.joints,
            settings,
            branch,
        )
        if not trajectory["ok"]:
            return {
                "ok": False,
                "error": "; ".join(trajectory.get("errors", [])) or "program preview failed",
                "trajectory": trajectory,
            }
        preview_target = target or {}
        preview_mode = "program"
    else:
        if target is None:
            return {"ok": False, "error": "path preview requires target or waypoints"}
        ik_result = inverse_kinematics(target, links, config.joints, state.reported_angles_deg, branch)
        if not ik_result["ok"] or not ik_result["selected"]:
            return {"ok": False, "error": "IK target has no valid solution", "ik": ik_result}

        resolved_target = dict(ik_result["target"])
        if mode == "linear":
            movement_target = dict(resolved_target)
            movement_target["phi_auto"] = False
            trajectory = build_linear_cartesian_trajectory(
                state.reported_angles_deg,
                movement_target,
                links,
                config.joints,
                settings,
                branch,
            )
        else:
            mode = "joint"
            trajectory = build_joint_trajectory(
                state.reported_angles_deg,
                [float(value) for value in ik_result["selected"]["angles_deg"]],
                config.joints,
                settings,
            )
        if not trajectory["ok"]:
            return {
                "ok": False,
                "error": "; ".join(trajectory.get("errors", [])) or "path preview failed",
                "ik": ik_result,
                "trajectory": trajectory,
            }
        preview_target = resolved_target
        preview_mode = mode

    preview_id = str(uuid4())
    preview = {
        "id": preview_id,
        "created_at": time(),
        "source": source,
        "mode": preview_mode,
        "target": preview_target,
        "settings": settings,
        "ik": ik_result,
        "trajectory": trajectory,
        "completion_feedback": "timed + STATUS estimate for hardware",
    }
    path_previews[preview_id] = preview
    for stale_id, stale in list(path_previews.items()):
        if time() - stale.get("created_at", 0.0) > 600:
            path_previews.pop(stale_id, None)
    log_event("motion", f"preview {preview_mode}", preview_id=preview_id, waypoint_count=trajectory.get("waypoint_count", 0))
    return {"ok": True, "preview_id": preview_id, "preview": preview}


async def broadcast_state() -> None:
    message = {"type": "state", "state": state.to_dict()}
    stale: list[WebSocket] = []
    for websocket in list(websockets):
        try:
            await websocket.send_json(message)
        except Exception:
            stale.append(websocket)
    for websocket in stale:
        websockets.discard(websocket)


def set_targets(
    targets: list[float],
    command_label: str = "set_targets",
    speed_deg_s: float | None = None,
    accel_deg_s2: float | None = None,
) -> dict[str, Any]:
    if state.motion_state == MotionState.ESTOP:
        state.set_error("emergency stop is active")
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.connected and not state.simulation:
        state.set_error("not connected to hardware and simulation is disabled")
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.simulation and not state.hardware_armed:
        state.set_error("hardware moves require the Armed toggle")
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.simulation:
        ready, reason = hardware_ready_for_motion()
        if not ready:
            state.set_error(reason)
            return {"ok": False, "error": reason, "state": state.to_dict()}
    can_move = validate_can_move(state)
    if not can_move.ok:
        state.set_error(can_move.reason)
        return {"ok": False, "error": can_move.reason, "state": state.to_dict()}

    start_result = validate_joint_targets(config, state.reported_angles_deg)
    if not start_result.ok:
        state.set_error(f"reported pose is outside limits: {start_result.reason}")
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    result = validate_joint_targets(config, targets)
    if not result.ok:
        state.set_error(result.reason)
        return {"ok": False, "error": result.reason, "state": state.to_dict()}

    speed = speed_deg_s if speed_deg_s and speed_deg_s > 0 else min(joint.max_speed_deg_s for joint in config.joints)
    accel = accel_deg_s2 if accel_deg_s2 and accel_deg_s2 > 0 else config.motion.acceleration_deg_s2
    if speed <= 0 or accel <= 0:
        state.set_error("speed and acceleration must be positive")
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    if state.motion_diagnostics.get("result") not in {"queued", "executing"}:
        run_id = start_motion_diagnostics(
            source=command_label,
            mode="joint_endpoint",
            target_deg=[float(value) for value in targets],
            expected_duration_s=0.0,
            waypoint_count=1,
        )
        update_motion_diagnostics(
            run_id,
            execution_state="executing",
            result="executing",
            current_waypoint_index=1,
            current_waypoint_total=1,
            active_target_deg=[float(value) for value in targets],
        )

    state.target_angles_deg = [float(value) for value in targets]
    limiter.current_deg = state.reported_angles_deg.copy()
    limiter.set_target(state.target_angles_deg)
    state.motion_state = MotionState.MOVING
    state.motion_execution_state = "command_sent"
    state.clear_error()

    command = format_movej(state.target_angles_deg, speed, accel)
    state.last_command = command
    state.updated_at = time()

    if not state.simulation and serial_client.is_connected:
        try:
            response = send_movej_and_read_response(command)
            refresh_serial_status()
        except SerialClientError as exc:
            state.set_error(str(exc), fault=True)
            finish_motion_diagnostics("failed", str(exc))
            return {"ok": False, "error": str(exc), "state": state.to_dict()}
    else:
        response = "SIMULATION"
        state.last_controller_response = response
        update_motion_diagnostics(controller_response=response, execution_state="accepted")

    return {"ok": True, "command": command_label, "command_line": command, "controller_response": response, "state": state.to_dict()}


async def start_joint_target_trajectory(
    targets: list[float],
    command_label: str,
    settings_payload: PathSettingsRequest | dict[str, Any] | None = None,
) -> dict[str, Any]:
    global path_task, path_task_source

    if path_task is not None and not path_task.done():
        replaceable_sources = {"set_joint_target", "set_all_joint_targets", "ws_set_joint", "ws_set_all"}
        if command_label in replaceable_sources and path_task_source in replaceable_sources:
            cancel_task(path_task)
        else:
            state.set_error("a path is already executing")
            await broadcast_state()
            return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if live_task is not None and not live_task.done():
        state.set_error("live motion is already executing")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.simulation and not state.hardware_armed:
        state.set_error("hardware moves require the Armed toggle")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.simulation:
        ready, reason = hardware_ready_for_motion()
        if not ready:
            state.set_error(reason)
            await broadcast_state()
            return {"ok": False, "error": reason, "state": state.to_dict()}

    can_move = validate_can_move(state)
    if not can_move.ok:
        state.set_error(can_move.reason)
        await broadcast_state()
        return {"ok": False, "error": can_move.reason, "state": state.to_dict()}

    settings = request_settings(settings_payload)
    trajectory = build_joint_trajectory(
        state.reported_angles_deg,
        [float(value) for value in targets],
        config.joints,
        settings,
    )
    if not trajectory["ok"]:
        state.set_error("; ".join(trajectory.get("errors", [])) or "joint trajectory failed")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "trajectory": trajectory, "state": state.to_dict()}

    preview_id = str(uuid4())
    preview = {
        "id": preview_id,
        "created_at": time(),
        "source": command_label,
        "mode": "joint",
        "target": {},
        "settings": settings,
        "ik": None,
        "trajectory": trajectory,
        "completion_feedback": "timed + STATUS estimate for hardware",
    }
    path_previews[preview_id] = preview
    state.clear_error()
    state.last_command = f"{command_label.upper()} {preview_id}"
    path_task_source = command_label
    path_task = asyncio.create_task(execute_joint_endpoint_move(preview))
    await broadcast_state()
    return {"ok": True, "command": command_label, "preview_id": preview_id, "preview": preview, "state": state.to_dict()}


async def execute_joint_endpoint_move(preview: dict[str, Any]) -> None:
    trajectory = preview["trajectory"]
    waypoints = trajectory.get("waypoints", [])
    if not waypoints:
        state.set_error("joint trajectory has no waypoints", fault=True)
        finish_motion_diagnostics("failed", state.last_error)
        await broadcast_state()
        return

    target = [float(value) for value in waypoints[-1]]
    settings = preview.get("settings", {})
    speed = settings.get("global_speed_deg_s")
    accel = settings.get("global_accel_deg_s2")
    expected_duration = float(trajectory.get("duration_s", 0.0))
    run_id = start_motion_diagnostics(
        source=str(preview.get("source", "joint")),
        mode="joint_endpoint",
        target_deg=target,
        expected_duration_s=expected_duration,
        waypoint_count=1,
        step_label=str(preview.get("task_step_label", "")),
        step_index=int(preview.get("task_step_index", 0) or 0),
        step_total=int(preview.get("task_step_total", 0) or 0),
    )
    update_motion_diagnostics(
        run_id,
        execution_state="executing",
        result="executing",
        current_waypoint_index=1,
        current_waypoint_total=1,
        active_target_deg=target,
    )

    try:
        response = set_targets(
            target,
            f"{preview.get('source', 'joint')}_endpoint",
            speed_deg_s=speed,
            accel_deg_s2=accel,
        )
        await broadcast_state()
        if not response["ok"]:
            finish_motion_diagnostics("failed", response.get("error", "joint endpoint failed"), run_id)
            return

        if state.simulation:
            deadline = monotonic() + max(1.0, expected_duration * 4.0 + 0.5)
            while monotonic() < deadline:
                if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
                    finish_motion_diagnostics("stopped", state.motion_state.value, run_id)
                    return
                if has_reached_target(state.reported_angles_deg, target, tolerance_deg=0.08):
                    state.motion_state = MotionState.IDLE
                    finish_motion_diagnostics("reached", run_id=run_id)
                    return
                await asyncio.sleep(0.03)
            state.set_error("simulation target timeout", fault=True)
            finish_motion_diagnostics("failed", state.last_error, run_id)
            return

        ok, message = await wait_for_hardware_target(
            target,
            timeout_s=max(1.0, expected_duration * 2.0 + 1.0),
            tolerance_deg=float(calibration_settings(config).get("movement_tolerance_deg", 0.2)),
        )
        if ok:
            finish_motion_diagnostics("reached", run_id=run_id)
        else:
            state.set_error(message, fault=True)
            finish_motion_diagnostics("failed", message, run_id)
    except asyncio.CancelledError:
        finish_motion_diagnostics("stopped", "cancelled", run_id)
        raise
    finally:
        await broadcast_state()


async def execute_simulated_waypoint_trajectory(trajectory: dict[str, Any], run_id: str) -> None:
    global simulation_trajectory_active
    waypoints = [[float(value) for value in waypoint] for waypoint in trajectory.get("waypoints", [])]
    if len(waypoints) < 2:
        state.set_error("simulation trajectory requires at least two waypoints", fault=True)
        finish_motion_diagnostics("failed", state.last_error, run_id)
        await broadcast_state()
        return

    times_s = _trajectory_times_s(trajectory, len(waypoints))
    if len(times_s) != len(waypoints) or times_s[-1] <= 0.0:
        state.set_error("simulation trajectory timing is invalid", fault=True)
        finish_motion_diagnostics("failed", state.last_error, run_id)
        await broadcast_state()
        return

    simulation_trajectory_active = True
    final_target = waypoints[-1]
    state.target_angles_deg = final_target.copy()
    state.motion_state = MotionState.MOVING
    state.motion_execution_state = "executing"
    state.clear_error()
    limiter.reset(state.reported_angles_deg)
    started = monotonic()
    sample_interval_s = 1.0 / max(config.motion.update_rate_hz, 1.0)

    try:
        while True:
            if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
                finish_motion_diagnostics("stopped", state.motion_state.value, run_id)
                return

            elapsed_s = min(monotonic() - started, times_s[-1])
            current = _interpolate_timed_waypoint(waypoints, times_s, elapsed_s)
            state.reported_angles_deg = current
            state.fk = forward_kinematics(current, config.links)
            limiter.reset(current)
            waypoint_index = 1
            for index, waypoint_time_s in enumerate(times_s):
                if elapsed_s >= waypoint_time_s:
                    waypoint_index = index + 1
            update_motion_diagnostics(
                run_id,
                execution_state="executing",
                result="executing",
                current_waypoint_index=waypoint_index,
                current_waypoint_total=len(waypoints),
                active_target_deg=current,
            )
            record_motion_sample(run_id)
            await broadcast_state()

            if elapsed_s >= times_s[-1]:
                state.reported_angles_deg = final_target.copy()
                state.target_angles_deg = final_target.copy()
                state.fk = forward_kinematics(final_target, config.links)
                limiter.reset(final_target)
                state.motion_state = MotionState.IDLE
                finish_motion_diagnostics("reached", run_id=run_id)
                await broadcast_state()
                return
            await asyncio.sleep(sample_interval_s)
    except asyncio.CancelledError:
        finish_motion_diagnostics("stopped", "cancelled", run_id)
        raise
    finally:
        simulation_trajectory_active = False


async def execute_waypoint_path(preview: dict[str, Any]) -> None:
    trajectory = preview["trajectory"]
    waypoints = trajectory.get("waypoints", [])
    segment_durations = trajectory.get("segment_durations_s", [])
    settings = preview.get("settings", {})
    speed = settings.get("global_speed_deg_s")
    accel = settings.get("global_accel_deg_s2")
    final_target = [float(value) for value in waypoints[-1]] if waypoints else state.target_angles_deg.copy()
    run_id = start_motion_diagnostics(
        source=str(preview.get("source", "path")),
        mode=str(trajectory.get("mode", preview.get("mode", "path"))),
        target_deg=final_target,
        expected_duration_s=float(trajectory.get("duration_s", 0.0)),
        waypoint_count=len(waypoints),
        step_label=str(preview.get("task_step_label", "")),
        step_index=int(preview.get("task_step_index", 0) or 0),
        step_total=int(preview.get("task_step_total", 0) or 0),
    )

    if not waypoints:
        state.set_error("trajectory has no waypoints", fault=True)
        finish_motion_diagnostics("failed", state.last_error, run_id)
        await broadcast_state()
        return

    if not state.simulation:
        speed_value = (
            float(speed)
            if speed is not None and float(speed) > 0
            else min(joint.max_speed_deg_s for joint in config.joints)
        )
        accel_value = (
            float(accel)
            if accel is not None and float(accel) > 0
            else config.motion.acceleration_deg_s2
        )
        try:
            if not serial_client.is_connected:
                raise SerialClientError("trajectory execution requires serial hardware connection")
            update_motion_diagnostics(
                run_id,
                execution_state="uploading",
                result="executing",
                current_waypoint_index=0,
                current_waypoint_total=len(waypoints),
                active_target_deg=final_target,
            )
            state.motion_state = MotionState.MOVING
            state.last_command = f"TRAJ_UPLOAD {len(waypoints)}"
            state.clear_error()
            await broadcast_state()

            upload = send_trajectory_and_read_response(trajectory, speed_value, accel_value)
            state.target_angles_deg = final_target.copy()
            limiter.current_deg = state.reported_angles_deg.copy()
            limiter.set_target(state.target_angles_deg)
            state.last_command = format_traj_start()
            state.motion_state = MotionState.MOVING
            update_motion_diagnostics(
                run_id,
                execution_state="executing",
                result="executing",
                current_waypoint_index=len(waypoints),
                current_waypoint_total=len(waypoints),
                active_target_deg=final_target,
                uploaded_waypoint_count=upload["point_count"],
                uploaded_duration_s=upload["duration_s"],
            )
            log_event(
                "motion",
                "trajectory uploaded",
                preview_id=preview.get("id", ""),
                waypoint_count=len(waypoints),
                uploaded_waypoint_count=upload["point_count"],
            )
            await broadcast_state()

            ok, message = await wait_for_hardware_target(
                final_target,
                timeout_s=max(1.0, float(trajectory.get("duration_s", upload["duration_s"])) * 2.0 + 2.0),
                tolerance_deg=float(calibration_settings(config).get("movement_tolerance_deg", 0.2)),
                poll_interval_s=0.20,
            )
            if ok:
                finish_motion_diagnostics("reached", run_id=run_id)
            else:
                state.set_error(message, fault=True)
                finish_motion_diagnostics("failed", message, run_id)
        except asyncio.CancelledError:
            finish_motion_diagnostics("stopped", "cancelled", run_id)
            raise
        except (SerialClientError, ValueError) as exc:
            state.set_error(str(exc), fault=True)
            finish_motion_diagnostics("failed", str(exc), run_id)
        finally:
            await broadcast_state()
        return

    if state.simulation:
        await execute_simulated_waypoint_trajectory(trajectory, run_id)
        return

    try:
        for index, waypoint in enumerate(waypoints):
            if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
                finish_motion_diagnostics("stopped", state.motion_state.value, run_id)
                break
            waypoint_values = [float(value) for value in waypoint]
            if index == 0 and has_reached_target(state.reported_angles_deg, waypoint_values, tolerance_deg=0.08):
                continue
            update_motion_diagnostics(
                run_id,
                execution_state="executing",
                result="executing",
                current_waypoint_index=index + 1,
                current_waypoint_total=len(waypoints),
                active_target_deg=waypoint_values,
            )
            response = set_targets(
                waypoint_values,
                f"{preview.get('source', 'path')}_waypoint_{index + 1}",
                speed_deg_s=speed,
                accel_deg_s2=accel,
            )
            await broadcast_state()
            if not response["ok"]:
                finish_motion_diagnostics("failed", response.get("error", "waypoint failed"), run_id)
                break

            wait_s = float(segment_durations[index]) if index < len(segment_durations) else 0.05
            is_final_waypoint = index == len(waypoints) - 1
            if state.simulation:
                if not is_final_waypoint:
                    await asyncio.sleep(max(wait_s, 0.0))
                    continue
                deadline = monotonic() + max(1.0, wait_s * 4.0 + 0.5)
                while monotonic() < deadline:
                    if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
                        break
                    if has_reached_target(state.reported_angles_deg, state.target_angles_deg, tolerance_deg=0.08):
                        break
                    await asyncio.sleep(0.03)
            else:
                await asyncio.sleep(max(wait_s, 0.02))
                if serial_client.is_connected:
                    try:
                        refresh_serial_status()
                    except SerialClientError as exc:
                        state.set_error(str(exc), fault=True)
                        finish_motion_diagnostics("failed", str(exc), run_id)
                        break
                if is_final_waypoint:
                    ok, message = await wait_for_hardware_target(
                        waypoint_values,
                        timeout_s=max(1.0, wait_s * 2.0 + 1.0),
                        tolerance_deg=float(calibration_settings(config).get("movement_tolerance_deg", 0.2)),
                    )
                    if not ok:
                        state.set_error(message, fault=True)
                        finish_motion_diagnostics("failed", message, run_id)
                        break
    except asyncio.CancelledError:
        finish_motion_diagnostics("stopped", "cancelled", run_id)
        raise
    finally:
        if state.motion_state == MotionState.MOVING and has_reached_target(
            state.reported_angles_deg, state.target_angles_deg, tolerance_deg=0.08
        ):
            state.motion_state = MotionState.IDLE
        if state.motion_execution_state not in {"failed", "stopped"}:
            finish_motion_diagnostics(
                "reached" if state.motion_state == MotionState.IDLE else state.motion_state.value,
                run_id=run_id,
            )
        await broadcast_state()


async def apply_tool_action(action: str, value: float | None = None, tool: str | None = None) -> dict[str, Any]:
    normalized = action.strip().lower()
    tools = tools_settings(config)
    configured_active = str(tools.get("active", "gripper"))
    active_tool = tool or configured_active
    presets = tools.get("presets", {})
    preset = presets.get(active_tool, {}) if isinstance(presets, dict) else {}
    if active_tool != configured_active:
        state.set_error(f"select {active_tool} before sending tool commands")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not isinstance(preset, dict) or not preset:
        state.set_error(f"tool preset {active_tool} is missing")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    tool_type = str(preset.get("type", "generic"))
    state.active_tool = active_tool
    state.tool_type = tool_type

    if tool_type == "servo_gripper" and normalized not in {"open", "close", "set"}:
        state.set_error(f"{active_tool} does not support TOOL {normalized.upper()}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if tool_type == "electromagnet" and normalized not in {"on", "off"}:
        state.set_error(f"{active_tool} does not support TOOL {normalized.upper()}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    if normalized in {"open", "close"}:
        command = format_tool(normalized)
        state.tool_state = "open" if normalized == "open" else "closed"
        state.tool_value = preset.get("open_value" if normalized == "open" else "closed_value")
    elif normalized in {"on", "off"}:
        command = format_tool(normalized)
        state.tool_state = normalized
        state.tool_value = 1.0 if normalized == "on" else 0.0
    else:
        command = format_tool("set", value)
        state.tool_state = "set"
        state.tool_value = max(0.0, min(1.0, float(value if value is not None else 0.0)))

    state.last_command = command
    if state.simulation:
        log_event("tool", command, simulation=True)
        state.updated_at = time()
        await broadcast_state()
        return {"ok": True, "command": command, "state": state.to_dict()}

    if not state.connected or not serial_client.is_connected:
        state.set_error("tool command requires serial hardware connection")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.hardware_armed:
        state.set_error("tool commands require the Armed toggle")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    tool_errors = active_tool_hardware_errors()
    if tool_errors:
        state.set_error("; ".join(tool_errors))
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    try:
        serial_client.clear_input()
        serial_client.send_line(command)
        response = read_serial_until_any(("OK command=TOOL", "ERR"), timeout_s=1.0)
        if response.startswith("ERR"):
            state.set_error(response)
            await broadcast_state()
            return {"ok": False, "error": response, "state": state.to_dict()}
        log_event("tool", command, response=response)
        refresh_serial_status()
    except SerialClientError as exc:
        state.set_error(str(exc), fault=True)
        await broadcast_state()
        return {"ok": False, "error": str(exc), "state": state.to_dict()}
    await broadcast_state()
    return {"ok": True, "command": command, "state": state.to_dict()}


async def execute_task_sequence(sequence: dict[str, Any], settings: dict[str, Any], branch: str) -> None:
    try:
        steps = sequence.get("steps", [])
        for step_index, step in enumerate(steps, start=1):
            if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
                log_event("task", "task aborted", state=state.motion_state.value)
                break
            label = str(step.get("label", step.get("kind", "step")))
            log_event("task", label)
            if step.get("kind") == "tool":
                result = await apply_tool_action(str(step.get("action", "open")), step.get("value"))
                if not result["ok"]:
                    break
                await asyncio.sleep(0.15)
                continue
            waypoint = step.get("waypoint")
            if not isinstance(waypoint, dict):
                state.set_error(f"task step {label} is missing a waypoint")
                break
            preview_result = build_preview(
                mode="program",
                target=None,
                waypoint_program=[waypoint],
                links=config.links,
                settings=settings,
                branch=branch,
                source="task",
            )
            if not preview_result["ok"]:
                state.set_error(preview_result.get("error", f"task step {label} preview failed"))
                break
            preview = preview_result["preview"]
            preview["task_step_label"] = label
            preview["task_step_index"] = step_index
            preview["task_step_total"] = len(steps)
            trajectory_mode = str(preview.get("trajectory", {}).get("mode", preview.get("mode", ""))).lower()
            if trajectory_mode == "joint":
                await execute_joint_endpoint_move(preview)
            else:
                await execute_waypoint_path(preview)
    except asyncio.CancelledError:
        log_event("task", "task cancelled")
        raise
    finally:
        await broadcast_state()


def reload_runtime_config() -> None:
    global config, limiter, serial_client
    config = load_config()
    limiter.config = config
    serial_client.config = config.serial
    state.joint_names = config.joint_names
    state.fk = forward_kinematics(state.reported_angles_deg, config.links)
    state.active_tool = str(tools_settings(config).get("active", "gripper"))
    state.tool_type = str(tool_settings(config).get("type", "servo_gripper"))
    state.closed_loop_mode = str(encoder_settings(config).get("closed_loop_mode", "off"))
    april_tag_session.configure(camera_settings(config), preserve_frames=True)
    if not state.simulation:
        apply_hardware_evaluation("stale", "runtime config changed; hardware sync required")
    else:
        apply_hardware_evaluation("simulation", "simulation mode")


def log_validation_warnings() -> None:
    for warning in model_validation_warnings(config):
        log_event("config", "model validation warning", warning=warning)
    for name, errors in named_position_errors(config).items():
        log_event("config", "named position invalid", name=name, errors=errors)


def apply_controller_status(status_line: str) -> None:
    state.last_status_line = status_line
    status = parse_status(status_line)
    state.reported_angles_deg = status.joints_deg
    state.homed = status.homed
    state.known_pose = status.known_pose
    state.pose_source = status.pose_source
    state.hardware_armed = status.armed
    if status.hardware_mode != "unknown":
        state.hardware_mode = status.hardware_mode
    if status.enabled_axes:
        state.hardware_enabled_axes = status.enabled_axes
    state.encoder_available = status.encoder_available
    state.encoder_angles_deg = status.encoder_angles_deg or [None] * len(config.joints)
    for index, angle in enumerate(state.encoder_angles_deg[: len(state.reported_angles_deg)]):
        if index < len(state.encoder_available) and state.encoder_available[index] == "1" and angle is not None:
            state.reported_angles_deg[index] = float(angle)
            state.known_pose = True
            if state.pose_source in {"unknown", ""}:
                state.pose_source = "encoder"
    state.closed_loop_mode = status.closed_loop_mode
    if status.tool_type != "unknown":
        state.tool_type = status.tool_type
    state.tool_state = status.tool_state
    if status.tool_value is not None:
        state.tool_value = status.tool_value
    if status.state in {item.value for item in MotionState}:
        state.motion_state = MotionState(status.state)
    state.last_error = "" if status.fault == "OK" else status.fault
    update_encoder_verification()
    state.fk = forward_kinematics(state.reported_angles_deg, config.links)
    record_motion_sample(active_motion_run_id)
    maybe_finish_reached_motion()
    state.updated_at = time()


def align_target_to_reported() -> None:
    state.target_angles_deg = state.reported_angles_deg.copy()
    limiter.current_deg = state.reported_angles_deg.copy()
    limiter.set_target(state.target_angles_deg)


def refresh_serial_status() -> None:
    if not serial_client.is_connected:
        return
    serial_client.clear_input()
    serial_client.send_line(format_status())
    status_line = serial_client.read_until_prefix("STATUS", timeout_s=1.0)
    apply_controller_status(status_line)


@app.on_event("startup")
async def startup() -> None:
    log_validation_warnings()
    asyncio.create_task(simulation_loop())


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html", headers={"Cache-Control": "no-cache"})


@app.get("/api/config")
async def get_config() -> dict[str, Any]:
    return public_config()


@app.get("/api/state")
async def get_state() -> dict[str, Any]:
    return state.to_dict()


@app.get("/api/serial/ports")
async def get_serial_ports() -> dict[str, Any]:
    return {"ok": True, "ports": list_serial_ports(), "last_port": config.serial.port}


@app.get("/api/events")
async def get_events(limit: int = 100) -> dict[str, Any]:
    return {"ok": True, "events": event_log.list(limit)}


@app.post("/api/events/clear")
async def clear_events() -> dict[str, Any]:
    event_log.clear()
    return {"ok": True, "events": []}


@app.get("/api/diagnostics")
async def diagnostics(limit: int = 120) -> dict[str, Any]:
    return {
        "ok": True,
        "events": event_log.list(limit),
        "state": state.to_dict(),
        "hardware": {
            "evaluation": evaluate_hardware_config(),
            "sync_status": state.config_sync_status,
            "sync_message": state.config_sync_message,
        },
        "encoders": encoder_settings(config),
        "kinematics": asdict(config.kinematics),
        "motion": state.motion_diagnostics,
        "validation": {
            "model_warnings": model_validation_warnings(config),
            "named_position_errors": named_position_errors(config),
        },
    }


@app.get("/api/named-positions")
async def get_named_positions() -> dict[str, Any]:
    positions = named_positions(config)
    errors = {
        name: validate_named_position(config, name, position)
        for name, position in positions.items()
    }
    return {"ok": True, "positions": positions, "errors": errors}


@app.post("/api/named-positions")
async def save_named_positions(request: NamedPositionsRequest) -> dict[str, Any]:
    errors = {
        name: messages
        for name, position in request.positions.items()
        if (messages := validate_named_position(config, name, position))
    }
    if errors:
        state.set_error("one or more named positions are invalid")
        await broadcast_state()
        return {"ok": False, "errors": errors, "state": state.to_dict()}
    try:
        save_calibration_updates(ensure_local_config(), {"named_positions": request.positions})
        reload_runtime_config()
        log_validation_warnings()
    except Exception as exc:
        state.set_error(f"could not save named positions: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    log_event("config", "named positions saved", count=len(request.positions))
    await broadcast_state()
    return {"ok": True, "positions": named_positions(config), "config": public_config(), "state": state.to_dict()}


@app.post("/api/tool")
async def tool_command(request: ToolRequest) -> dict[str, Any]:
    return await apply_tool_action(request.action, request.value, request.tool)


@app.get("/api/tools")
async def get_tools() -> dict[str, Any]:
    return {"ok": True, "tools": tools_settings(config), "active": tools_settings(config).get("active", "gripper")}


@app.post("/api/tools")
async def save_tools(request: ToolsRequest) -> dict[str, Any]:
    tools = tools_settings(config)
    tools["active"] = request.active
    if request.presets:
        presets = tools.setdefault("presets", {})
        for name, preset in request.presets.items():
            merged = presets.get(name, {})
            merged.update(preset)
            presets[name] = merged
    errors = validate_tools_payload(tools)
    if errors:
        state.set_error("; ".join(errors))
        await broadcast_state()
        return {"ok": False, "errors": errors, "error": state.last_error, "state": state.to_dict()}
    try:
        calibration = calibration_settings(config)
        calibration["tool_dimensions_validated"] = False
        save_calibration_updates(ensure_local_config(), {"tools": tools, "calibration": calibration})
        reload_runtime_config()
        log_validation_warnings()
    except Exception as exc:
        state.set_error(f"could not save tool settings: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    log_event("config", "tool settings saved", active=request.active)
    await broadcast_state()
    return {"ok": True, "tools": tools_settings(config), "config": public_config(), "state": state.to_dict()}


@app.get("/api/vision/config")
async def get_vision_config() -> dict[str, Any]:
    return {
        "ok": True,
        "camera": camera_settings(config),
        "color_profiles": color_profiles(config),
        "drop_zones": drop_zones(config),
    }


def open_camera(camera: dict[str, Any]) -> Any:
    import cv2

    capture = cv2.VideoCapture(int(camera.get("source_index", 0)))
    resolution = camera.get("resolution") if isinstance(camera.get("resolution"), dict) else {}
    width = int(resolution.get("width", 0) or 0)
    height = int(resolution.get("height", 0) or 0)
    if width > 0:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError("could not open camera")
    return capture


def capture_camera_frame(camera: dict[str, Any]) -> Any:
    capture = open_camera(camera)
    ok, image = capture.read()
    capture.release()
    if not ok:
        raise RuntimeError("could not read camera frame")
    return image


def april_tag_status_payload() -> dict[str, Any]:
    camera = camera_settings(config)
    settings = april_tag_settings(camera)
    saved = settings.get("result") if isinstance(settings.get("result"), dict) else None
    return {
        "camera": camera,
        "settings": settings,
        "session": april_tag_session.summary(),
        "saved_result": saved,
    }


@app.post("/api/vision/settings")
async def save_vision_settings(request: VisionSettingsRequest) -> dict[str, Any]:
    updates = request.__dict__
    try:
        save_calibration_updates(ensure_local_config(), updates)
        reload_runtime_config()
    except Exception as exc:
        state.set_error(f"could not save vision settings: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    log_event("vision", "vision settings saved")
    await broadcast_state()
    return {"ok": True, "config": public_config(), "state": state.to_dict()}


@app.get("/api/vision/apriltag/status")
async def get_apriltag_status() -> dict[str, Any]:
    return {"ok": True, **april_tag_status_payload()}


@app.post("/api/vision/apriltag/reset")
async def reset_apriltag_session() -> dict[str, Any]:
    april_tag_session.configure(camera_settings(config), preserve_frames=False)
    log_event("vision", "AprilTag calibration session reset")
    return {"ok": True, **april_tag_status_payload()}


@app.post("/api/vision/apriltag/capture")
async def capture_apriltag_sample(request: AprilTagCaptureRequest) -> dict[str, Any]:
    camera = camera_settings(config)
    april_tag_session.configure(camera, preserve_frames=True)
    sample_count = max(1, min(int(request.sample_count), 60))
    interval_s = max(0.0, min(float(request.sample_interval_ms) / 1000.0, 1.0))
    latest_image = None
    latest_detections = []
    try:
        if request.image_b64:
            sample_count = 1
            images = [decode_image_b64(request.image_b64)]
        else:
            images = []
            capture = open_camera(camera)
            try:
                for index in range(sample_count):
                    ok, image = capture.read()
                    if not ok:
                        raise RuntimeError("could not read camera frame")
                    images.append(image)
                    if index + 1 < sample_count and interval_s:
                        await asyncio.sleep(interval_s)
            finally:
                capture.release()
        for image in images:
            detections = detect_apriltags(image, april_tag_session.settings)
            if request.accumulate:
                april_tag_session.add(detections)
            latest_image = image
            latest_detections = detections
        result = (
            april_tag_session.solve()
            if request.accumulate
            else estimate_camera_pose(latest_detections, camera, april_tag_session.settings)
        )
        annotated = annotate_apriltag_frame(latest_image, latest_detections, camera, result)
    except Exception as exc:
        state.set_error(f"AprilTag capture failed: {exc}")
        log_event("vision", "AprilTag capture failed", error=str(exc))
        await broadcast_state()
        return {"ok": False, "error": state.last_error, **april_tag_status_payload()}

    configured = configured_tag_ids(april_tag_session.settings)
    detections_payload = [
        detection.to_dict(configured=detection.tag_id in configured)
        for detection in latest_detections
    ]
    log_event(
        "vision",
        "AprilTag sample captured",
        samples=sample_count,
        tags=[item["id"] for item in detections_payload],
        accepted=result.get("accepted", False),
    )
    return {
        "ok": True,
        "image_b64": encode_image_b64(annotated),
        "detections": detections_payload,
        "result": result,
        **april_tag_status_payload(),
    }


@app.post("/api/vision/apriltag/save")
async def save_apriltag_calibration(request: AprilTagSaveRequest) -> dict[str, Any]:
    camera = camera_settings(config)
    april_tag_session.configure(camera, preserve_frames=True)
    result = april_tag_session.solve()
    summary = april_tag_session.summary()
    required_ids = {int(value) for value in april_tag_session.settings.get("required_ids", [])}
    visible_ids = set(summary.get("tag_ids", []))
    missing_ids = sorted(required_ids - visible_ids)
    errors: list[str] = []
    if request.require_all_tags and missing_ids:
        errors.append(f"missing required tags: {missing_ids}")
    if not result.get("minimum_samples_met"):
        errors.append(
            f"need at least {summary.get('minimum_samples', 0)} accumulated frames; "
            f"have {summary.get('frame_count', 0)}"
        )
    if request.require_all_tags and not result.get("required_tag_samples_met"):
        minimum_samples = int(summary.get("minimum_samples", 0))
        counts = result.get("tag_observation_counts", {})
        errors.append(
            f"each required tag needs at least {minimum_samples} observations; "
            + ", ".join(
                f"{tag_id}={counts.get(str(tag_id), 0)}"
                for tag_id in sorted(required_ids)
            )
        )
    if not result.get("accepted"):
        errors.append(result.get("error") or "camera pose did not pass quality checks")
    if errors:
        return {"ok": False, "error": "; ".join(errors), "result": result, **april_tag_status_payload()}

    updated_camera = camera_settings(config)
    calibration = updated_camera.setdefault("calibration", {})
    april_tag = calibration.setdefault("apriltag", {})
    april_tag["result"] = result
    april_tag["saved_at"] = time()
    try:
        save_calibration_updates(ensure_local_config(), {"camera": updated_camera})
        reload_runtime_config()
    except Exception as exc:
        state.set_error(f"could not save AprilTag calibration: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "result": result, **april_tag_status_payload()}
    log_event(
        "vision",
        "AprilTag calibration saved",
        pose_id=result.get("id"),
        tags=result.get("tags_used", []),
        confidence=(result.get("metrics") or {}).get("confidence"),
    )
    await broadcast_state()
    return {"ok": True, "result": result, "config": public_config(), **april_tag_status_payload()}


@app.post("/api/vision/apriltag/verify")
async def verify_apriltag_calibration(request: AprilTagCaptureRequest) -> dict[str, Any]:
    camera = camera_settings(config)
    saved = april_tag_settings(camera).get("result")
    if not isinstance(saved, dict) or not saved.get("accepted"):
        return {"ok": False, "error": "no accepted AprilTag camera pose has been saved"}
    try:
        image = decode_image_b64(request.image_b64) if request.image_b64 else capture_camera_frame(camera)
        detections = detect_apriltags(image, april_tag_settings(camera))
        live = estimate_camera_pose(detections, camera)
        if not live.get("ok"):
            raise RuntimeError(live.get("error", "could not estimate live camera pose"))
        import cv2
        import numpy as np

        saved_position = np.asarray(saved["camera_to_robot"]["position_mm"], dtype=np.float64)
        live_position = np.asarray(live["camera_to_robot"]["position_mm"], dtype=np.float64)
        saved_rotation = np.asarray(saved["camera_to_robot"]["rotation_matrix"], dtype=np.float64)
        live_rotation = np.asarray(live["camera_to_robot"]["rotation_matrix"], dtype=np.float64)
        delta_rotation = saved_rotation.T @ live_rotation
        delta_rvec, _ = cv2.Rodrigues(delta_rotation)
        comparison = {
            "position_delta_mm": float(np.linalg.norm(live_position - saved_position)),
            "orientation_delta_deg": float(np.linalg.norm(delta_rvec) * 180.0 / np.pi),
        }
        annotated = annotate_apriltag_frame(image, detections, camera, live)
    except Exception as exc:
        return {"ok": False, "error": f"AprilTag verification failed: {exc}"}
    return {
        "ok": True,
        "image_b64": encode_image_b64(annotated),
        "live_result": live,
        "saved_result": saved,
        "comparison": comparison,
        "detections": [
            detection.to_dict(configured=detection.tag_id in configured_tag_ids(april_tag_settings(camera)))
            for detection in detections
        ],
    }


@app.post("/api/vision/detect")
async def detect_vision(request: VisionDetectRequest) -> dict[str, Any]:
    profiles = color_profiles(config)
    if request.profile_names:
        profiles = {name: profile for name, profile in profiles.items() if name in set(request.profile_names)}
    try:
        if request.image_b64:
            image = decode_image_b64(request.image_b64)
        else:
            camera = camera_settings(config)
            image = capture_camera_frame(camera)
        detections = detect_configured_colors(image, profiles, camera_settings(config).get("calibration", {}))
    except Exception as exc:
        state.set_error(f"vision detection failed: {exc}")
        log_event("vision", "detection failed", error=str(exc))
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    log_event("vision", "detection complete", detections=detections)
    return {"ok": True, "detections": detections}


@app.get("/api/vision/frame")
async def get_vision_frame() -> dict[str, Any]:
    try:
        camera = camera_settings(config)
        image = capture_camera_frame(camera)
        detections = detect_configured_colors(image, color_profiles(config), camera.get("calibration", {}))
        annotated = annotated_detection_frame(image, detections)
        return {"ok": True, "image_b64": encode_image_b64(annotated), "detections": detections}
    except Exception as exc:
        return {"ok": False, "error": str(exc), "detections": []}


@app.post("/api/hardware-arm")
async def set_hardware_arm(request: ArmRequest) -> dict[str, Any]:
    requested = bool(request.armed)
    if requested and not state.simulation:
        ready, reason = hardware_ready_for_motion()
        if not ready:
            state.hardware_armed = False
            state.set_error(reason)
            await broadcast_state()
            return {"ok": False, "error": reason, "state": state.to_dict()}
    if not state.simulation and serial_client.is_connected:
        try:
            serial_client.clear_input()
            serial_client.send_line(format_arm(requested))
            response = read_serial_until_any(("OK command=ARM", "ERR"), timeout_s=1.0)
            if response.startswith("ERR"):
                state.hardware_armed = False
                state.set_error(response)
                await broadcast_state()
                return {"ok": False, "error": response, "state": state.to_dict()}
            refresh_serial_status()
            if not requested:
                align_target_to_reported()
        except SerialClientError as exc:
            state.hardware_armed = False
            state.set_error(str(exc), fault=True)
            await broadcast_state()
            return {"ok": False, "error": str(exc), "state": state.to_dict()}
    else:
        state.hardware_armed = requested
    if not state.hardware_armed and not state.simulation:
        state.live_motion_enabled = False
        cancel_task(live_task)
    state.last_command = "HARDWARE_ARMED" if state.hardware_armed else "HARDWARE_DISARMED"
    state.updated_at = time()
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/hardware/sync")
async def hardware_sync() -> dict[str, Any]:
    evaluation = apply_hardware_evaluation()
    if evaluation["mode"] == "invalid":
        state.config_sync_status = "invalid"
        state.config_sync_message = "; ".join(evaluation["errors"]) or "hardware config is invalid"
        await broadcast_state()
        return {"ok": False, "status": state.config_sync_status, "evaluation": evaluation, "state": state.to_dict()}
    if state.simulation or not serial_client.is_connected:
        apply_hardware_evaluation("not_connected", "serial hardware is not connected")
        await broadcast_state()
        return {"ok": False, "status": state.config_sync_status, "evaluation": evaluation, "state": state.to_dict()}
    result = sync_hardware_config()
    await broadcast_state()
    return {**result, "state": state.to_dict()}


@app.post("/api/hardware/setpose")
async def hardware_setpose(request: SetPoseRequest) -> dict[str, Any]:
    result = validate_joint_targets(config, request.angles_deg)
    if not result.ok:
        state.set_error(result.reason)
        await broadcast_state()
        return {"ok": False, "error": result.reason, "state": state.to_dict()}
    if state.simulation:
        state.reported_angles_deg = [float(value) for value in request.angles_deg]
        state.target_angles_deg = state.reported_angles_deg.copy()
        limiter.current_deg = state.reported_angles_deg.copy()
        limiter.set_target(state.target_angles_deg)
        state.fk = forward_kinematics(state.reported_angles_deg, config.links)
        state.homed = True
        state.known_pose = True
        state.pose_source = "setpose"
        state.last_command = "SETPOSE_SIM"
        state.updated_at = time()
        log_event("safety", "simulation pose set", angles_deg=state.reported_angles_deg)
        await broadcast_state()
        return {"ok": True, "state": state.to_dict()}
    if state.hardware_armed:
        state.set_error("SETPOSE requires hardware to be disarmed")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    try:
        serial_client.clear_input()
        serial_client.send_line(format_setpose([float(value) for value in request.angles_deg]))
        response = read_serial_until_any(("OK command=SETPOSE", "ERR"), timeout_s=1.0)
        if response.startswith("ERR"):
            state.set_error(response)
            await broadcast_state()
            return {"ok": False, "error": response, "state": state.to_dict()}
        refresh_serial_status()
        align_target_to_reported()
        state.known_pose = True
        state.pose_source = "setpose"
        log_event("safety", "hardware pose set", angles_deg=state.reported_angles_deg)
    except SerialClientError as exc:
        state.set_error(str(exc), fault=True)
        await broadcast_state()
        return {"ok": False, "error": str(exc), "state": state.to_dict()}
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.get("/api/kinematics/dh")
async def get_dh_table() -> dict[str, Any]:
    return {"ok": True, "kinematics": asdict(config.kinematics), "fk": state.fk}


@app.post("/api/kinematics/dh")
async def save_dh_table(request: CalibrationRequest) -> dict[str, Any]:
    if not request.kinematics:
        return {"ok": False, "error": "kinematics payload is required", "state": state.to_dict()}
    try:
        save_calibration_updates(ensure_local_config(), {"kinematics": request.kinematics})
        reload_runtime_config()
        log_validation_warnings()
    except Exception as exc:
        state.set_error(f"could not save DH table: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    await broadcast_state()
    return {"ok": True, "config": public_config(), "state": state.to_dict()}


@app.post("/api/ik/solve")
async def solve_ik(request: IkSolveRequest) -> dict[str, Any]:
    links = links_from_override(request.links_mm)
    result = inverse_kinematics(
        request.target.__dict__,
        links,
        config.joints,
        state.reported_angles_deg,
        request.branch,
    )
    return {"ok": result["ok"], "ik": result}


@app.post("/api/path/preview")
async def preview_path(request: PathPreviewRequest) -> dict[str, Any]:
    links = links_from_override(request.links_mm)
    return build_preview(
        mode=request.mode,
        target=request.target.__dict__ if request.target else None,
        waypoint_program=request.waypoints,
        links=links,
        settings=request_settings(request.settings),
        branch=request.branch,
        source="path",
    )


@app.post("/api/path/execute")
async def execute_path(request: PathExecuteRequest) -> dict[str, Any]:
    global path_task, path_task_source
    preview = path_previews.get(request.preview_id)
    if preview is None:
        state.set_error("path preview not found or expired")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.simulation and not state.hardware_armed:
        state.set_error("hardware moves require the Armed toggle")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.simulation:
        ready, reason = hardware_ready_for_motion()
        if not ready:
            state.set_error(reason)
            await broadcast_state()
            return {"ok": False, "error": reason, "state": state.to_dict()}
    if path_task is not None and not path_task.done():
        state.set_error("a path is already executing")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if live_task is not None and not live_task.done():
        state.set_error("live motion is already executing")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    can_move = validate_can_move(state)
    if not can_move.ok:
        state.set_error(can_move.reason)
        await broadcast_state()
        return {"ok": False, "error": can_move.reason, "state": state.to_dict()}

    state.clear_error()
    state.last_command = f"PATH_EXECUTE {request.preview_id}"
    path_task_source = "path_execute"
    trajectory_mode = str(preview.get("trajectory", {}).get("mode", preview.get("mode", ""))).lower()
    if trajectory_mode == "joint":
        path_task = asyncio.create_task(execute_joint_endpoint_move(preview))
    else:
        path_task = asyncio.create_task(execute_waypoint_path(preview))
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/task/preview")
async def preview_task(request: TaskPreviewRequest) -> dict[str, Any]:
    profiles = color_profiles(config)
    if request.task in {"sorting", "color_sorting"}:
        detections = request.detections or ([request.detection] if request.detection else [])
        if len(detections) > 1:
            sequence = build_batch_sorting_sequence(config, detections, profiles)
        else:
            detection = detections[0] if detections else {}
            sequence = build_sorting_sequence(config, detection, profiles)
    else:
        target = request.object_target or request.detection or {}
        sequence = build_pick_and_place_sequence(config, target, request.drop_zone)
    if not sequence["ok"]:
        state.set_error("; ".join(sequence.get("errors", [])) or "task preview failed")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "sequence": sequence, "state": state.to_dict()}

    preview_result = build_preview(
        mode="program",
        target=None,
        waypoint_program=sequence["waypoints"],
        links=config.links,
        settings=request_settings(request.settings),
        branch=request.branch,
        source="task",
    )
    if not preview_result["ok"]:
        state.set_error(preview_result.get("error", "task motion preview failed"))
        await broadcast_state()
        return {**preview_result, "sequence": sequence, "state": state.to_dict()}
    preview_id = preview_result["preview_id"]
    task_previews[preview_id] = {
        "id": preview_id,
        "created_at": time(),
        "sequence": sequence,
        "settings": request_settings(request.settings),
        "branch": request.branch,
    }
    preview_result["sequence"] = sequence
    log_event("task", f"{sequence['task']} preview", preview_id=preview_id)
    return preview_result


@app.post("/api/task/execute")
async def execute_task(request: TaskExecuteRequest) -> dict[str, Any]:
    global task_task
    preview = task_previews.get(request.preview_id)
    if preview is None:
        state.set_error("task preview not found or expired")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.simulation and not state.hardware_armed:
        state.set_error("task execution requires the Armed toggle")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.simulation:
        ready, reason = hardware_ready_for_motion()
        if not ready:
            state.set_error(reason)
            await broadcast_state()
            return {"ok": False, "error": reason, "state": state.to_dict()}
    can_move = validate_can_move(state)
    if not can_move.ok:
        state.set_error(can_move.reason)
        await broadcast_state()
        return {"ok": False, "error": can_move.reason, "state": state.to_dict()}
    if any(task is not None and not task.done() for task in [path_task, live_task, task_task]):
        state.set_error("motion or task execution is already running")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    task_task = asyncio.create_task(
        execute_task_sequence(preview["sequence"], preview.get("settings", {}), preview.get("branch", "auto"))
    )
    state.last_command = f"TASK_EXECUTE {request.preview_id}"
    log_event("task", "task execution started", preview_id=request.preview_id)
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/live-motion")
async def set_live_motion(request: LiveMotionRequest) -> dict[str, Any]:
    if request.enabled:
        if state.motion_state == MotionState.ESTOP:
            state.set_error("emergency stop is active")
            await broadcast_state()
            return {"ok": False, "error": state.last_error, "state": state.to_dict()}
        if not state.simulation and not state.hardware_armed:
            state.set_error("live hardware moves require the Armed toggle")
            await broadcast_state()
            return {"ok": False, "error": state.last_error, "state": state.to_dict()}
        if not state.simulation:
            ready, reason = hardware_ready_for_motion()
            if not ready:
                state.set_error(reason)
                await broadcast_state()
                return {"ok": False, "error": reason, "state": state.to_dict()}
        can_move = validate_can_move(state)
        if not can_move.ok:
            state.set_error(can_move.reason)
            await broadcast_state()
            return {"ok": False, "error": can_move.reason, "state": state.to_dict()}
        state.live_motion_enabled = True
        state.last_command = "LIVE_MOTION_ON"
    else:
        if cartesian_jog_runtime.get("active") and not state.simulation and serial_client.is_connected:
            try:
                send_jog_stop_and_read_response()
            except SerialClientError as exc:
                state.set_error(str(exc), fault=True)
                await broadcast_state()
                return {"ok": False, "error": str(exc), "state": state.to_dict()}
        state.live_motion_enabled = False
        cancel_task(live_task)
        reset_cartesian_jog_runtime()
        state.last_command = "LIVE_MOTION_OFF"
    state.updated_at = time()
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/live-target")
async def live_target(request: LiveTargetRequest) -> dict[str, Any]:
    global live_task
    if not state.live_motion_enabled:
        state.set_error("live motion is disabled")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if path_task is not None and not path_task.done():
        state.set_error("cannot live jog while a path is executing")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not state.simulation and not state.hardware_armed:
        state.set_error("live hardware moves require the Armed toggle")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    settings = request_settings(request.settings)
    settings.setdefault("planner_type", "s_curve")
    settings.setdefault("waypoint_rate_hz", config.motion.command_rate_limit_hz)

    if request.angles_deg is not None:
        trajectory = build_joint_trajectory(
            state.reported_angles_deg,
            [float(value) for value in request.angles_deg],
            config.joints,
            settings,
        )
        if not trajectory["ok"]:
            state.set_error("; ".join(trajectory.get("errors", [])) or "live target failed")
            await broadcast_state()
            return {"ok": False, "error": state.last_error, "trajectory": trajectory, "state": state.to_dict()}
        preview_id = str(uuid4())
        preview = {
            "id": preview_id,
            "created_at": time(),
            "source": "live",
            "mode": "jog",
            "target": {},
            "settings": settings,
            "ik": None,
            "trajectory": trajectory,
            "completion_feedback": "timed + STATUS estimate for hardware",
        }
        path_previews[preview_id] = preview
        result = {"ok": True, "preview_id": preview_id, "preview": preview}
    else:
        result = build_preview(
            mode=request.mode,
            target=request.target.__dict__ if request.target else None,
            waypoint_program=None,
            links=config.links,
            settings=settings,
            branch=request.branch,
            source="live",
        )
        if not result["ok"]:
            state.set_error(result.get("error", "live target failed"))
            await broadcast_state()
            return {**result, "state": state.to_dict()}
        preview = result["preview"]

    cancel_task(live_task)
    state.clear_error()
    state.last_command = f"LIVE_TARGET {result['preview_id']}"
    trajectory_mode = str(preview.get("trajectory", {}).get("mode", preview.get("mode", ""))).lower()
    if trajectory_mode in {"joint", "jog"}:
        live_task = asyncio.create_task(execute_joint_endpoint_move(preview))
    else:
        live_task = asyncio.create_task(execute_waypoint_path(preview))
    await broadcast_state()
    return {**result, "state": state.to_dict()}


def _cartesian_jog_can_run() -> tuple[bool, str]:
    if state.motion_state == MotionState.ESTOP:
        return False, "emergency stop is active"
    if state.motion_state == MotionState.FAULT:
        return False, "clear the fault before Cartesian jog"
    if path_task is not None and not path_task.done():
        return False, "cannot Cartesian jog while a path is executing"
    if task_task is not None and not task_task.done():
        return False, "cannot Cartesian jog while a task is executing"
    if live_task is not None and not live_task.done():
        return False, "another live motion command is executing"
    if not state.connected and not state.simulation:
        return False, "not connected to hardware and simulation is disabled"
    if not state.simulation:
        if not state.hardware_armed:
            return False, "Cartesian jog requires the Armed toggle"
        if not state.live_motion_enabled:
            return False, "enable Live Real before hardware Cartesian jog"
        ready, reason = hardware_ready_for_motion()
        if not ready:
            return False, reason
    can_move = validate_can_move(state)
    if not can_move.ok:
        return False, can_move.reason
    return True, ""


def _apply_cartesian_jog_target(
    target_deg: list[float],
    joint_velocity_deg_s: list[float],
    accel_deg_s2: float,
) -> str:
    state.target_angles_deg = [float(value) for value in target_deg]
    has_commanded_velocity = any(abs(float(value)) > 0.001 for value in joint_velocity_deg_s)
    had_hardware_velocity = (
        not state.simulation
        and cartesian_jog_runtime.get("active")
        and any(
            abs(float(value)) > 0.001
            for value in (cartesian_jog_runtime.get("joint_velocity_deg_s") or [])
        )
    )
    state.motion_state = MotionState.MOVING if has_commanded_velocity or had_hardware_velocity else MotionState.IDLE
    state.motion_execution_state = "cartesian_jog"
    state.clear_error()
    if state.simulation:
        state.reported_angles_deg = state.target_angles_deg.copy()
        state.fk = forward_kinematics(state.reported_angles_deg, config.links)
        limiter.reset(state.reported_angles_deg)
        state.last_controller_response = "SIMULATION"
        state.last_command = "CARTESIAN_JOG_SIM"
        return state.last_controller_response
    limiter.set_target(state.target_angles_deg)
    if not serial_client.is_connected:
        state.last_controller_response = "SIMULATION"
        state.last_command = "CARTESIAN_JOG_SIM"
        return state.last_controller_response

    command = format_jogv(joint_velocity_deg_s, accel_deg_s2)
    state.last_command = command
    response = send_jogv_and_read_response(command)
    now = monotonic()
    if now - float(cartesian_jog_runtime.get("last_status_poll") or 0.0) >= 0.25:
        try:
            refresh_serial_status()
            cartesian_jog_runtime["last_status_poll"] = now
        except SerialClientError:
            # Keep jog commands responsive; the next normal status poll/stop can surface the connection issue.
            pass
    return response


async def stop_cartesian_jog_internal(reason: str = "cartesian jog stopped") -> dict[str, Any]:
    run_id = cartesian_jog_runtime.get("run_id")
    cartesian_jog_runtime["active"] = False
    cartesian_jog_runtime["seed_deg"] = None
    cartesian_jog_runtime["joint_velocity_deg_s"] = [0.0 for _ in config.joints]
    state.target_angles_deg = state.reported_angles_deg.copy()
    limiter.set_target(state.target_angles_deg)
    if state.motion_state == MotionState.MOVING:
        state.motion_state = MotionState.IDLE if state.simulation else MotionState.STOPPED
    if not state.simulation and serial_client.is_connected:
        try:
            send_jog_stop_and_read_response()
            refresh_serial_status()
        except SerialClientError as exc:
            state.set_error(str(exc), fault=True)
            finish_motion_diagnostics("failed", str(exc), str(run_id) if run_id else None)
            await broadcast_state()
            return {"ok": False, "error": str(exc), "state": state.to_dict()}
    if run_id:
        finish_motion_diagnostics("stopped", reason, str(run_id))
    state.last_command = "CARTESIAN_JOG_STOP"
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/cartesian-jog")
async def cartesian_jog(request: CartesianJogRequest) -> dict[str, Any]:
    ok, reason = _cartesian_jog_can_run()
    if not ok:
        state.set_error(reason)
        await broadcast_state()
        return {"ok": False, "error": reason, "state": state.to_dict()}

    vx = float(request.vx_mm_s)
    vy = float(request.vy_mm_s)
    vz = float(request.vz_mm_s)
    vphi = float(request.vphi_deg_s)
    if not all(value == value and abs(value) < 1e6 for value in [vx, vy, vz, vphi]):
        state.set_error("Cartesian jog command contains a non-finite or unreasonable velocity")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if abs(vx) + abs(vy) + abs(vz) + abs(vphi) <= 1e-6:
        return await stop_cartesian_jog_internal("zero Cartesian jog command")

    settings = request_settings(request.settings)
    dt_s = _cartesian_jog_dt(request.dt_s)
    tcp_limit = max(1.0, float(request.tcp_speed_mm_s or settings.get("tcp_speed_mm_s") or 60.0))
    phi_limit = max(1.0, float(request.phi_speed_deg_s or settings.get("phi_speed_deg_s") or 45.0))
    tcp_speed = (vx * vx + vy * vy + vz * vz) ** 0.5
    clamp_notes: list[str] = []
    if tcp_speed > tcp_limit:
        scale = tcp_limit / max(tcp_speed, 1e-9)
        vx *= scale
        vy *= scale
        vz *= scale
        clamp_notes.append("TCP speed clamped")
    if abs(vphi) > phi_limit:
        vphi = _clamp_scalar(vphi, -phi_limit, phi_limit)
        clamp_notes.append("phi speed clamped")

    now = monotonic()
    seed = _cartesian_jog_seed(now)
    speed_limits, accel_limits = _joint_limits_from_settings(settings)
    max_joint_step = max(speed_limits) * dt_s
    task_delta = _cartesian_jog_task_delta(vx, vy, vz, vphi, dt_s)
    ik_step = differential_ik_step(
        seed,
        task_delta,
        config.links,
        config.joints,
        damping=float(config.kinematics.damping),
        max_joint_step_deg=max_joint_step,
    )
    if not ik_step["ok"]:
        state.set_error(str(ik_step.get("error", "Cartesian jog IK failed")))
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "jog": ik_step, "state": state.to_dict()}

    if ik_step.get("blocked"):
        # A blocked sample is a rejected velocity command, not a new target.
        # Send zero joint velocity so hardware decelerates, keep the accepted
        # seed unchanged, and let the next valid/reverse sample solve cleanly.
        target_deg = seed.copy()
        joint_velocity = [0.0 for _ in config.joints]
        joint_notes: list[str] = []
    else:
        target_deg, joint_velocity, joint_notes = _limit_cartesian_jog_target(
            seed,
            [float(value) for value in ik_step["target_angles_deg"]],
            dt_s,
            speed_limits,
            accel_limits,
        )
    validation = validate_joint_targets(config, target_deg)
    if not validation.ok:
        state.set_error(validation.reason)
        await broadcast_state()
        return {"ok": False, "error": validation.reason, "jog": ik_step, "state": state.to_dict()}

    run_id = cartesian_jog_runtime.get("run_id")
    if not cartesian_jog_runtime.get("active") or not run_id:
        run_id = start_motion_diagnostics(
            source="cartesian_jog",
            mode="cartesian_jog",
            target_deg=target_deg,
            expected_duration_s=0.0,
            waypoint_count=0,
        )
        cartesian_jog_runtime["run_id"] = run_id
    update_motion_diagnostics(
        str(run_id),
        execution_state="cartesian_jog",
        result="executing",
        requested_target_deg=target_deg,
        active_target_deg=target_deg,
        cartesian_command={
            "vx_mm_s": vx,
            "vy_mm_s": vy,
            "vz_mm_s": vz,
            "vphi_deg_s": vphi,
            "dt_s": dt_s,
        },
    )

    try:
        response = _apply_cartesian_jog_target(
            target_deg,
            joint_velocity,
            accel_deg_s2=max(accel_limits),
        )
    except SerialClientError as exc:
        state.set_error(str(exc), fault=True)
        finish_motion_diagnostics("failed", str(exc), str(run_id))
        await broadcast_state()
        return {"ok": False, "error": str(exc), "jog": ik_step, "state": state.to_dict()}

    cartesian_jog_runtime.update(
        {
            "active": True,
            "last_update": now,
            "seed_deg": target_deg,
            "joint_velocity_deg_s": joint_velocity,
            "run_id": run_id,
        }
    )
    record_motion_sample(str(run_id))
    state.updated_at = time()
    jog_result = {
        **ik_step,
        "target_angles_deg": target_deg,
        "joint_velocity_deg_s": joint_velocity,
        "dt_s": dt_s,
        "blocked": bool(ik_step.get("blocked")) or bool(joint_notes),
        "notes": [*ik_step.get("notes", []), *clamp_notes, *joint_notes],
        "failure_code": ik_step.get("failure_code") or ("joint_limit" if joint_notes else None),
        "failure_reason": ik_step.get("failure_reason") or (joint_notes[0] if joint_notes else None),
        "controller_response": response,
    }
    await broadcast_state()
    return {"ok": True, "jog": jog_result, "state": state.to_dict()}


@app.post("/api/cartesian-jog/stop")
async def stop_cartesian_jog() -> dict[str, Any]:
    return await stop_cartesian_jog_internal()


@app.post("/api/config/calibration")
async def save_calibration(request: CalibrationRequest) -> dict[str, Any]:
    config_path = ensure_local_config()
    updates = request.__dict__
    if isinstance(updates.get("tools"), dict):
        tool_errors = validate_tools_payload(updates["tools"])
        if tool_errors:
            state.set_error("; ".join(tool_errors))
            await broadcast_state()
            return {"ok": False, "errors": tool_errors, "error": state.last_error, "state": state.to_dict()}
        calibration = calibration_settings(config)
        calibration["tool_dimensions_validated"] = False
        updates["calibration"] = calibration
    if serial_client.is_connected and not state.simulation:
        ready, reason = config_sync_ready()
        if not ready:
            state.set_error(reason)
            await broadcast_state()
            return {"ok": False, "error": reason, "state": state.to_dict()}
    try:
        with tempfile.TemporaryDirectory() as tmp_dir:
            draft_path = Path(tmp_dir) / "robot.local.yaml"
            shutil.copyfile(config_path, draft_path)
            save_calibration_updates(draft_path, updates)
            load_config(draft_path)

        save_calibration_updates(config_path, updates)
        reload_runtime_config()
        log_validation_warnings()
        if serial_client.is_connected and not state.simulation:
            sync_hardware_config()
    except Exception as exc:
        state.set_error(f"could not save calibration: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    state.last_command = "SAVE_CALIBRATION"
    state.clear_error()
    log_event("config", "calibration saved", path=str(config.source_path))
    await broadcast_state()
    return {"ok": True, "config": public_config(), "state": state.to_dict()}


@app.post("/api/connect")
async def connect(request: ConnectRequest) -> dict[str, Any]:
    cancel_motion_tasks()
    disable_live_motion()
    if request.simulation is True:
        state.simulation = True
        state.connected = True
        state.serial_port = None
        state.hardware_armed = False
        state.known_pose = True
        state.pose_source = "simulation"
        apply_hardware_evaluation("simulation", "simulation mode")
        state.motion_state = MotionState.IDLE
        state.last_command = "SIMULATION CONNECT"
        state.clear_error()
        log_event("connection", "simulation connected")
        await broadcast_state()
        return {"ok": True, "state": state.to_dict()}

    state.simulation = bool(request.simulation) if request.simulation is not None else False
    try:
        serial_client.connect(request.port, request.baud_rate)
        serial_client.send_line(format_hello())
        hello = serial_client.read_until_prefix("HELLO", timeout_s=2.0)
        serial_client.send_line(format_status())
        status_line = serial_client.read_until_prefix("STATUS", timeout_s=2.0)
    except SerialClientError as exc:
        state.connected = False
        state.simulation = False
        state.hardware_armed = False
        state.known_pose = False
        state.pose_source = "unknown"
        state.set_error(str(exc))
        await broadcast_state()
        return {"ok": False, "error": str(exc), "state": state.to_dict()}

    state.connected = True
    state.serial_port = request.port or config.serial.port
    state.hardware_armed = False
    state.motion_state = MotionState.IDLE
    state.last_command = hello
    state.clear_error()
    apply_controller_status(status_line)
    align_target_to_reported()
    sync_hardware_config()
    try:
        save_calibration_updates(
            ensure_local_config(),
            {"serial": {"last_port": state.serial_port, "baud_rate": request.baud_rate or config.serial.baud_rate}},
        )
    except Exception as exc:
        log_event("connection", "could not save last serial port", error=str(exc))
    log_event("connection", "serial connected", port=state.serial_port)
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/disconnect")
async def disconnect() -> dict[str, Any]:
    cancel_motion_tasks()
    if serial_client.is_connected and not state.simulation:
        try:
            serial_client.send_line(format_stop())
        except Exception as exc:
            log_event("connection", "could not send stop before disconnect", error=str(exc))
    serial_client.disconnect()
    state.connected = False
    state.simulation = False
    state.hardware_armed = False
    state.live_motion_enabled = False
    state.known_pose = False
    state.pose_source = "unknown"
    state.config_sync_status = "not_connected"
    state.config_sync_message = "serial hardware is disconnected"
    apply_hardware_evaluation()
    state.serial_port = None
    state.motion_state = MotionState.STOPPED
    state.last_command = "DISCONNECT"
    log_event("connection", "serial disconnected")
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/joint")
async def set_joint_target(request: JointTargetRequest) -> dict[str, Any]:
    if request.index < 0 or request.index >= len(config.joints):
        state.set_error("joint index out of range")
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    targets = state.target_angles_deg.copy()
    targets[request.index] = request.angle_deg
    return await start_joint_target_trajectory(targets, "set_joint_target", request.settings)


@app.post("/api/joints")
async def set_all_joint_targets(request: AllTargetsRequest) -> dict[str, Any]:
    return await start_joint_target_trajectory(request.angles_deg, "set_all_joint_targets", request.settings)


@app.post("/api/home")
async def home() -> dict[str, Any]:
    response = set_targets(config.home_pose, "home")
    state.last_command = format_home() if response["ok"] else state.last_command
    state.homed = response["ok"]
    if response["ok"]:
        state.known_pose = True
        state.pose_source = "home"
        log_event("motion", "home accepted")
    if not state.simulation and serial_client.is_connected and response["ok"]:
        serial_client.send_line(format_home())
        refresh_serial_status()
    await broadcast_state()
    return response


@app.post("/api/stop")
async def stop() -> dict[str, Any]:
    cancel_motion_tasks()
    state.live_motion_enabled = False
    state.target_angles_deg = state.reported_angles_deg.copy()
    limiter.set_target(state.target_angles_deg)
    state.motion_state = MotionState.STOPPED
    state.last_command = format_stop()
    finish_motion_diagnostics("stopped", "STOP")
    if not state.simulation and serial_client.is_connected:
        serial_client.send_line(format_stop())
        refresh_serial_status()
        align_target_to_reported()
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/estop")
async def estop() -> dict[str, Any]:
    cancel_motion_tasks()
    state.target_angles_deg = state.reported_angles_deg.copy()
    limiter.set_target(state.target_angles_deg)
    state.motion_state = MotionState.ESTOP
    state.hardware_armed = False
    state.live_motion_enabled = False
    state.last_command = format_estop()
    finish_motion_diagnostics("stopped", "ESTOP")
    if not state.simulation and serial_client.is_connected:
        serial_client.send_line(format_estop())
        refresh_serial_status()
        align_target_to_reported()
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/clear-estop")
async def clear_estop() -> dict[str, Any]:
    if not state.simulation:
        state.set_error("clearing ESTOP is only allowed in simulation mode in this starter app")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if state.motion_state == MotionState.ESTOP:
        state.motion_state = MotionState.STOPPED
    state.live_motion_enabled = False
    state.clear_error()
    state.last_command = "CLEAR_ESTOP_SIM"
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    websockets.add(websocket)
    await websocket.send_json({"type": "config", "config": public_config()})
    await websocket.send_json({"type": "state", "state": state.to_dict()})
    try:
        while True:
            payload = await websocket.receive_json()
            command = payload.get("command")
            if command == "set_all_joint_targets":
                await start_joint_target_trajectory(
                    [float(value) for value in payload.get("angles_deg", [])],
                    "ws_set_all",
                    payload.get("settings"),
                )
            elif command == "set_joint_target":
                index = int(payload.get("index", -1))
                targets = state.target_angles_deg.copy()
                if 0 <= index < len(targets):
                    targets[index] = float(payload.get("angle_deg", targets[index]))
                    await start_joint_target_trajectory(targets, "ws_set_joint", payload.get("settings"))
                else:
                    state.set_error("joint index out of range")
            elif command == "stop":
                await stop()
            elif command == "estop":
                await estop()
            elif command == "home":
                await home()
            elif command == "clear_estop":
                await clear_estop()
            await broadcast_state()
    except WebSocketDisconnect:
        pass
    finally:
        websockets.discard(websocket)


async def simulation_loop() -> None:
    last = monotonic()
    interval = 1.0 / config.motion.update_rate_hz
    while True:
        now = monotonic()
        dt_s = max(0.0, now - last)
        last = now

        if state.simulation and not simulation_trajectory_active:
            apply_simulation_step(state, limiter, dt_s)
        state.fk = forward_kinematics(state.reported_angles_deg, config.links)
        record_motion_sample(active_motion_run_id)
        maybe_finish_reached_motion()
        state.updated_at = time()
        await broadcast_state()
        await asyncio.sleep(interval)
