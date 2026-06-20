from __future__ import annotations

import asyncio
import hashlib
import json
import subprocess
import shutil
import tempfile
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime, timezone
from math import isfinite
from pathlib import Path
from time import monotonic, time
from typing import Any
from uuid import uuid4

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse
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
from .cartesian_servo import CartesianServo, CartesianServoLimits
from .cartesian_calibration import (
    calibration_settings as kinematics_calibration_settings,
    calibration_summary as kinematics_calibration_summary,
    correct_cartesian_target,
    correct_waypoint_program,
    create_sample as create_kinematics_calibration_sample,
    fit_profile as fit_kinematics_calibration_profile,
    predict_physical_pose,
    workspace_context as kinematics_workspace_context,
)
from .config import LinkConfig, RobotConfig, ensure_local_config, load_config, save_calibration_updates
from .demo_settings import (
    camera_settings,
    calibration_settings,
    color_sorting_task_defaults,
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
from .kinematics import COORDINATE_FRAME, forward_kinematics, inverse_kinematics
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
    format_jog_stop,
    format_jogj,
    format_jogv,
    format_movej,
    format_servoj,
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
from .tasks import (
    build_batch_sorting_sequence,
    build_color_sorting_plan,
    build_pick_and_place_sequence,
    build_sorting_sequence,
    named_position_waypoint,
    normalize_color_sorting_settings,
    TaskSettingsError,
)
from .vision import (
    CameraCapture,
    VisionPipeline,
    apply_planar_transform,
    decode_image_b64,
    encode_image_b64,
    workspace_aruco_settings,
)
from .workspace_calibration import (
    WorkspaceCalibrationSession,
    annotate_workspace_calibration,
    detect_fiducials,
    marker_box_corners,
    marker_centers,
    saved_homography,
    workspace_mapping_errors,
)


APP_DIR = Path(__file__).resolve().parent
STATIC_DIR = APP_DIR / "static"
PROJECT_DIR = APP_DIR.parent


def _git_revision() -> dict[str, Any]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "--short=12", "HEAD"],
            cwd=PROJECT_DIR,
            capture_output=True,
            check=True,
            text=True,
            timeout=2,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain", "--untracked-files=no"],
                cwd=PROJECT_DIR,
                capture_output=True,
                check=True,
                text=True,
                timeout=2,
            ).stdout.strip()
        )
        return {"commit": commit, "dirty": dirty}
    except (OSError, subprocess.SubprocessError):
        return {"commit": None, "dirty": None}


def _origin_main_revision() -> str | None:
    try:
        output = subprocess.run(
            ["git", "ls-remote", "origin", "refs/heads/main"],
            cwd=PROJECT_DIR,
            capture_output=True,
            check=True,
            text=True,
            timeout=4,
        ).stdout.strip()
        return output.split()[0][:12] if output else None
    except (OSError, subprocess.SubprocessError):
        return None


def _frontend_files() -> list[Path]:
    return [
        path
        for path in STATIC_DIR.rglob("*")
        if path.is_file() and path.suffix.lower() in {".js", ".css", ".html"}
    ]


def _backend_files() -> list[Path]:
    return [
        path
        for path in APP_DIR.rglob("*.py")
        if path.is_file()
    ]


def _fingerprint_files(files: list[Path]) -> str:
    digest = hashlib.sha256()
    for path in sorted(set(files), key=lambda candidate: str(candidate).lower()):
        try:
            relative_path = path.relative_to(PROJECT_DIR)
        except ValueError:
            relative_path = path
        digest.update(str(relative_path).replace("\\", "/").encode("utf-8"))
        digest.update(path.read_bytes())
    return digest.hexdigest()[:12]


def frontend_fingerprint() -> str:
    return _fingerprint_files(_frontend_files())


def backend_fingerprint() -> str:
    return _fingerprint_files(_backend_files())


def source_fingerprint() -> str:
    """Compatibility helper for callers that still need one combined source hash."""
    return _fingerprint_files([*_backend_files(), *_frontend_files()])


def config_fingerprint() -> str | None:
    if not config.source_path.exists():
        return None
    return _fingerprint_files([config.source_path])

config: RobotConfig = load_config()
RUNNING_BACKEND_BUILD_ID = backend_fingerprint()
RUNNING_CONFIG_ID = config_fingerprint()
RUNNING_BUILD_STARTED_AT = datetime.now(timezone.utc).isoformat()
RUNNING_GIT = _git_revision()
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
task_selection_events: dict[str, asyncio.Event] = {}
task_selection_choices: dict[str, str] = {}
simulation_vision_queue: list[dict[str, Any]] = []
cartesian_jog_task: asyncio.Task[None] | None = None
event_log = EventLog()
active_motion_run_id: str | None = None
simulation_trajectory_active = False
MAX_TRAJECTORY_UPLOAD_POINTS = 220
TASK_PREVIEW_TTL_S = 600.0
PREVIEW_START_TOLERANCE_DEG = 0.1
CARTESIAN_JOG_STALE_S = 0.35
cartesian_servo = CartesianServo(config.links, config.joints, state.reported_angles_deg)
cartesian_jog_runtime: dict[str, Any] = {
    "active": False,
    "last_input": 0.0,
    "last_status_poll": 0.0,
    "last_broadcast": 0.0,
    "command_velocity": [0.0, 0.0, 0.0, 0.0],
    "joint_velocity_deg_s": [0.0, 0.0, 0.0, 0.0],
    "run_id": None,
    "settings": None,
    "last_result": None,
    "stop_requested": False,
    "stop_reason": None,
}
april_tag_session = AprilTagCalibrationSession(camera_settings(config))
workspace_calibration_session = WorkspaceCalibrationSession(
    workspace_aruco_settings(camera_settings(config))
)
camera_capture = CameraCapture()
vision_pipeline = VisionPipeline()


def robot_model_fingerprint(links: LinkConfig | None = None) -> str:
    active_links = links or config.links
    payload = {
        "links": asdict(active_links),
        "joints": [
            {
                "min_deg": joint.min_deg,
                "max_deg": joint.max_deg,
                "home_deg": joint.home_deg,
                "zero_offset_deg": joint.zero_offset_deg,
                "direction_sign": joint.direction_sign,
            }
            for joint in config.joints
        ],
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()[:12]


def pose_snapshot_fields(*, planning_links: LinkConfig | None = None) -> dict[str, Any]:
    return {
        "start_pose_revision": int(state.pose_revision),
        "start_reported_angles_deg": [float(value) for value in state.reported_angles_deg],
        "start_reported_at": float(state.reported_at),
        "start_pose_source": state.pose_source,
        "config_id": RUNNING_CONFIG_ID,
        "model_fingerprint": robot_model_fingerprint(),
        "planning_model_fingerprint": robot_model_fingerprint(planning_links),
    }


def preview_stale_reason(preview: dict[str, Any]) -> str | None:
    if "start_pose_revision" not in preview or "start_reported_angles_deg" not in preview:
        return "preview is missing an authoritative start pose; preview again"
    if preview.get("config_id") != RUNNING_CONFIG_ID:
        return "robot configuration changed after preview; preview again"
    if preview.get("model_fingerprint") != robot_model_fingerprint():
        return "robot model changed after preview; preview again"

    start_angles = preview.get("start_reported_angles_deg")
    if not isinstance(start_angles, list) or len(start_angles) != len(state.reported_angles_deg):
        return "preview start pose is invalid; preview again"
    deltas = [
        abs(float(current) - float(start))
        for current, start in zip(state.reported_angles_deg, start_angles, strict=True)
    ]
    max_delta = max(deltas, default=0.0)
    if max_delta > PREVIEW_START_TOLERANCE_DEG:
        return (
            "preview start pose is stale: "
            f"planned at revision {preview.get('start_pose_revision')} from "
            f"{[round(float(value), 3) for value in start_angles]}, "
            f"current revision {state.pose_revision} is "
            f"{[round(float(value), 3) for value in state.reported_angles_deg]} "
            f"(max delta {max_delta:.3f} deg); preview again"
        )
    return None


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
    tcp_accel_mm_s2: float | None = None
    phi_accel_deg_s2: float | None = None


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
    apply_calibration: bool = True
    program_revision: int | None = None


class PathExecuteRequest(BaseModel):
    preview_id: str
    program_revision: int | None = None


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
    tasks: dict[str, Any] | None = None
    path_defaults: dict[str, Any] | None = None
    tool: dict[str, Any] | None = None
    tools: dict[str, Any] | None = None
    encoders: dict[str, Any] | None = None
    calibration: dict[str, Any] | None = None
    kinematics_calibration: dict[str, Any] | None = None
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


class VisionProjectRequest(BaseModel):
    detections: list[dict[str, Any]]


class SimulationVisionQueueRequest(BaseModel):
    frames: list[dict[str, Any]]


class WorkspaceCalibrationRequest(BaseModel):
    image_b64: str | None = None


class WorkspaceCalibrationRunRequest(BaseModel):
    max_frames: int = 36
    sample_interval_ms: int = 60


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
    task_settings: dict[str, Any] | None = None
    selected_detection_ids: list[str] | None = None
    branch: str = "auto"


class TaskExecuteRequest(BaseModel):
    preview_id: str


class TaskSelectionRequest(BaseModel):
    run_id: str
    detection_id: str


class KinematicsCalibrationTargetsRequest(BaseModel):
    rows: int = 3
    columns: int = 4
    margin_mm: float = 25.0
    z_mm: float = 45.0
    phi_deg: float = 0.0
    apply_calibration: bool = False


class KinematicsCalibrationSampleRequest(BaseModel):
    intended_target: dict[str, Any]
    command_target: dict[str, Any] | None = None
    measured: dict[str, Any]
    role: str = "fit"
    quality: float = 1.0
    measurement_source: dict[str, Any] | None = None
    joint_source: str = "reported"
    notes: str = ""


class KinematicsCalibrationFitRequest(BaseModel):
    model_type: str = "affine_xy_z_offset"
    profile_key: str | None = None
    enable_after_fit: bool = False


class KinematicsCalibrationEnableRequest(BaseModel):
    enabled: bool
    profile_key: str | None = None


def public_config() -> dict[str, Any]:
    return {
        "app_version": running_version_payload(),
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
        "tasks": {
            "color_sorting": color_sorting_task_defaults(config),
        },
        "path_defaults": default_path_settings(),
        "tool": tool_settings(config),
        "tools": tools_settings(config),
        "encoders": encoder_settings(config),
        "calibration": calibration_settings(config),
        "kinematics_calibration": kinematics_calibration_summary(config),
        "geometry": geometry_settings(config),
        "validation": {
            "model_warnings": model_validation_warnings(config),
            "named_position_errors": named_position_errors(config),
        },
    }


def running_version_payload() -> dict[str, Any]:
    frontend_build_id = frontend_fingerprint()
    disk_backend_build_id = backend_fingerprint()
    disk_config_id = config_fingerprint()
    current_git = _git_revision()
    origin_main_commit = _origin_main_revision()
    checkout_changed = bool(
        RUNNING_GIT["commit"]
        and current_git["commit"]
        and RUNNING_GIT["commit"] != current_git["commit"]
    )
    backend_changed = disk_backend_build_id != RUNNING_BACKEND_BUILD_ID
    config_changed = bool(
        RUNNING_CONFIG_ID
        and disk_config_id
        and RUNNING_CONFIG_ID != disk_config_id
    )
    remote_differs = bool(
        origin_main_commit
        and current_git["commit"]
        and origin_main_commit != current_git["commit"]
    )
    restart_required = backend_changed
    reasons: list[str] = []
    if backend_changed:
        reasons.append("backend Python files changed after the server process started")
    if config_changed:
        reasons.append("configuration file changed after the runtime config was loaded")
    if remote_differs:
        reasons.append("the local checkout is not at origin/main")
    return {
        "frontend_build_id": frontend_build_id,
        "running_backend_build_id": RUNNING_BACKEND_BUILD_ID,
        "disk_backend_build_id": disk_backend_build_id,
        "running_config_id": RUNNING_CONFIG_ID,
        "disk_config_id": disk_config_id,
        "backend_restart_required": restart_required,
        "config_reload_required": config_changed,
        "remote_differs": remote_differs,
        "checkout_changed_since_start": checkout_changed,
        # Compatibility fields make an older browser detect the new frontend
        # build and refresh once after this version contract changes.
        "running_build_id": frontend_build_id,
        "disk_build_id": frontend_build_id,
        "restart_required": restart_required,
        "pull_required": remote_differs,
        "reasons": reasons,
        "started_at": RUNNING_BUILD_STARTED_AT,
        "git_commit": RUNNING_GIT["commit"],
        "git_dirty_at_start": RUNNING_GIT["dirty"],
        "current_git_commit": current_git["commit"],
        "current_git_dirty": current_git["dirty"],
        "origin_main_commit": origin_main_commit,
        "checkout_path": str(PROJECT_DIR),
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
    if not state.known_pose:
        return False, "hardware pose is unknown; use Set Pose while disarmed before arming or moving"
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
    if state.hardware_armed:
        return False, "disarm hardware before syncing controller configuration"
    return True, ""


def _joint_pose_mapping_signature(robot_config: RobotConfig) -> list[dict[str, Any]]:
    signatures: list[dict[str, Any]] = []
    for joint in robot_config.joints:
        hardware = joint.hardware
        conversion: dict[str, Any] = {}
        if joint.actuator == "stepper" and hardware.stepper:
            conversion = {
                "full_steps_per_rev": hardware.stepper.motor_full_steps_per_rev,
                "microsteps": hardware.stepper.microsteps,
                "gear_ratio": hardware.stepper.gear_ratio,
            }
        elif joint.actuator == "servo" and hardware.servo:
            conversion = {
                "pulse_min_us": hardware.servo.pulse_min_us,
                "pulse_max_us": hardware.servo.pulse_max_us,
                "servo_range_deg": hardware.servo.servo_range_deg,
                "neutral_deg": hardware.servo.neutral_deg,
                "gear_ratio": hardware.servo.gear_ratio,
            }
        signatures.append(
            {
                "actuator": joint.actuator,
                "home_deg": joint.home_deg,
                "zero_offset_deg": joint.zero_offset_deg,
                "direction_sign": joint.direction_sign,
                "conversion": conversion,
            }
        )
    return signatures


def _joint_controller_signature(robot_config: RobotConfig) -> list[dict[str, Any]]:
    return [
        {
            "name": joint.name,
            "actuator": joint.actuator,
            "min_deg": joint.min_deg,
            "max_deg": joint.max_deg,
            "home_deg": joint.home_deg,
            "max_speed_deg_s": joint.max_speed_deg_s,
            "max_accel_deg_s2": joint.max_accel_deg_s2,
            "zero_offset_deg": joint.zero_offset_deg,
            "direction_sign": joint.direction_sign,
            "hardware": asdict(joint.hardware),
        }
        for joint in robot_config.joints
    ]


def _joint_io_signature(robot_config: RobotConfig) -> list[dict[str, Any]]:
    signatures: list[dict[str, Any]] = []
    for joint in robot_config.joints:
        hardware = joint.hardware
        io: dict[str, Any] = {"actuator": joint.actuator}
        if joint.actuator == "stepper" and hardware.stepper:
            io.update(
                {
                    "enabled": hardware.stepper.enabled,
                    "step_pin": hardware.stepper.step_pin,
                    "dir_pin": hardware.stepper.dir_pin,
                    "enable_pin": hardware.stepper.enable_pin,
                    "enable_active_low": hardware.stepper.enable_active_low,
                    "driver_model": hardware.stepper.driver_model,
                }
            )
        elif joint.actuator == "servo" and hardware.servo:
            io.update(
                {
                    "enabled": hardware.servo.enabled,
                    "pwm_pin": hardware.servo.pwm_pin,
                    "pwm_frequency_hz": hardware.servo.pwm_frequency_hz,
                }
            )
        signatures.append(io)
    return signatures


def _tool_controller_signature(robot_config: RobotConfig) -> dict[str, Any]:
    settings = tools_settings(robot_config)
    presets = settings.get("presets") if isinstance(settings.get("presets"), dict) else {}
    return {
        "active": settings.get("active"),
        "presets": {
            name: {
                key: deepcopy(value)
                for key, value in preset.items()
                if key != "tcp_offset_mm"
            }
            for name, preset in presets.items()
            if isinstance(preset, dict)
        },
    }


def classify_config_change(previous: RobotConfig, current: RobotConfig) -> dict[str, Any]:
    pose_mapping_changed = _joint_pose_mapping_signature(previous) != _joint_pose_mapping_signature(current)
    joint_controller_changed = _joint_controller_signature(previous) != _joint_controller_signature(current)
    io_changed = _joint_io_signature(previous) != _joint_io_signature(current)
    tool_controller_changed = _tool_controller_signature(previous) != _tool_controller_signature(current)
    planning_model_changed = (
        previous.links != current.links
        or previous.kinematics != current.kinematics
        or [
            (joint.min_deg, joint.max_deg)
            for joint in previous.joints
        ]
        != [
            (joint.min_deg, joint.max_deg)
            for joint in current.joints
        ]
    )
    sync_required = joint_controller_changed or tool_controller_changed

    categories: list[str] = []
    reasons: list[str] = []
    if planning_model_changed:
        categories.append("model")
        reasons.append("geometry, TCP, kinematics, or planning limits changed")
    if pose_mapping_changed:
        categories.append("actuator_mapping")
        reasons.append("actuator zero, sign, gearing, servo mapping, or home reference changed")
    if io_changed or tool_controller_changed:
        categories.append("io")
        reasons.append("controller axis or tool IO configuration changed")
    if joint_controller_changed and not pose_mapping_changed and not io_changed:
        categories.append("controller_limits")
        reasons.append("controller joint limits or motion caps changed")
    if not categories and previous.raw != current.raw:
        categories.append("runtime")
        reasons.append("runtime-only settings changed")

    return {
        "changed": bool(categories),
        "categories": categories,
        "reasons": reasons,
        "sync_required": sync_required,
        "disarm_required": sync_required,
        "pose_invalidated": pose_mapping_changed,
        "pose_revalidation_required": pose_mapping_changed,
        "previews_invalidated": planning_model_changed,
    }


def config_change_ready(change: dict[str, Any]) -> tuple[bool, str]:
    if state.motion_state == MotionState.MOVING:
        return False, "stop motion before applying configuration changes"
    if state.live_motion_enabled:
        return False, "turn off live motion before applying configuration changes"
    if any(task is not None and not task.done() for task in [path_task, live_task, task_task]):
        return False, "wait for active motion or task execution to finish before applying configuration changes"
    if change.get("disarm_required") and state.hardware_armed:
        return False, "disarm hardware before saving controller or actuator configuration"
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


def validated_task_path_settings(settings: PathSettingsRequest | dict[str, Any] | None) -> dict[str, Any]:
    merged = request_settings(settings)
    errors: list[str] = []

    def positive_number(key: str) -> None:
        value = merged.get(key)
        try:
            number = float(value)
        except (TypeError, ValueError):
            errors.append(f"{key} must be a finite positive number")
            return
        if not isfinite(number) or number <= 0.0:
            errors.append(f"{key} must be a finite positive number")
        else:
            merged[key] = number

    for key in ("global_speed_deg_s", "global_accel_deg_s2", "waypoint_rate_hz", "cartesian_step_mm"):
        positive_number(key)

    planner = str(merged.get("planner_type", "s_curve")).strip().lower().replace("-", "_")
    if planner not in {"s_curve", "scurve", "linear", "none", "trapezoid"}:
        errors.append("planner_type must be s_curve, trapezoid, or linear")
    merged["planner_type"] = planner

    for key in ("jerk_percent", "blend_percent"):
        try:
            number = float(merged.get(key))
        except (TypeError, ValueError):
            errors.append(f"{key} must be between 0 and 100")
            continue
        if not isfinite(number) or not 0.0 <= number <= 100.0:
            errors.append(f"{key} must be between 0 and 100")
        else:
            merged[key] = number

    for key in ("per_joint_speed_deg_s", "per_joint_accel_deg_s2"):
        values = merged.get(key)
        if not isinstance(values, list) or len(values) != len(config.joints):
            errors.append(f"{key} must contain exactly {len(config.joints)} positive values")
            continue
        normalized: list[float] = []
        for value in values:
            try:
                number = float(value)
            except (TypeError, ValueError):
                number = float("nan")
            if not isfinite(number) or number <= 0.0:
                errors.append(f"{key} must contain exactly {len(config.joints)} positive values")
                break
            normalized.append(number)
        else:
            merged[key] = normalized

    if errors:
        raise TaskSettingsError("; ".join(dict.fromkeys(errors)))
    return merged


def reset_cartesian_jog_runtime() -> None:
    global cartesian_jog_task
    try:
        current_task = asyncio.current_task()
    except RuntimeError:
        current_task = None
    if (
        cartesian_jog_task is not None
        and cartesian_jog_task is not current_task
        and not cartesian_jog_task.done()
    ):
        cartesian_jog_task.cancel()
    cartesian_jog_task = None
    cartesian_servo.reset(state.reported_angles_deg)
    cartesian_jog_runtime.update(
        {
            "active": False,
            "last_input": 0.0,
            "last_status_poll": 0.0,
            "last_broadcast": 0.0,
            "command_velocity": [0.0, 0.0, 0.0, 0.0],
            "joint_velocity_deg_s": [0.0 for _ in config.joints],
            "run_id": None,
            "settings": None,
            "last_result": None,
            "stop_requested": False,
            "stop_reason": None,
        }
    )


def _clamp_scalar(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


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


def _cartesian_servo_period_s() -> float:
    rate_hz = _clamp_scalar(float(config.motion.update_rate_hz), 20.0, 40.0)
    return 1.0 / rate_hz


def _cartesian_servo_limits(settings: dict[str, Any]) -> CartesianServoLimits:
    speed_limits, accel_limits = _joint_limits_from_settings(settings)
    tcp_accel = float(settings.get("tcp_accel_mm_s2") or 360.0)
    phi_accel = float(settings.get("phi_accel_deg_s2") or 240.0)
    return CartesianServoLimits(
        joint_speed_deg_s=speed_limits,
        joint_accel_deg_s2=accel_limits,
        tcp_accel_mm_s2=max(1.0, tcp_accel),
        phi_accel_deg_s2=max(1.0, phi_accel),
    )


def cancel_task(task: asyncio.Task[None] | None) -> None:
    if task is not None and not task.done():
        task.cancel()


def cancel_motion_tasks() -> None:
    global active_motion_run_id
    cancel_task(path_task)
    cancel_task(live_task)
    cancel_task(task_task)
    reset_cartesian_jog_runtime()
    if state.pending_motion.get("status") in {"queued", "accepted", "executing", "command_sent", "uploading"}:
        state.pending_motion = {
            **state.pending_motion,
            "status": "cancelled",
            "finished_at": time(),
        }
    active_motion_run_id = None


ACTIVE_TASK_STATUSES = {"queued", "running", "capturing", "planning", "executing", "waiting_for_selection", "stopping"}
TERMINAL_TASK_STATUSES = {"completed", "failed", "stopped"}


def update_task_execution(**updates: Any) -> dict[str, Any]:
    payload = dict(state.task_execution or {})
    payload.update(updates)
    payload["updated_at"] = time()
    state.task_execution = payload
    state.updated_at = time()
    return payload


def start_task_execution_state(
    *,
    run_id: str,
    preview_id: str,
    task: str,
    strategy: str,
    total_objects: int,
    settings: dict[str, Any],
) -> None:
    state.task_execution = {
        "run_id": run_id,
        "preview_id": preview_id,
        "task": task,
        "strategy": strategy,
        "status": "queued",
        "phase": "queued",
        "current_object": None,
        "current_step": None,
        "completed_count": 0,
        "remaining_count": max(0, total_objects),
        "total_count": max(0, total_objects),
        "latest_capture": None,
        "ignored_objects": [],
        "candidate_objects": [],
        "tool_feedback": {"available": False, "status": "not_implemented"},
        "holding_uncertain": False,
        "warnings": [],
        "terminal_reason": None,
        "settings": settings,
        "started_at": time(),
        "updated_at": time(),
    }
    state.updated_at = time()


def finish_task_execution(status: str, reason: str, *, holding_uncertain: bool = False) -> None:
    if not state.task_execution:
        return
    if state.task_execution.get("status") in TERMINAL_TASK_STATUSES:
        return
    update_task_execution(
        status=status,
        phase=status,
        terminal_reason=reason,
        holding_uncertain=holding_uncertain,
        finished_at=time(),
    )


def task_active() -> bool:
    return (state.task_execution or {}).get("status") in ACTIVE_TASK_STATUSES


def task_motion_gate_reason() -> str | None:
    if not state.simulation and not state.hardware_armed:
        return "task execution requires the Armed toggle"
    if not state.simulation:
        ready, reason = hardware_ready_for_motion()
        if not ready:
            return reason
        tool_errors = active_tool_hardware_errors()
        if tool_errors:
            return "; ".join(tool_errors)
        if not calibration_settings(config).get("tool_dimensions_validated", False):
            return "task execution requires validated active-tool dimensions"
    can_move = validate_can_move(state)
    if not can_move.ok:
        return can_move.reason
    return None


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
    state.pending_motion = {
        "run_id": run_id,
        "source": source,
        "mode": mode,
        "start_pose_revision": state.pose_revision,
        "start_reported_angles_deg": [float(value) for value in state.reported_angles_deg],
        "target_angles_deg": [float(value) for value in target_deg],
        "status": "queued",
        "started_at": time(),
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
        if state.pending_motion.get("run_id") == diagnostics.get("run_id"):
            state.pending_motion = {
                **state.pending_motion,
                "status": str(updates["execution_state"]),
                "updated_at": time(),
            }


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
    if state.pending_motion.get("run_id") == diagnostics.get("run_id"):
        state.pending_motion = {
            **state.pending_motion,
            "status": result,
            "error": error or "",
            "final_reported_angles_deg": [float(value) for value in state.reported_angles_deg],
            "finished_at": time(),
        }
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


def send_servoj_and_read_response(command: str) -> str:
    serial_client.clear_input()
    serial_client.send_line(command)
    response = read_serial_until_any(("OK command=SERVOJ", "ERR"), timeout_s=0.75)
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
        log_event("controller", "config sync blocked", reason=reason)
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
        if state.config_change.get("pose_revalidation_required") and not state.known_pose:
            state.config_sync_message = (
                "controller configuration synced; current pose remains unknown. "
                "Use Set Pose while disarmed before arming."
            )
        else:
            state.config_sync_message = response
        state.last_command = response
        state.clear_error()
        log_event(
            "controller",
            response,
            pose_revalidation_required=bool(state.config_change.get("pose_revalidation_required")),
            known_pose=state.known_pose,
        )
        return {
            "ok": True,
            "status": state.config_sync_status,
            "evaluation": evaluation,
            "response": response,
            "message": state.config_sync_message,
            "pose_revalidation_required": bool(state.config_change.get("pose_revalidation_required")),
        }

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
    apply_calibration: bool = True,
    program_revision: int | None = None,
) -> dict[str, Any]:
    mode = mode.lower()
    ik_result: dict[str, Any] | None = None
    calibration_compatible = links == config.links
    calibration_requested = bool(apply_calibration and calibration_compatible)
    calibration_metadata: dict[str, Any] | list[dict[str, Any]] | None = None
    command_target: dict[str, Any] = {}

    if waypoint_program:
        prepared_program, corrections = correct_waypoint_program(
            waypoint_program,
            config,
            apply_enabled=calibration_requested,
        )
        if apply_calibration and not calibration_compatible:
            for correction in corrections:
                correction["reason"] = "kinematics_override"
                correction["warnings"] = [
                    *correction.get("warnings", []),
                    "Cartesian calibration is not applied while previewing overridden link geometry",
                ]
        trajectory = build_program_trajectory(
            state.reported_angles_deg,
            prepared_program,
            links,
            config.joints,
            settings,
            branch,
        )
        if not trajectory["ok"]:
            return {
                "ok": False,
                "error": "; ".join(trajectory.get("errors", [])) or "program preview failed",
                "diagnostic_category": "ik_reachability",
                "calibration": corrections,
                "trajectory": trajectory,
            }
        if any(correction.get("applied") for correction in corrections):
            trajectory["requested_cartesian_waypoints"] = [
                correction["requested_target"] for correction in corrections
            ]
            trajectory["physical_cartesian_waypoints"] = [
                predict_physical_pose(forward_kinematics(angles, links), config)
                for angles in trajectory.get("waypoints", [])
            ]
        preview_target = target or {}
        preview_mode = "program"
        calibration_metadata = corrections
    else:
        if target is None:
            return {"ok": False, "error": "path preview requires target or waypoints"}
        requested_target = deepcopy(target)
        command_target, correction = correct_cartesian_target(
            requested_target,
            config,
            apply_enabled=calibration_requested,
        )
        if apply_calibration and not calibration_compatible:
            correction["reason"] = "kinematics_override"
            correction["warnings"] = [
                *correction.get("warnings", []),
                "Cartesian calibration is not applied while previewing overridden link geometry",
            ]
        calibration_metadata = correction
        ik_result = inverse_kinematics(
            command_target,
            links,
            config.joints,
            state.reported_angles_deg,
            branch,
        )
        if not ik_result["ok"] or not ik_result["selected"]:
            return {
                "ok": False,
                "error": "IK target has no valid solution",
                "diagnostic_category": "ik_reachability",
                "requested_target": requested_target,
                "command_target": command_target,
                "calibration": correction,
                "ik": ik_result,
            }

        resolved_command_target = dict(ik_result["target"])
        resolved_target = deepcopy(requested_target)
        resolved_target["x_mm"] = float(requested_target["x_mm"])
        resolved_target["y_mm"] = float(requested_target["y_mm"])
        resolved_target["z_mm"] = float(requested_target["z_mm"])
        resolved_target["phi_deg"] = float(resolved_command_target["phi_deg"])
        if bool(requested_target.get("phi_auto", False)) or requested_target.get("phi_deg") is None:
            resolved_target["phi_auto"] = True
        command_target = resolved_command_target
        if mode == "linear":
            movement_target = dict(resolved_command_target)
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
                "diagnostic_category": "ik_reachability",
                "requested_target": resolved_target,
                "command_target": resolved_command_target,
                "calibration": correction,
                "ik": ik_result,
                "trajectory": trajectory,
            }
        if correction.get("applied"):
            trajectory["requested_cartesian_waypoints"] = [resolved_target]
            trajectory["physical_cartesian_waypoints"] = [
                predict_physical_pose(forward_kinematics(angles, links), config)
                for angles in trajectory.get("waypoints", [])
            ]
        preview_target = resolved_target
        preview_mode = mode

    preview_id = str(uuid4())
    preview = {
        "id": preview_id,
        "created_at": time(),
        "source": source,
        **pose_snapshot_fields(planning_links=links),
        "mode": preview_mode,
        "target": preview_target,
        "command_target": command_target,
        "calibration": calibration_metadata,
        "settings": settings,
        "ik": ik_result,
        "trajectory": trajectory,
        "completion_feedback": "timed + STATUS estimate for hardware",
    }
    if preview_mode == "program":
        preview["program_revision"] = program_revision
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
        **pose_snapshot_fields(),
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
    if len(waypoints) == 1:
        final_target = waypoints[0]
        state.target_angles_deg = final_target.copy()
        state.update_reported_pose(final_target, source="simulation", known_pose=True)
        state.fk = forward_kinematics(final_target, config.links)
        limiter.reset(final_target)
        state.motion_state = MotionState.IDLE
        state.clear_error()
        update_motion_diagnostics(
            run_id,
            execution_state="reached",
            result="reached",
            current_waypoint_index=1,
            current_waypoint_total=1,
            active_target_deg=final_target,
        )
        finish_motion_diagnostics("reached", run_id=run_id)
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
            state.update_reported_pose(current, source="simulation", known_pose=True)
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
                state.update_reported_pose(final_target, source="simulation", known_pose=True)
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

    if len(waypoints) == 1 and has_reached_target(
        state.reported_angles_deg,
        [float(value) for value in waypoints[0]],
        tolerance_deg=0.08,
    ):
        final_target = [float(value) for value in waypoints[0]]
        state.target_angles_deg = final_target.copy()
        state.update_reported_pose(final_target, source="simulation", known_pose=True)
        state.fk = forward_kinematics(final_target, config.links)
        limiter.reset(final_target)
        state.motion_state = MotionState.IDLE
        state.clear_error()
        finish_motion_diagnostics("reached", run_id=run_id)
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


async def execute_task_sequence(
    sequence: dict[str, Any],
    settings: dict[str, Any],
    branch: str,
    *,
    terminal_on_finish: bool = True,
) -> dict[str, Any]:
    failed_reason: str | None = None
    stopped = False
    cancelled = False
    try:
        steps = sequence.get("steps", [])
        for step_index, step in enumerate(steps, start=1):
            if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
                log_event("task", "task aborted", state=state.motion_state.value)
                stopped = True
                break
            label = str(step.get("label", step.get("kind", "step")))
            update_task_execution(
                status="executing",
                phase="executing_sequence",
                current_step={
                    "label": label,
                    "index": step_index,
                    "total": len(steps),
                    "kind": step.get("kind"),
                },
                current_object={
                    "index": step.get("object_index"),
                    "detection_id": step.get("detection_id"),
                    "color": step.get("color"),
                    "drop_zone": step.get("drop_zone"),
                    "grid_slot": step.get("grid_slot"),
                },
            )
            await broadcast_state()
            log_event("task", label)
            if step.get("kind") == "tool":
                result = await apply_tool_action(str(step.get("action", "open")), step.get("value"))
                if not result["ok"]:
                    failed_reason = result.get("error") or "tool action failed"
                    break
                update_task_execution(
                    tool_feedback={
                        "available": False,
                        "status": state.tool_state,
                        "commanded": True,
                    }
                )
                await asyncio.sleep(max(0.0, float(settings.get("tool_action_delay_ms", 150)) / 1000.0))
                continue
            waypoint = step.get("waypoint")
            if not isinstance(waypoint, dict):
                state.set_error(f"task step {label} is missing a waypoint")
                failed_reason = state.last_error
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
                failed_reason = state.last_error
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
            if state.motion_state == MotionState.FAULT or state.motion_execution_state == "failed":
                failed_reason = state.last_error or "task motion step failed"
                break
            if state.motion_state in {MotionState.ESTOP, MotionState.STOPPED}:
                stopped = True
                break

            object_index = step.get("object_index")
            next_object_index = (
                steps[step_index].get("object_index")
                if step_index < len(steps) and isinstance(steps[step_index], dict)
                else None
            )
            if object_index is not None and next_object_index != object_index:
                completed = max(
                    int((state.task_execution or {}).get("completed_count", 0)),
                    int(object_index),
                )
                total = int((state.task_execution or {}).get("total_count", completed))
                update_task_execution(
                    completed_count=completed,
                    remaining_count=max(0, total - completed),
                )
        return {
            "ok": failed_reason is None and not stopped and state.motion_state != MotionState.FAULT,
            "error": failed_reason or (state.motion_state.value if stopped else ""),
        }
    except asyncio.CancelledError:
        cancelled = True
        log_event("task", "task cancelled")
        if terminal_on_finish:
            finish_task_execution("stopped", "task cancelled", holding_uncertain=True)
        raise
    finally:
        if terminal_on_finish and not cancelled:
            if failed_reason:
                finish_task_execution("failed", failed_reason, holding_uncertain=True)
            elif stopped:
                finish_task_execution("stopped", state.motion_state.value, holding_uncertain=True)
            elif state.motion_state == MotionState.FAULT:
                finish_task_execution("failed", state.last_error or "task motion fault", holding_uncertain=True)
            else:
                total = int((state.task_execution or {}).get("total_count", 0))
                update_task_execution(completed_count=total, remaining_count=0)
                finish_task_execution("completed", "sequence complete")
        await broadcast_state()


async def move_task_named_position(name: str, settings: dict[str, Any], branch: str, label: str) -> dict[str, Any]:
    waypoint = named_position_waypoint(config, name)
    if waypoint is None:
        return {"ok": False, "error": f"named position {name} is missing"}
    sequence = {
        "ok": True,
        "task": "task_position",
        "steps": [{"kind": "move", "label": label, "waypoint": waypoint}],
        "waypoints": [waypoint],
    }
    return await execute_task_sequence(sequence, settings, branch, terminal_on_finish=False)


def simulation_vision_status() -> dict[str, Any]:
    return {
        "queued_frames": len(simulation_vision_queue),
        "remaining_frames": len(simulation_vision_queue),
        "simulation": state.simulation,
    }


def simulation_vision_payload(*, consume: bool) -> dict[str, Any]:
    if not state.simulation:
        raise RuntimeError("synthetic vision is available only in simulation")
    if not simulation_vision_queue:
        raise RuntimeError("simulation vision queue is empty")
    frame = deepcopy(simulation_vision_queue[0])
    if consume:
        simulation_vision_queue.pop(0)
    detections = frame.get("detections")
    if not isinstance(detections, list):
        raise RuntimeError("simulation vision frame detections must be a list")
    workspace = frame.get("workspace")
    if not isinstance(workspace, dict):
        workspace = {
            "status": "simulated",
            "message": "synthetic simulation detections",
            "visible_ids": [],
            "missing_ids": [],
        }
    return {
        "ok": True,
        "captured_at": time(),
        "detections": deepcopy(detections),
        "workspace": deepcopy(workspace),
        "provider": str(frame.get("provider") or "simulation_queue"),
        "calibration_source": str(frame.get("calibration_source") or "simulation_queue"),
        "synthetic": True,
        "queue": simulation_vision_status(),
    }


async def closed_loop_capture() -> dict[str, Any]:
    if state.simulation:
        return simulation_vision_payload(consume=True)
    camera = camera_settings(config)
    image = await asyncio.to_thread(capture_camera_frame, camera)
    result = vision_pipeline.process(image, camera, color_profiles(config))
    return {
        "ok": True,
        "captured_at": time(),
        "detections": result["detections"],
        "workspace": result["workspace"],
        "provider": result["provider"],
        "calibration_source": result["calibration_source"],
    }


async def wait_for_manual_task_selection(run_id: str, candidates: list[dict[str, Any]]) -> str:
    event = asyncio.Event()
    task_selection_events[run_id] = event
    update_task_execution(
        status="waiting_for_selection",
        phase="waiting_for_selection",
        candidate_objects=candidates,
        current_step={"label": "manual selection", "kind": "operator"},
    )
    await broadcast_state()
    try:
        await event.wait()
        return task_selection_choices.pop(run_id, "")
    finally:
        task_selection_events.pop(run_id, None)


async def execute_closed_loop_sorting(preview: dict[str, Any]) -> None:
    run_id = str((state.task_execution or {}).get("run_id") or uuid4())
    task_settings = normalize_color_sorting_settings(config, preview.get("task_settings"))
    path_settings = preview.get("settings", {})
    branch = preview.get("branch", "auto")
    completed = 0
    grid_zone_counts: dict[str, int] = {}
    terminal_reason = ""
    try:
        while completed < int(task_settings.get("max_objects", 1)):
            gate_reason = task_motion_gate_reason()
            if gate_reason:
                state.set_error(gate_reason)
                finish_task_execution("failed", gate_reason, holding_uncertain=True)
                await broadcast_state()
                return

            update_task_execution(
                status="running",
                phase="moving_camera_clear",
                completed_count=completed,
                remaining_count=max(0, int(task_settings.get("max_objects", 1)) - completed),
                current_step={"label": "camera clear", "kind": "move"},
            )
            await broadcast_state()
            clear_result = await move_task_named_position(
                str(task_settings.get("camera_clear_position") or task_settings.get("safe_position") or "safe"),
                path_settings,
                branch,
                "camera clear",
            )
            if not clear_result.get("ok"):
                reason = clear_result.get("error") or "camera-clear move failed"
                state.set_error(reason)
                finish_task_execution("failed", reason, holding_uncertain=True)
                await broadcast_state()
                return

            await asyncio.sleep(max(0.0, float(task_settings.get("capture_settle_ms", 0)) / 1000.0))
            update_task_execution(
                status="capturing",
                phase="capturing",
                current_step={"label": "capture", "kind": "vision"},
            )
            await broadcast_state()
            try:
                capture = await closed_loop_capture()
            except Exception as exc:
                reason = f"closed-loop capture failed: {exc}"
                state.set_error(reason)
                finish_task_execution("failed", reason, holding_uncertain=True)
                await broadcast_state()
                return

            update_task_execution(
                status="planning",
                phase="planning",
                latest_capture={
                    "captured_at": capture["captured_at"],
                    "detection_count": len(capture.get("detections", [])),
                    "provider": capture.get("provider"),
                    "calibration_source": capture.get("calibration_source"),
                    "workspace": capture.get("workspace"),
                },
            )
            await broadcast_state()
            plan = build_color_sorting_plan(
                config,
                capture.get("detections", []),
                color_profiles(config),
                task_settings={
                    **task_settings,
                    "execution_strategy": "closed_loop",
                    "_initial_zone_counts": grid_zone_counts,
                },
            )
            metadata = plan.get("task_preview", {})
            if plan.get("selection_required"):
                candidates = metadata.get("candidate_objects", [])
                update_task_execution(
                    ignored_objects=metadata.get("ignored_detections", []),
                    candidate_objects=candidates,
                    warnings=metadata.get("warnings", []),
                )
                if not candidates:
                    terminal_reason = "empty scene"
                    finish_task_execution("completed", terminal_reason)
                    await broadcast_state()
                    return
                selected_id = await wait_for_manual_task_selection(run_id, candidates)
                if not selected_id:
                    terminal_reason = "manual selection cancelled"
                    finish_task_execution("stopped", terminal_reason, holding_uncertain=True)
                    await broadcast_state()
                    return
                plan = build_color_sorting_plan(
                    config,
                    capture.get("detections", []),
                    color_profiles(config),
                    task_settings={
                        **task_settings,
                        "execution_strategy": "closed_loop",
                        "_initial_zone_counts": grid_zone_counts,
                    },
                    selected_detection_ids=[selected_id],
                )
                metadata = plan.get("task_preview", {})

            update_task_execution(
                ignored_objects=metadata.get("ignored_detections", []),
                candidate_objects=metadata.get("candidate_objects", []),
                warnings=metadata.get("warnings", []),
                current_object=metadata.get("next_object"),
            )
            await broadcast_state()

            if not plan.get("ok"):
                errors = plan.get("errors", [])
                if errors and "no calibrated detections match enabled color profiles" in errors[0]:
                    terminal_reason = "empty scene"
                    finish_task_execution("completed", terminal_reason)
                    await broadcast_state()
                    return
                reason = "; ".join(errors) or plan.get("error") or "closed-loop planning failed"
                state.set_error(reason)
                finish_task_execution("failed", reason, holding_uncertain=True)
                await broadcast_state()
                return

            gate_reason = task_motion_gate_reason()
            if gate_reason:
                state.set_error(gate_reason)
                finish_task_execution("failed", gate_reason, holding_uncertain=True)
                await broadcast_state()
                return

            preflight = build_preview(
                mode="program",
                target=None,
                waypoint_program=plan.get("waypoints", []),
                links=config.links,
                settings=path_settings,
                branch=branch,
                source="task",
            )
            if not preflight.get("ok"):
                reason = preflight.get("error", "closed-loop cycle preflight failed")
                state.set_error(reason)
                finish_task_execution("failed", reason, holding_uncertain=True)
                await broadcast_state()
                return

            update_task_execution(
                status="executing",
                phase="executing_object",
                current_object=metadata.get("next_object"),
                remaining_count=max(0, int(task_settings.get("max_objects", 1)) - completed),
            )
            await broadcast_state()
            result = await execute_task_sequence(
                plan,
                {**path_settings, "tool_action_delay_ms": task_settings.get("tool_action_delay_ms", 150)},
                branch,
                terminal_on_finish=False,
            )
            if not result.get("ok"):
                reason = result.get("error") or state.last_error or "closed-loop cycle failed"
                finish_task_execution("failed", reason, holding_uncertain=True)
                await broadcast_state()
                return

            completed += 1
            current_object = metadata.get("next_object") or {}
            slot = current_object.get("grid_slot") if isinstance(current_object, dict) else None
            zone_name = current_object.get("drop_zone") if isinstance(current_object, dict) else None
            if zone_name and slot:
                grid_zone_counts[str(zone_name)] = int(slot.get("index", 0)) + 1
            update_task_execution(
                status="running",
                phase="cycle_complete",
                completed_count=completed,
                remaining_count=max(0, int(task_settings.get("max_objects", 1)) - completed),
                current_step={"label": "cycle complete", "kind": "task"},
            )
            await broadcast_state()
            await asyncio.sleep(max(0.0, float(task_settings.get("tool_settle_ms", 0)) / 1000.0))

        terminal_reason = "max_objects reached"
        finish_task_execution("completed", terminal_reason)
        await broadcast_state()
    except asyncio.CancelledError:
        finish_task_execution("stopped", "task cancelled", holding_uncertain=True)
        await broadcast_state()
        raise
    finally:
        task_selection_events.pop(run_id, None)
        task_selection_choices.pop(run_id, None)


def reload_runtime_config(
    loaded_config: RobotConfig | None = None,
    change: dict[str, Any] | None = None,
) -> dict[str, Any]:
    global config, limiter, serial_client, RUNNING_CONFIG_ID
    previous_config = config
    next_config = loaded_config or load_config()
    config_change = change or classify_config_change(previous_config, next_config)
    config = next_config
    RUNNING_CONFIG_ID = config_fingerprint()
    limiter.config = config
    serial_client.config = config.serial
    cartesian_servo.reconfigure(config.links, config.joints)
    cartesian_servo.reset(state.reported_angles_deg)
    state.joint_names = config.joint_names
    state.fk = forward_kinematics(state.reported_angles_deg, config.links)
    state.active_tool = str(tools_settings(config).get("active", "gripper"))
    state.tool_type = str(tool_settings(config).get("type", "servo_gripper"))
    state.closed_loop_mode = str(encoder_settings(config).get("closed_loop_mode", "off"))
    camera = camera_settings(config)
    april_tag_session.configure(camera, preserve_frames=True)
    workspace_calibration_session.configure(
        workspace_aruco_settings(camera),
        preserve_frames=True,
    )
    vision_pipeline.configure(camera)
    path_previews.clear()
    task_previews.clear()
    simulation_vision_queue.clear()
    state.config_change = {
        **config_change,
        "applied_at": time(),
    }
    if not state.simulation:
        if config_change.get("pose_invalidated"):
            state.homed = False
            state.update_reported_pose(
                state.reported_angles_deg,
                source="unknown",
                known_pose=False,
                force_revision=True,
            )
        if config_change.get("sync_required"):
            reason = "; ".join(config_change.get("reasons") or ["controller configuration changed"])
            message = f"{reason}; disarm, sync controller configuration"
            if config_change.get("pose_revalidation_required"):
                message += ", then use Set Pose to revalidate the current pose"
            apply_hardware_evaluation("stale", message)
        else:
            apply_hardware_evaluation()
    else:
        apply_hardware_evaluation("simulation", "simulation mode")
    log_event(
        "config",
        "runtime configuration reloaded",
        categories=config_change.get("categories", []),
        sync_required=bool(config_change.get("sync_required")),
        pose_invalidated=bool(config_change.get("pose_invalidated")),
        reasons=config_change.get("reasons", []),
    )
    return state.config_change


def log_validation_warnings() -> None:
    for warning in model_validation_warnings(config):
        log_event("config", "model validation warning", warning=warning)
    for name, errors in named_position_errors(config).items():
        log_event("config", "named position invalid", name=name, errors=errors)


def apply_controller_status(status_line: str) -> None:
    state.last_status_line = status_line
    status = parse_status(status_line)
    state.homed = status.homed
    reported_angles = [float(value) for value in status.joints_deg]
    known_pose = status.known_pose
    pose_source = status.pose_source
    state.hardware_armed = status.armed
    if status.hardware_mode != "unknown":
        state.hardware_mode = status.hardware_mode
    if status.enabled_axes:
        state.hardware_enabled_axes = status.enabled_axes
    state.encoder_available = status.encoder_available
    state.encoder_angles_deg = status.encoder_angles_deg or [None] * len(config.joints)
    for index, angle in enumerate(state.encoder_angles_deg[: len(reported_angles)]):
        if index < len(state.encoder_available) and state.encoder_available[index] == "1" and angle is not None:
            reported_angles[index] = float(angle)
            known_pose = True
            if pose_source in {"unknown", ""}:
                pose_source = "encoder"
    state.update_reported_pose(
        reported_angles,
        source=pose_source,
        known_pose=known_pose,
    )
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
async def index() -> HTMLResponse:
    frontend_build_id = frontend_fingerprint()
    html = (STATIC_DIR / "index.html").read_text(encoding="utf-8")
    html = html.replace("__APP_BUILD_ID__", frontend_build_id)
    return HTMLResponse(
        html,
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
        },
    )


@app.get("/api/version")
async def get_version() -> dict[str, Any]:
    return {"ok": True, **running_version_payload()}


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
        "kinematics_calibration": kinematics_calibration_summary(config),
        "motion": state.motion_diagnostics,
        "pose_contract": {
            "pose_revision": state.pose_revision,
            "model_fingerprint": robot_model_fingerprint(),
            "config_id": RUNNING_CONFIG_ID,
            "preview_start_tolerance_deg": PREVIEW_START_TOLERANCE_DEG,
        },
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
        config_path = ensure_local_config()
        updates = {"tools": tools, "calibration": calibration}
        with tempfile.TemporaryDirectory() as tmp_dir:
            draft_path = Path(tmp_dir) / "robot.local.yaml"
            shutil.copyfile(config_path, draft_path)
            save_calibration_updates(draft_path, updates)
            draft_config = load_config(draft_path)
            change = classify_config_change(config, draft_config)
            ready, reason = config_change_ready(change)
            if not ready:
                state.set_error(reason)
                await broadcast_state()
                return {"ok": False, "error": reason, "config_change": change, "state": state.to_dict()}
        save_calibration_updates(config_path, updates)
        reload_runtime_config(load_config(config_path), change)
        log_validation_warnings()
    except Exception as exc:
        state.set_error(f"could not save tool settings: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    log_event("config", "tool settings saved", active=request.active)
    await broadcast_state()
    return {
        "ok": True,
        "tools": tools_settings(config),
        "config": public_config(),
        "config_change": state.config_change,
        "state": state.to_dict(),
    }


@app.get("/api/vision/config")
async def get_vision_config() -> dict[str, Any]:
    return {
        "ok": True,
        "camera": camera_settings(config),
        "color_profiles": color_profiles(config),
        "drop_zones": drop_zones(config),
        "detection_contract": {
            "required": ["id", "label", "confidence", "center_px", "bbox_px", "timestamp"],
            "optional": ["robot", "coordinate_source", "projection_quality", "drop_zone"],
            "providers": ["workspace_color", "configured_hsv", "external_ai"],
        },
    }


def capture_camera_frame(camera: dict[str, Any]) -> Any:
    if not bool(camera.get("enabled", False)):
        raise RuntimeError("camera is disabled in Settings")
    return camera_capture.read(camera)


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


def workspace_detection_payload(
    detection: Any,
    settings: dict[str, Any],
) -> list[dict[str, Any]]:
    centers = marker_centers(detection.corners, detection.ids)
    inner_corners = marker_box_corners(
        detection.corners,
        detection.ids,
        settings.get("tag_box_corner_index", {}),
    )
    robot_centers = settings.get("tag_centers_robot_mm")
    if not isinstance(robot_centers, dict):
        robot_centers = {}
    required = {int(value) for value in settings.get("required_ids", [])}
    payload: list[dict[str, Any]] = []
    for marker_id in sorted(centers):
        center = centers[marker_id]
        corner = inner_corners.get(marker_id)
        robot = robot_centers.get(str(marker_id), robot_centers.get(marker_id))
        payload.append(
            {
                "id": marker_id,
                "configured": marker_id in required,
                "center_px": {"x": float(center[0]), "y": float(center[1])},
                "workspace_corner_px": (
                    {"x": float(corner[0]), "y": float(corner[1])}
                    if corner is not None
                    else None
                ),
                "robot_center_mm": (
                    {"x": float(robot[0]), "y": float(robot[1])}
                    if isinstance(robot, (list, tuple)) and len(robot) >= 2
                    else None
                ),
            }
        )
    return payload


def workspace_status_payload() -> dict[str, Any]:
    camera = camera_settings(config)
    settings = workspace_aruco_settings(camera)
    resolution = settings.get("reference_resolution")
    if not isinstance(resolution, dict):
        resolution = {}
    shape = (
        int(resolution.get("height", 0) or 0),
        int(resolution.get("width", 0) or 0),
        3,
    )
    saved_result: dict[str, Any] | None = None
    if shape[0] > 0 and shape[1] > 0:
        homography, metrics = saved_homography(settings, shape)
        saved_result = {
            "ok": homography is not None,
            "metrics": metrics,
            "reference_resolution": {
                "width": shape[1],
                "height": shape[0],
            },
            "saved_at": settings.get("last_calibrated_at"),
        }
    return {
        "camera": camera,
        "settings": settings,
        "session": workspace_calibration_session.summary(),
        "saved_result": saved_result,
    }


def persist_workspace_calibration_result(result: dict[str, Any]) -> dict[str, Any]:
    updated_camera = camera_settings(config)
    calibration = updated_camera.setdefault("calibration", {})
    workspace = calibration.setdefault("workspace_aruco", {})
    workspace.update(
        {
            "enabled": True,
            "dictionary": result["dictionary"],
            "reference_points_px": result["reference_points_px"],
            "reference_workspace_corners_px": result[
                "reference_workspace_corners_px"
            ],
            "reference_resolution": result["reference_resolution"],
            "last_calibrated_at": time(),
            "last_calibration_metrics": result["metrics"],
        }
    )
    save_calibration_updates(ensure_local_config(), {"camera": updated_camera})
    reload_runtime_config()
    return public_config()


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


@app.get("/api/vision/workspace/status")
async def get_workspace_calibration_status() -> dict[str, Any]:
    workspace_calibration_session.configure(
        workspace_aruco_settings(camera_settings(config)),
        preserve_frames=True,
    )
    return {"ok": True, **workspace_status_payload()}


@app.post("/api/vision/workspace/calibrate")
async def calibrate_workspace(
    request: WorkspaceCalibrationRunRequest,
) -> dict[str, Any]:
    camera = camera_settings(config)
    settings = workspace_aruco_settings(camera)
    workspace_calibration_session.configure(settings, preserve_frames=False)
    max_frames = max(
        workspace_calibration_session.minimum_samples,
        min(int(request.max_frames), 120),
    )
    interval_s = max(
        0.0,
        min(float(request.sample_interval_ms) / 1000.0, 0.5),
    )
    latest_image = None
    latest_detection = None
    try:
        if not bool(camera.get("enabled", False)):
            raise RuntimeError("camera is disabled in Settings")
        for frame_index in range(max_frames):
            latest_image = await asyncio.to_thread(capture_camera_frame, camera)
            latest_detection = await asyncio.to_thread(
                detect_fiducials,
                latest_image,
                settings,
            )
            workspace_calibration_session.add(
                latest_detection,
                latest_image.shape,
            )
            if workspace_calibration_session.summary().get("ready"):
                break
            if frame_index + 1 < max_frames and interval_s > 0:
                await asyncio.sleep(interval_s)

        result = workspace_calibration_session.solve(require_minimum_samples=True)
        annotated = annotate_workspace_calibration(
            latest_image,
            latest_detection,
            settings,
            result,
        )
        if not result.get("ok"):
            return {
                "ok": False,
                "error": result.get(
                    "error",
                    "workspace calibration did not collect enough stable frames",
                ),
                "image_b64": encode_image_b64(annotated),
                "detections": workspace_detection_payload(
                    latest_detection,
                    settings,
                ),
                "result": result,
                **workspace_status_payload(),
            }
        updated_config = persist_workspace_calibration_result(result)
    except Exception as exc:
        error = f"workspace calibration failed: {exc}"
        state.set_error(error)
        log_event("vision", "workspace calibration failed", error=str(exc))
        await broadcast_state()
        return {"ok": False, "error": error, **workspace_status_payload()}

    log_event(
        "vision",
        "workspace calibrated and saved",
        frames=result.get("frame_count"),
        dictionary=result.get("dictionary"),
        rmse_mm=result.get("metrics", {}).get("rmse_mm"),
    )
    await broadcast_state()
    return {
        "ok": True,
        "calibrated": True,
        "image_b64": encode_image_b64(annotated),
        "detections": workspace_detection_payload(latest_detection, settings),
        "result": result,
        "config": updated_config,
        **workspace_status_payload(),
    }

@app.post("/api/vision/workspace/verify")
async def verify_workspace_calibration(
    request: WorkspaceCalibrationRequest,
) -> dict[str, Any]:
    camera = camera_settings(config)
    settings = workspace_aruco_settings(camera)
    try:
        image = (
            decode_image_b64(request.image_b64)
            if request.image_b64
            else await asyncio.to_thread(capture_camera_frame, camera)
        )
        detection = detect_fiducials(image, settings)
        homography, saved_metrics = saved_homography(settings, image.shape)
        if homography is None:
            raise RuntimeError(
                saved_metrics.get("error", "no planar workspace calibration is saved")
            )
        comparison = workspace_mapping_errors(homography, detection, settings)
        if not comparison.get("point_count"):
            raise RuntimeError(comparison.get("error", "no configured workspace tags detected"))
        annotated = annotate_workspace_calibration(
            image,
            detection,
            settings,
            comparison,
        )
    except Exception as exc:
        return {"ok": False, "error": f"workspace verification failed: {exc}"}
    return {
        "ok": bool(comparison.get("ok")),
        "error": (
            None
            if comparison.get("ok")
            else (
                f"missing required tags: {comparison.get('missing_ids', [])}"
                if comparison.get("missing_ids")
                else (
                    f"verification exceeds limits: {comparison.get('rmse_mm', 0):.2f} mm RMSE, "
                    f"{comparison.get('max_error_mm', 0):.2f} mm max"
                )
            )
        ),
        "image_b64": encode_image_b64(annotated),
        "comparison": comparison,
        "detections": workspace_detection_payload(detection, settings),
        **workspace_status_payload(),
    }


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
            if not bool(camera.get("enabled", False)):
                raise RuntimeError("camera is disabled in Settings")
            images = await asyncio.to_thread(
                camera_capture.read_many,
                camera,
                sample_count,
                interval_s,
            )
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
    planar = result.get("planar") if isinstance(result.get("planar"), dict) else {}
    planar_ok = bool(planar.get("ok"))
    pose_accepted = bool(result.get("accepted"))
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
    if not pose_accepted and not planar_ok:
        errors.append(result.get("error") or "workspace calibration did not pass quality checks")
    if errors:
        return {"ok": False, "error": "; ".join(errors), "result": result, **april_tag_status_payload()}

    updated_camera = camera_settings(config)
    calibration = updated_camera.setdefault("calibration", {})
    april_tag = calibration.setdefault("apriltag", {})
    if planar_ok and not pose_accepted:
        result = {
            **result,
            "saved_projection": "planar_homography",
            "save_note": "Saved planar workspace calibration for robot X/Y coordinates.",
        }
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
        planar_only=planar_ok and not pose_accepted,
    )
    await broadcast_state()
    return {"ok": True, "result": result, "config": public_config(), **april_tag_status_payload()}


@app.post("/api/vision/apriltag/verify")
async def verify_apriltag_calibration(request: AprilTagCaptureRequest) -> dict[str, Any]:
    camera = camera_settings(config)
    saved = april_tag_settings(camera).get("result")
    saved_planar = saved.get("planar") if isinstance(saved, dict) and isinstance(saved.get("planar"), dict) else {}
    saved_has_planar = bool(saved_planar.get("ok"))
    if not isinstance(saved, dict) or not (saved.get("accepted") or saved_has_planar):
        return {"ok": False, "error": "no planar workspace calibration has been saved"}
    try:
        image = (
            decode_image_b64(request.image_b64)
            if request.image_b64
            else await asyncio.to_thread(capture_camera_frame, camera)
        )
        detections = detect_apriltags(image, april_tag_settings(camera))
        live = estimate_camera_pose(detections, camera)
        if saved.get("accepted"):
            if not live.get("ok"):
                raise RuntimeError(live.get("error", "could not verify live workspace calibration"))
            import cv2
            import numpy as np

            saved_position = np.asarray(saved["camera_to_robot"]["position_mm"], dtype=np.float64)
            live_position = np.asarray(live["camera_to_robot"]["position_mm"], dtype=np.float64)
            saved_rotation = np.asarray(saved["camera_to_robot"]["rotation_matrix"], dtype=np.float64)
            live_rotation = np.asarray(live["camera_to_robot"]["rotation_matrix"], dtype=np.float64)
            delta_rotation = saved_rotation.T @ live_rotation
            delta_rvec, _ = cv2.Rodrigues(delta_rotation)
            comparison = {
                "mode": "camera_pose",
                "position_delta_mm": float(np.linalg.norm(live_position - saved_position)),
                "orientation_delta_deg": float(np.linalg.norm(delta_rvec) * 180.0 / np.pi),
            }
        else:
            live_planar = live.get("planar") if isinstance(live.get("planar"), dict) else {}
            if not live_planar.get("ok"):
                raise RuntimeError(live.get("error") or live_planar.get("error") or "could not estimate planar homography")
            comparison = {
                "mode": "planar_homography",
                "planar_rmse_mm": float(live_planar.get("rmse_mm", 0.0)),
                "planar_max_error_mm": float(live_planar.get("max_error_mm", 0.0)),
                "tags_used": live_planar.get("tags_used", []),
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
    try:
        if request.image_b64:
            image = decode_image_b64(request.image_b64)
        else:
            camera = camera_settings(config)
            image = await asyncio.to_thread(capture_camera_frame, camera)
        result = vision_pipeline.process(
            image,
            camera_settings(config),
            profiles,
            request.profile_names,
        )
    except Exception as exc:
        state.set_error(f"vision detection failed: {exc}")
        log_event("vision", "detection failed", error=str(exc))
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    log_event(
        "vision",
        "detection complete",
        count=len(result["detections"]),
        provider=result["provider"],
        calibration_source=result["calibration_source"],
    )
    return {
        "ok": True,
        "detections": result["detections"],
        "workspace": result["workspace"],
        "provider": result["provider"],
        "calibration_source": result["calibration_source"],
    }


@app.get("/api/vision/frame")
async def get_vision_frame() -> dict[str, Any]:
    try:
        if state.simulation:
            return simulation_vision_payload(consume=False)
        camera = camera_settings(config)
        image = await asyncio.to_thread(capture_camera_frame, camera)
        result = vision_pipeline.process(
            image,
            camera,
            color_profiles(config),
        )
        return {
            "ok": True,
            "raw_image_b64": encode_image_b64(image),
            "image_b64": encode_image_b64(result["annotated"]),
            "detections": result["detections"],
            "workspace": result["workspace"],
            "provider": result["provider"],
            "calibration_source": result["calibration_source"],
        }
    except Exception as exc:
        return {"ok": False, "error": str(exc), "detections": []}


@app.post("/api/simulation/vision/queue")
async def set_simulation_vision_queue(request: SimulationVisionQueueRequest) -> dict[str, Any]:
    if not state.simulation:
        return {"ok": False, "error": "synthetic vision is available only in simulation"}
    frames: list[dict[str, Any]] = []
    for index, frame in enumerate(request.frames):
        if not isinstance(frame, dict):
            return {"ok": False, "error": f"frames[{index}] must be an object"}
        detections = frame.get("detections")
        if not isinstance(detections, list):
            return {"ok": False, "error": f"frames[{index}].detections must be a list"}
        frames.append(deepcopy(frame))
    simulation_vision_queue.clear()
    simulation_vision_queue.extend(frames)
    return {"ok": True, "queue": simulation_vision_status()}


@app.get("/api/simulation/vision/queue")
async def get_simulation_vision_queue() -> dict[str, Any]:
    if not state.simulation:
        return {"ok": False, "error": "synthetic vision is available only in simulation"}
    return {"ok": True, "queue": simulation_vision_status()}


@app.delete("/api/simulation/vision/queue")
async def clear_simulation_vision_queue() -> dict[str, Any]:
    if not state.simulation:
        return {"ok": False, "error": "synthetic vision is available only in simulation"}
    simulation_vision_queue.clear()
    return {"ok": True, "queue": simulation_vision_status()}


@app.get("/api/vision/workspace/live")
async def get_live_workspace_projection() -> dict[str, Any]:
    try:
        camera = camera_settings(config)
        image = await asyncio.to_thread(capture_camera_frame, camera)
        result = vision_pipeline.project_workspace_frame(image, camera)
        return {
            **result,
            "captured_at": time(),
        }
    except Exception as exc:
        return {
            "ok": False,
            "error": str(exc),
            "workspace_projection": None,
        }


@app.post("/api/vision/project")
async def project_external_vision(request: VisionProjectRequest) -> dict[str, Any]:
    """Projection adapter for future YOLO/AI inference output."""
    try:
        detections = vision_pipeline.project_external_detections(
            request.detections,
            camera_settings(config),
        )
    except Exception as exc:
        return {"ok": False, "error": f"could not project external detections: {exc}"}
    return {
        "ok": True,
        "provider": "external_ai",
        "detections": detections,
    }


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
        state.update_reported_pose(
            [float(value) for value in request.angles_deg],
            source="setpose",
            known_pose=True,
            force_revision=True,
        )
        state.target_angles_deg = state.reported_angles_deg.copy()
        limiter.current_deg = state.reported_angles_deg.copy()
        limiter.set_target(state.target_angles_deg)
        state.fk = forward_kinematics(state.reported_angles_deg, config.links)
        state.homed = False
        if state.config_change.get("pose_revalidation_required"):
            state.config_change = {
                **state.config_change,
                "pose_revalidation_required": False,
                "pose_revalidated_at": time(),
            }
        state.last_command = "SETPOSE_SIM"
        log_event("safety", "simulation pose set", angles_deg=state.reported_angles_deg)
        await broadcast_state()
        return {"ok": True, "state": state.to_dict()}
    if state.hardware_armed:
        state.set_error("SETPOSE requires hardware to be disarmed")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    pose_revision_before = state.pose_revision
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
        state.update_reported_pose(
            state.reported_angles_deg,
            source="setpose",
            known_pose=True,
            force_revision=state.pose_revision == pose_revision_before,
        )
        state.homed = False
        if state.config_change.get("pose_revalidation_required"):
            state.config_change = {
                **state.config_change,
                "pose_revalidation_required": False,
                "pose_revalidated_at": time(),
            }
            state.config_sync_message = "controller configuration synced; pose revalidated by operator Set Pose"
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


def _persist_kinematics_calibration(settings: dict[str, Any]) -> None:
    config_path = ensure_local_config()
    with tempfile.TemporaryDirectory() as tmp_dir:
        draft_path = Path(tmp_dir) / "robot.local.yaml"
        shutil.copyfile(config_path, draft_path)
        save_calibration_updates(draft_path, {"kinematics_calibration": settings})
        load_config(draft_path)
    save_calibration_updates(config_path, {"kinematics_calibration": settings})
    reload_runtime_config()


def _workspace_polygon() -> list[list[float]]:
    camera = camera_settings(config)
    calibration = camera.get("calibration") if isinstance(camera, dict) else None
    workspace = calibration.get("workspace_aruco") if isinstance(calibration, dict) else None
    polygon = workspace.get("workspace_polygon_robot_mm") if isinstance(workspace, dict) else None
    if not isinstance(polygon, list) or len(polygon) < 3:
        raise ValueError("workspace polygon is not configured")
    normalized: list[list[float]] = []
    for point in polygon:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            raise ValueError("workspace polygon contains an invalid point")
        x = float(point[0])
        y = float(point[1])
        if not isfinite(x) or not isfinite(y):
            raise ValueError("workspace polygon contains a non-finite point")
        normalized.append([x, y])
    return normalized


def _point_inside_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


@app.get("/api/kinematics-calibration")
async def get_kinematics_calibration() -> dict[str, Any]:
    return {"ok": True, **kinematics_calibration_summary(config)}


@app.post("/api/kinematics-calibration/targets")
async def generate_kinematics_calibration_targets(
    request: KinematicsCalibrationTargetsRequest,
) -> dict[str, Any]:
    if not 1 <= request.rows <= 10 or not 1 <= request.columns <= 10:
        return {"ok": False, "error": "rows and columns must be between 1 and 10"}
    if not all(
        isfinite(float(value))
        for value in [request.margin_mm, request.z_mm, request.phi_deg]
    ):
        return {"ok": False, "error": "target generation values must be finite"}
    try:
        polygon = _workspace_polygon()
        min_x = min(point[0] for point in polygon) + max(0.0, float(request.margin_mm))
        max_x = max(point[0] for point in polygon) - max(0.0, float(request.margin_mm))
        min_y = min(point[1] for point in polygon) + max(0.0, float(request.margin_mm))
        max_y = max(point[1] for point in polygon) - max(0.0, float(request.margin_mm))
        if min_x > max_x or min_y > max_y:
            raise ValueError("margin leaves no usable workspace area")
        points: list[dict[str, Any]] = []
        for row in range(request.rows):
            y = (min_y + max_y) * 0.5 if request.rows == 1 else min_y + (max_y - min_y) * row / (request.rows - 1)
            columns = range(request.columns) if row % 2 == 0 else reversed(range(request.columns))
            for column in columns:
                x = (
                    (min_x + max_x) * 0.5
                    if request.columns == 1
                    else min_x + (max_x - min_x) * column / (request.columns - 1)
                )
                if not _point_inside_polygon(x, y, polygon):
                    continue
                intended = {
                    "x_mm": float(x),
                    "y_mm": float(y),
                    "z_mm": float(request.z_mm),
                    "phi_deg": float(request.phi_deg),
                }
                command, correction = correct_cartesian_target(
                    intended,
                    config,
                    apply_enabled=request.apply_calibration,
                )
                ik = inverse_kinematics(
                    command,
                    config.links,
                    config.joints,
                    state.reported_angles_deg,
                )
                points.append(
                    {
                        "index": len(points) + 1,
                        "intended_target": intended,
                        "command_target": command,
                        "calibration": correction,
                        "reachable": bool(ik.get("ok") and ik.get("selected")),
                        "diagnostic_category": "reachable" if ik.get("ok") and ik.get("selected") else "ik_reachability",
                        "ik_notes": ik.get("notes", []),
                    }
                )
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    summary = kinematics_calibration_summary(config)
    return {
        "ok": True,
        "points": points,
        "workspace": kinematics_workspace_context(config),
        "fit_quality": summary.get("fit_quality"),
        "validation_quality": summary.get("validation_quality"),
        "reachability": {
            "reachable_count": sum(1 for point in points if point["reachable"]),
            "unreachable_count": sum(1 for point in points if not point["reachable"]),
        },
    }


@app.post("/api/kinematics-calibration/samples")
async def save_kinematics_calibration_sample(
    request: KinematicsCalibrationSampleRequest,
) -> dict[str, Any]:
    if state.motion_state == MotionState.MOVING:
        return {"ok": False, "error": "wait for motion to stop before saving a calibration sample"}
    if not state.simulation and (not state.connected or not state.known_pose):
        return {"ok": False, "error": "hardware calibration samples require a connected robot with a known pose"}
    try:
        current_fk = forward_kinematics(state.reported_angles_deg, config.links)
        sample = create_kinematics_calibration_sample(
            request.__dict__,
            config,
            state.reported_angles_deg,
            current_fk,
        )
        settings = kinematics_calibration_settings(config)
        profile_key = sample["tool"]
        profiles = settings.setdefault("profiles", {})
        profile = deepcopy(profiles.get(profile_key) or {})
        samples = profile.get("samples")
        if not isinstance(samples, list):
            samples = []
        samples.append(sample)
        profile.update(
            {
                "tool": profile_key,
                "enabled": bool(profile.get("enabled", False)),
                "model_type": str(profile.get("model_type") or settings.get("default_model") or "affine_xy_z_offset"),
                "workspace": kinematics_workspace_context(config),
                "samples": samples,
            }
        )
        profiles[profile_key] = profile
        settings["active_profile"] = profile_key
        if sample["role"] == "fit":
            profile.pop("result", None)
            profile["enabled"] = False
            settings["enabled"] = False
        elif isinstance(profile.get("result"), dict):
            settings, _ = fit_kinematics_calibration_profile(
                settings,
                config,
                profile_key=profile_key,
                model_type=str(profile.get("model_type") or settings.get("default_model")),
            )
        _persist_kinematics_calibration(settings)
    except Exception as exc:
        return {"ok": False, "error": str(exc), "diagnostic_category": "invalid_sample"}
    log_event("calibration", "TCP sample saved", sample_id=sample["id"], role=sample["role"])
    return {
        "ok": True,
        "sample": sample,
        "config": public_config(),
        **kinematics_calibration_summary(config),
    }


@app.delete("/api/kinematics-calibration/samples/{sample_id}")
async def delete_kinematics_calibration_sample(sample_id: str) -> dict[str, Any]:
    settings = kinematics_calibration_settings(config)
    profile_key = str(kinematics_calibration_summary(config).get("active_profile_key") or "")
    profiles = settings.get("profiles")
    profile = profiles.get(profile_key) if isinstance(profiles, dict) else None
    if not isinstance(profile, dict):
        return {"ok": False, "error": "active calibration profile not found"}
    samples = profile.get("samples")
    if not isinstance(samples, list):
        return {"ok": False, "error": "calibration profile has no samples"}
    retained = [sample for sample in samples if str(sample.get("id")) != sample_id]
    if len(retained) == len(samples):
        return {"ok": False, "error": "calibration sample not found"}
    profile["samples"] = retained
    profile.pop("result", None)
    profile["enabled"] = False
    settings["enabled"] = False
    _persist_kinematics_calibration(settings)
    log_event("calibration", "TCP sample deleted", sample_id=sample_id)
    return {"ok": True, "config": public_config(), **kinematics_calibration_summary(config)}


@app.post("/api/kinematics-calibration/fit")
async def fit_kinematics_calibration(
    request: KinematicsCalibrationFitRequest,
) -> dict[str, Any]:
    if state.motion_state == MotionState.MOVING:
        return {"ok": False, "error": "stop motion before fitting calibration"}
    try:
        settings, result = fit_kinematics_calibration_profile(
            kinematics_calibration_settings(config),
            config,
            profile_key=request.profile_key,
            model_type=request.model_type,
        )
        profile_key = str(settings.get("active_profile"))
        profile = settings["profiles"][profile_key]
        profile["enabled"] = bool(request.enable_after_fit)
        settings["enabled"] = bool(request.enable_after_fit)
        _persist_kinematics_calibration(settings)
    except Exception as exc:
        return {"ok": False, "error": str(exc), "diagnostic_category": "fit_quality"}
    log_event(
        "calibration",
        "TCP calibration fitted",
        model_type=request.model_type,
        fit_status=result.get("fit", {}).get("status"),
    )
    return {
        "ok": True,
        "result": result,
        "config": public_config(),
        **kinematics_calibration_summary(config),
    }


@app.post("/api/kinematics-calibration/enable")
async def enable_kinematics_calibration(
    request: KinematicsCalibrationEnableRequest,
) -> dict[str, Any]:
    if state.motion_state == MotionState.MOVING:
        return {"ok": False, "error": "stop motion before changing calibration state"}
    settings = kinematics_calibration_settings(config)
    profile_key = str(
        request.profile_key
        or kinematics_calibration_summary(config).get("active_profile_key")
        or ""
    )
    profiles = settings.get("profiles")
    profile = profiles.get(profile_key) if isinstance(profiles, dict) else None
    if request.enabled and (not isinstance(profile, dict) or not isinstance(profile.get("result"), dict)):
        return {"ok": False, "error": "fit and save a calibration result before enabling it"}
    if isinstance(profile, dict):
        profile["enabled"] = bool(request.enabled)
    settings["active_profile"] = profile_key
    settings["enabled"] = bool(request.enabled)
    try:
        _persist_kinematics_calibration(settings)
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    log_event("calibration", "TCP calibration state changed", enabled=request.enabled, profile=profile_key)
    return {"ok": True, "config": public_config(), **kinematics_calibration_summary(config)}


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
        apply_calibration=request.apply_calibration,
        program_revision=request.program_revision,
    )


@app.post("/api/path/execute")
async def execute_path(request: PathExecuteRequest) -> dict[str, Any]:
    global path_task, path_task_source
    preview = path_previews.get(request.preview_id)
    if preview is None:
        state.set_error("path preview not found or expired")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    preview_revision = preview.get("program_revision")
    if preview.get("mode") == "program" and preview_revision is not None and request.program_revision != preview_revision:
        state.set_error("program changed since preview; preview the current sequence again")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    stale_reason = preview_stale_reason(preview)
    if stale_reason:
        state.set_error(stale_reason)
        log_event(
            "motion",
            "preview execution rejected",
            preview_id=request.preview_id,
            start_pose_revision=preview.get("start_pose_revision"),
            current_pose_revision=state.pose_revision,
            reason=stale_reason,
        )
        await broadcast_state()
        return {"ok": False, "error": stale_reason, "state": state.to_dict()}
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
    profiles = deepcopy(color_profiles(config))
    task_settings_raw = request.task_settings or {}
    profile_overrides = None
    if isinstance(task_settings_raw, dict):
        profile_overrides = (
            task_settings_raw.get("color_profile_overrides")
            or task_settings_raw.get("draft_color_profiles")
            or task_settings_raw.get("color_profiles")
        )
    if isinstance(profile_overrides, dict):
        for name, profile in profile_overrides.items():
            normalized_name = str(name).strip().lower()
            if normalized_name and isinstance(profile, dict):
                merged = deepcopy(profiles.get(normalized_name, {}))
                merged.update(deepcopy(profile))
                profiles[normalized_name] = merged
    try:
        path_settings = validated_task_path_settings(request.settings)
        if request.task in {"sorting", "color_sorting"}:
            detections = request.detections or ([request.detection] if request.detection else [])
            if not detections:
                raise TaskSettingsError("refresh detections before previewing a color-sorting task")
            sequence = build_color_sorting_plan(
                config,
                detections,
                profiles,
                task_settings=request.task_settings,
                selected_detection_ids=request.selected_detection_ids,
            )
        else:
            target = request.object_target or request.detection or {}
            if not target:
                raise TaskSettingsError("pick-and-place preview requires an object target")
            task_settings = normalize_color_sorting_settings(
                config,
                {"execution_strategy": "batch_once", **(request.task_settings or {})},
            )
            sequence = build_pick_and_place_sequence(config, target, request.drop_zone, task_settings=task_settings)
            sequence["task_preview"] = {
                "strategy": "batch_once",
                "normalized_settings": task_settings,
                "selected_objects": [
                    {
                        "index": 1,
                        "color": None,
                        "drop_zone": sequence.get("drop_zone"),
                        "object_target": sequence.get("object_target"),
                        "drop_target": sequence.get("drop_target"),
                        "motion_modes": sequence.get("motion_modes"),
                    }
                ] if sequence.get("ok") else [],
                "ignored_detections": [],
                "assigned_targets": [
                    {
                        "drop_zone": sequence.get("drop_zone"),
                        "target": sequence.get("drop_target"),
                        "grid_slot": sequence.get("grid_slot"),
                    }
                ] if sequence.get("ok") else [],
                "motion_modes": sequence.get("motion_modes", {}),
                "warnings": [],
                "estimated_duration_s": 0.0,
            }
    except TaskSettingsError as exc:
        state.set_error(str(exc))
        await broadcast_state()
        return {
            "ok": False,
            "error": str(exc),
            "sequence": {"ok": False, "steps": [], "waypoints": []},
            "task_preview": {"warnings": [], "ignored_detections": [], "selected_objects": []},
            "state": state.to_dict(),
        }
    if not sequence["ok"]:
        state.set_error("; ".join(sequence.get("errors", [])) or "task preview failed")
        await broadcast_state()
        return {
            "ok": False,
            "error": state.last_error,
            "sequence": sequence,
            "task_preview": sequence.get("task_preview", {}),
            "state": state.to_dict(),
        }

    preview_result = build_preview(
        mode="program",
        target=None,
        waypoint_program=sequence["waypoints"],
        links=config.links,
        settings=path_settings,
        branch=request.branch,
        source="task",
    )
    if not preview_result["ok"]:
        state.set_error(preview_result.get("error", "task motion preview failed"))
        await broadcast_state()
        return {
            **preview_result,
            "sequence": sequence,
            "task_preview": sequence.get("task_preview", {}),
            "state": state.to_dict(),
        }
    preview_id = preview_result["preview_id"]
    task_preview = dict(sequence.get("task_preview", {}))
    task_preview["estimated_duration_s"] = preview_result["preview"].get("trajectory", {}).get("duration_s", 0.0)
    sequence["task_preview"] = task_preview
    task_previews[preview_id] = {
        "id": preview_id,
        "created_at": time(),
        "start_pose_revision": preview_result["preview"]["start_pose_revision"],
        "start_reported_angles_deg": preview_result["preview"]["start_reported_angles_deg"],
        "start_reported_at": preview_result["preview"]["start_reported_at"],
        "start_pose_source": preview_result["preview"]["start_pose_source"],
        "model_fingerprint": preview_result["preview"]["model_fingerprint"],
        "sequence": sequence,
        "task": sequence.get("task", request.task),
        "strategy": sequence.get("strategy") or task_preview.get("strategy", "batch_once"),
        "task_settings": task_preview.get("normalized_settings") or request.task_settings or {},
        "task_preview": task_preview,
        "settings": path_settings,
        "branch": request.branch,
        "config_id": RUNNING_CONFIG_ID,
        "consumed": False,
    }
    for stale_id, stale in list(task_previews.items()):
        if time() - float(stale.get("created_at", 0.0)) > TASK_PREVIEW_TTL_S:
            task_previews.pop(stale_id, None)
    preview_result["sequence"] = sequence
    preview_result["task_preview"] = task_preview
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
    if preview.get("consumed"):
        state.set_error("task preview has already been executed; preview the task again")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if time() - float(preview.get("created_at", 0.0)) > TASK_PREVIEW_TTL_S:
        task_previews.pop(request.preview_id, None)
        state.set_error("task preview expired; preview the task again")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if preview.get("config_id") != RUNNING_CONFIG_ID:
        state.set_error("robot configuration changed after preview; preview the task again")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    stale_reason = preview_stale_reason(preview)
    if stale_reason:
        state.set_error(stale_reason.replace("preview again", "preview the task again"))
        log_event(
            "task",
            "task preview execution rejected",
            preview_id=request.preview_id,
            start_pose_revision=preview.get("start_pose_revision"),
            current_pose_revision=state.pose_revision,
            reason=state.last_error,
        )
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    gate_reason = task_motion_gate_reason()
    if gate_reason:
        state.set_error(gate_reason)
        await broadcast_state()
        return {"ok": False, "error": gate_reason, "state": state.to_dict()}
    if any(task is not None and not task.done() for task in [path_task, live_task, task_task]):
        state.set_error("motion or task execution is already running")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    strategy = str(preview.get("strategy") or preview.get("task_preview", {}).get("strategy") or "batch_once")
    task_settings = preview.get("task_settings", {})
    if isinstance(task_settings, dict) and task_settings.get("_has_unsaved_color_profiles"):
        state.set_error("save draft color profiles and drop preset mappings before starting the task")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if strategy == "closed_loop" and state.simulation and not simulation_vision_queue:
        state.set_error("closed-loop simulation requires queued synthetic vision frames")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if strategy == "closed_loop" and not state.simulation and not camera_settings(config).get("enabled"):
        state.set_error("closed-loop task execution requires an enabled camera")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    sequence = preview.get("sequence", {})
    total_objects = int(
        sequence.get("object_count")
        or len(sequence.get("objects", []))
        or len(preview.get("task_preview", {}).get("selected_objects", []))
        or 1
    )
    if strategy == "closed_loop":
        total_objects = int(preview.get("task_settings", {}).get("max_objects", total_objects) or total_objects)
    run_id = str(uuid4())
    start_task_execution_state(
        run_id=run_id,
        preview_id=request.preview_id,
        task=str(sequence.get("task") or preview.get("task") or "task"),
        strategy=strategy,
        total_objects=total_objects,
        settings=preview.get("task_settings", {}),
    )
    preview["consumed"] = True
    if strategy == "closed_loop":
        task_task = asyncio.create_task(execute_closed_loop_sorting(preview))
    else:
        task_task = asyncio.create_task(
            execute_task_sequence(
                sequence,
                {
                    **preview.get("settings", {}),
                    "tool_action_delay_ms": preview.get("task_settings", {}).get("tool_action_delay_ms", 150),
                },
                preview.get("branch", "auto"),
            )
        )
    state.last_command = f"TASK_EXECUTE {request.preview_id}"
    log_event("task", "task execution started", preview_id=request.preview_id)
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/task/select")
async def select_task_detection(request: TaskSelectionRequest) -> dict[str, Any]:
    execution = state.task_execution or {}
    if execution.get("run_id") != request.run_id:
        state.set_error("task selection run ID does not match the active task")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if execution.get("status") != "waiting_for_selection":
        state.set_error("task is not waiting for a manual detection selection")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    event = task_selection_events.get(request.run_id)
    if event is None:
        state.set_error("manual selection waiter is not available")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    task_selection_choices[request.run_id] = request.detection_id
    update_task_execution(
        phase="selection_received",
        current_step={"label": "selection received", "kind": "operator"},
    )
    event.set()
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/task/stop")
async def stop_task() -> dict[str, Any]:
    active_task = task_task
    result = await stop()
    if active_task is not None and active_task is not asyncio.current_task() and not active_task.done():
        try:
            await active_task
        except asyncio.CancelledError:
            pass
    if state.task_execution and state.task_execution.get("status") in ACTIVE_TASK_STATUSES:
        finish_task_execution("stopped", "task stop requested", holding_uncertain=True)
    await broadcast_state()
    return {"ok": bool(result.get("ok")), "state": state.to_dict()}


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
            **pose_snapshot_fields(),
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


async def _apply_cartesian_servo_target(
    target_deg: list[float],
    joint_velocity_deg_s: list[float],
    period_s: float,
) -> str:
    state.target_angles_deg = [float(value) for value in target_deg]
    has_commanded_velocity = any(abs(float(value)) > 0.001 for value in joint_velocity_deg_s)
    state.motion_state = MotionState.MOVING if has_commanded_velocity else MotionState.IDLE
    state.motion_execution_state = "cartesian_jog"
    state.clear_error()
    if state.simulation:
        state.update_reported_pose(
            state.target_angles_deg,
            source="simulation",
            known_pose=True,
        )
        state.fk = forward_kinematics(state.reported_angles_deg, config.links)
        limiter.reset(state.reported_angles_deg)
        state.last_controller_response = "SIMULATION"
        state.last_command = "CARTESIAN_JOG_SIM"
        return state.last_controller_response
    if not serial_client.is_connected:
        raise SerialClientError("Cartesian servo lost the hardware connection")

    # Each command is a finite synchronized position segment. The controller
    # does not independently smooth joint velocities, so the PC remains the
    # single authoritative servo loop.
    command = format_servoj(target_deg, period_s)
    state.last_command = command
    response = await asyncio.to_thread(send_servoj_and_read_response, command)
    now = monotonic()
    if now - float(cartesian_jog_runtime.get("last_status_poll") or 0.0) >= 0.25:
        try:
            await asyncio.to_thread(refresh_serial_status)
            cartesian_jog_runtime["last_status_poll"] = now
        except SerialClientError:
            # The next segment or explicit stop will surface a hard transport
            # failure; a delayed status response must not jitter the servo.
            pass
    return response


async def _run_cartesian_servo() -> None:
    global cartesian_jog_task
    period_s = _cartesian_servo_period_s()
    next_tick = monotonic()
    run_id = str(cartesian_jog_runtime.get("run_id") or "")
    try:
        while cartesian_jog_runtime.get("active"):
            now = monotonic()
            command_velocity = [
                float(value)
                for value in (cartesian_jog_runtime.get("command_velocity") or [0.0, 0.0, 0.0, 0.0])
            ]
            last_input = float(cartesian_jog_runtime.get("last_input") or 0.0)
            if (
                any(abs(value) > 1e-6 for value in command_velocity)
                and last_input > 0.0
                and now - last_input > CARTESIAN_JOG_STALE_S
            ):
                command_velocity = [0.0, 0.0, 0.0, 0.0]
                cartesian_jog_runtime["command_velocity"] = command_velocity
                cartesian_jog_runtime["stop_requested"] = True
                cartesian_jog_runtime["stop_reason"] = "Cartesian jog input watchdog expired"
                cartesian_servo.set_command(command_velocity)

            settings = cartesian_jog_runtime.get("settings")
            if not isinstance(settings, dict):
                settings = request_settings(None)
            result = cartesian_servo.step(period_s, _cartesian_servo_limits(settings))
            try:
                response = await _apply_cartesian_servo_target(
                    [float(value) for value in result["target_angles_deg"]],
                    [float(value) for value in result["joint_velocity_deg_s"]],
                    period_s,
                )
            except SerialClientError as exc:
                state.set_error(str(exc), fault=True)
                if run_id:
                    finish_motion_diagnostics("failed", str(exc), run_id)
                break

            result = {**result, "controller_response": response, "period_s": period_s}
            cartesian_jog_runtime["last_result"] = result
            cartesian_jog_runtime["joint_velocity_deg_s"] = result["joint_velocity_deg_s"]
            if run_id:
                update_motion_diagnostics(
                    run_id,
                    execution_state="cartesian_jog",
                    result="executing",
                    requested_target_deg=result["target_angles_deg"],
                    active_target_deg=result["target_angles_deg"],
                    cartesian_command={
                        "target_velocity": result["target_task_velocity"],
                        "applied_velocity": result["applied_task_velocity"],
                        "velocity_scale": result["velocity_scale"],
                        "period_s": period_s,
                    },
                )
                record_motion_sample(run_id)

            state.updated_at = time()
            if now - float(cartesian_jog_runtime.get("last_broadcast") or 0.0) >= 0.1:
                cartesian_jog_runtime["last_broadcast"] = now
                await broadcast_state()

            if cartesian_servo.is_stopped():
                break
            next_tick += period_s
            await asyncio.sleep(max(0.0, next_tick - monotonic()))
    except asyncio.CancelledError:
        raise
    finally:
        cartesian_jog_runtime["active"] = False
        cartesian_jog_runtime["command_velocity"] = [0.0, 0.0, 0.0, 0.0]
        cartesian_jog_runtime["joint_velocity_deg_s"] = [0.0 for _ in config.joints]
        cartesian_servo.set_command([0.0, 0.0, 0.0, 0.0])
        if state.motion_state != MotionState.FAULT:
            state.motion_state = MotionState.IDLE if state.simulation else MotionState.STOPPED
        state.motion_execution_state = "idle"
        if not state.simulation and serial_client.is_connected:
            try:
                await asyncio.to_thread(send_jog_stop_and_read_response)
            except SerialClientError as exc:
                state.set_error(str(exc), fault=True)
        stop_reason = str(cartesian_jog_runtime.get("stop_reason") or "Cartesian jog stopped")
        if run_id and state.motion_state != MotionState.FAULT:
            finish_motion_diagnostics("stopped", stop_reason, run_id)
        if cartesian_jog_task is asyncio.current_task():
            cartesian_jog_task = None
        await broadcast_state()


async def stop_cartesian_jog_internal(reason: str = "cartesian jog stopped") -> dict[str, Any]:
    cartesian_jog_runtime["stop_requested"] = True
    cartesian_jog_runtime["stop_reason"] = reason
    cartesian_jog_runtime["last_input"] = monotonic()
    cartesian_jog_runtime["command_velocity"] = [0.0, 0.0, 0.0, 0.0]
    cartesian_servo.set_command([0.0, 0.0, 0.0, 0.0])
    task = cartesian_jog_task
    if task is not None and not task.done() and task is not asyncio.current_task():
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout=1.5)
        except TimeoutError:
            task.cancel()
    elif not cartesian_jog_runtime.get("active"):
        state.motion_state = MotionState.IDLE if state.simulation else MotionState.STOPPED
        state.motion_execution_state = "idle"
        if not state.simulation and serial_client.is_connected:
            try:
                await asyncio.to_thread(send_jog_stop_and_read_response)
            except SerialClientError as exc:
                state.set_error(str(exc), fault=True)
                await broadcast_state()
                return {"ok": False, "error": str(exc), "state": state.to_dict()}
    state.last_command = "CARTESIAN_JOG_STOP"
    await broadcast_state()
    return {"ok": state.motion_state != MotionState.FAULT, "state": state.to_dict()}


@app.post("/api/cartesian-jog")
async def cartesian_jog(request: CartesianJogRequest) -> dict[str, Any]:
    global active_motion_run_id, cartesian_jog_task
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
    run_id = cartesian_jog_runtime.get("run_id")
    if not cartesian_jog_runtime.get("active") or not run_id:
        cartesian_servo.reset(state.reported_angles_deg)
        run_id = start_motion_diagnostics(
            source="cartesian_jog",
            mode="cartesian_jog",
            target_deg=state.reported_angles_deg,
            expected_duration_s=0.0,
            waypoint_count=0,
        )
        cartesian_jog_runtime["run_id"] = run_id

    command_velocity = [vx, vy, vz, vphi]
    cartesian_servo.set_command(command_velocity)
    if request.dt_s is not None:
        period_s = _clamp_scalar(float(request.dt_s), 0.005, 0.1)
        result = cartesian_servo.step(period_s, _cartesian_servo_limits(settings))
        try:
            response = await _apply_cartesian_servo_target(
                [float(value) for value in result["target_angles_deg"]],
                [float(value) for value in result["joint_velocity_deg_s"]],
                period_s,
            )
        except SerialClientError as exc:
            state.set_error(str(exc), fault=True)
            finish_motion_diagnostics("failed", str(exc), str(run_id))
            await broadcast_state()
            return {"ok": False, "error": str(exc), "state": state.to_dict()}
        result = {
            **result,
            "controller_response": response,
            "period_s": period_s,
            "notes": [*result.get("notes", []), *clamp_notes],
        }
        cartesian_jog_runtime.update(
            {
                "active": False,
                "last_input": now,
                "command_velocity": [0.0, 0.0, 0.0, 0.0],
                "joint_velocity_deg_s": result["joint_velocity_deg_s"],
                "run_id": run_id,
                "settings": settings,
                "last_result": result,
                "stop_requested": False,
                "stop_reason": None,
            }
        )
        cartesian_servo.set_command([0.0, 0.0, 0.0, 0.0])
        update_motion_diagnostics(
            str(run_id),
            execution_state="cartesian_jog",
            result="executing",
            requested_target_deg=result["target_angles_deg"],
            active_target_deg=result["target_angles_deg"],
            cartesian_command={
                "target_velocity": result["target_task_velocity"],
                "applied_velocity": result["applied_task_velocity"],
                "velocity_scale": result["velocity_scale"],
                "period_s": period_s,
            },
        )
        record_motion_sample(str(run_id))
        active_motion_run_id = None
        state.updated_at = time()
        await broadcast_state()
        return {"ok": True, "jog": result, "state": state.to_dict()}

    cartesian_jog_runtime.update(
        {
            "active": True,
            "last_input": now,
            "command_velocity": command_velocity,
            "run_id": run_id,
            "settings": settings,
            "stop_requested": False,
            "stop_reason": None,
        }
    )
    if cartesian_jog_task is None or cartesian_jog_task.done():
        cartesian_jog_task = asyncio.create_task(_run_cartesian_servo())
    await asyncio.sleep(0)
    state.updated_at = time()
    jog_result = cartesian_jog_runtime.get("last_result") or {
        "ok": True,
        "blocked": False,
        "target_task_velocity": command_velocity,
        "applied_task_velocity": [0.0, 0.0, 0.0, 0.0],
        "target_angles_deg": cartesian_servo.commanded_joints_deg.tolist(),
        "joint_velocity_deg_s": [0.0 for _ in config.joints],
        "notes": [],
    }
    jog_result = {**jog_result, "notes": [*jog_result.get("notes", []), *clamp_notes]}
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
    try:
        with tempfile.TemporaryDirectory() as tmp_dir:
            draft_path = Path(tmp_dir) / "robot.local.yaml"
            shutil.copyfile(config_path, draft_path)
            save_calibration_updates(draft_path, updates)
            draft_config = load_config(draft_path)
            change = classify_config_change(config, draft_config)
            ready, reason = config_change_ready(change)
            if not ready:
                state.set_error(reason)
                await broadcast_state()
                return {
                    "ok": False,
                    "error": reason,
                    "config_change": change,
                    "state": state.to_dict(),
                }

        save_calibration_updates(config_path, updates)
        saved_config = load_config(config_path)
        reload_runtime_config(saved_config, change)
        log_validation_warnings()
    except Exception as exc:
        state.set_error(f"could not save calibration: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    state.last_command = "SAVE_CALIBRATION"
    state.clear_error()
    log_event(
        "config",
        "calibration saved",
        path=str(config.source_path),
        categories=change.get("categories", []),
        sync_required=bool(change.get("sync_required")),
        pose_invalidated=bool(change.get("pose_invalidated")),
    )
    await broadcast_state()
    return {
        "ok": True,
        "config": public_config(),
        "config_change": state.config_change,
        "state": state.to_dict(),
    }


@app.post("/api/connect")
async def connect(request: ConnectRequest) -> dict[str, Any]:
    cancel_motion_tasks()
    disable_live_motion()
    if request.simulation is True:
        state.simulation = True
        state.connected = True
        state.serial_port = None
        state.hardware_armed = False
        state.update_reported_pose(
            state.reported_angles_deg,
            source="simulation",
            known_pose=True,
        )
        apply_hardware_evaluation("simulation", "simulation mode")
        state.motion_state = MotionState.IDLE
        state.last_command = "SIMULATION CONNECT"
        state.clear_error()
        log_event("connection", "simulation connected")
        await broadcast_state()
        return {"ok": True, "state": state.to_dict()}

    simulation_vision_queue.clear()
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
        state.update_reported_pose(
            state.reported_angles_deg,
            source="unknown",
            known_pose=False,
        )
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
    simulation_vision_queue.clear()
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
    state.update_reported_pose(
        state.reported_angles_deg,
        source="unknown",
        known_pose=False,
    )
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
    response = await start_joint_target_trajectory(config.home_pose, "home")
    if response["ok"]:
        log_event("motion", "go home accepted", preview_id=response.get("preview_id"))
    return response


@app.post("/api/stop")
async def stop() -> dict[str, Any]:
    cancel_motion_tasks()
    pose_revision_before = state.pose_revision
    state.live_motion_enabled = False
    state.target_angles_deg = state.reported_angles_deg.copy()
    limiter.set_target(state.target_angles_deg)
    state.motion_state = MotionState.STOPPED
    state.last_command = format_stop()
    finish_motion_diagnostics("stopped", "STOP")
    if task_active():
        finish_task_execution("stopped", "STOP", holding_uncertain=True)
    if not state.simulation and serial_client.is_connected:
        serial_client.send_line(format_stop())
        refresh_serial_status()
        align_target_to_reported()
    state.update_reported_pose(
        state.reported_angles_deg,
        source=state.pose_source,
        known_pose=state.known_pose,
        force_revision=state.pose_revision == pose_revision_before,
    )
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/estop")
async def estop() -> dict[str, Any]:
    cancel_motion_tasks()
    pose_revision_before = state.pose_revision
    state.target_angles_deg = state.reported_angles_deg.copy()
    limiter.set_target(state.target_angles_deg)
    state.motion_state = MotionState.ESTOP
    state.hardware_armed = False
    state.live_motion_enabled = False
    state.last_command = format_estop()
    finish_motion_diagnostics("stopped", "ESTOP")
    if task_active():
        finish_task_execution("stopped", "ESTOP", holding_uncertain=True)
    if not state.simulation and serial_client.is_connected:
        serial_client.send_line(format_estop())
        refresh_serial_status()
        align_target_to_reported()
    state.update_reported_pose(
        state.reported_angles_deg,
        source=state.pose_source,
        known_pose=state.known_pose,
        force_revision=state.pose_revision == pose_revision_before,
    )
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
    try:
        await websocket.send_json({"type": "config", "config": public_config()})
        await websocket.send_json({"type": "state", "state": state.to_dict()})
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
