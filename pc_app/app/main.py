from __future__ import annotations

import asyncio
import hashlib
import json
import subprocess
import shutil
import tempfile
from copy import deepcopy
from dataclasses import asdict, replace
from datetime import datetime, timezone
from itertools import product
from math import dist, isfinite, sqrt
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
    DEFAULT_CALIBRATION_MODEL,
    calibration_context as kinematics_calibration_context,
    calibration_settings as kinematics_calibration_settings,
    calibration_summary as kinematics_calibration_summary,
    correct_cartesian_target,
    correct_waypoint_program,
    create_sample as create_kinematics_calibration_sample,
    fit_profile as fit_kinematics_calibration_profile,
    predict_physical_pose,
    save_manual_radial_offsets,
    workspace_context as kinematics_workspace_context,
)
from .calibration_truth import model_truth_summary
from .config import LinkConfig, RobotConfig, ensure_local_config, load_config, save_calibration_updates
from .encoder import (
    calibrated_joint_angle,
    empty_evidence,
    encoder_axis,
    evidence_from_status,
    normalize_encoder_settings,
    validate_encoder_settings,
    wrapped_delta_deg,
)
from .demo_settings import (
    active_tool_dimensions_validated,
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
    ik_selection_policy,
)
from .position_library import (
    POSITION_LIBRARY_SCHEMA_VERSION,
    PositionLibraryError,
    position_library_errors,
    position_library_records,
    position_record_to_legacy_position,
    validate_position_record,
    normalize_position_record,
)
from .physical_model_calibration import (
    PARAMETER_GROUPS as PHYSICAL_MODEL_PARAMETER_GROUPS,
    fit_physical_model,
    physical_model_updates,
)
from .program_library import (
    PROGRAM_SCHEMA_VERSION,
    ProgramLibraryError,
    all_programs,
    copy_program_to_user,
    delete_user_program,
    find_program,
    program_motion_fingerprint,
    save_user_program_cached_plan,
    save_user_program,
)
from .task_destinations import (
    TASK_DESTINATIONS_SCHEMA_VERSION,
    TaskDestinationError,
    legacy_drop_zones_from_task_destinations,
    task_destination_errors,
    task_destination_payload,
    task_destinations as resolve_task_destinations,
)
from .protocol import (
    format_arm,
    format_alignj,
    format_config_lines,
    format_correctj,
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
    parse_hello_capabilities,
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
state.closed_loop_mode = str(encoder_settings(config).get("mode", "diagnostic"))
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
task_confirmation_events: dict[str, asyncio.Event] = {}
simulation_vision_queue: list[dict[str, Any]] = []
latest_vision_snapshot: dict[str, Any] = {}
encoder_calibration_sessions: dict[str, dict[str, Any]] = {}
encoder_calibration_sweep_task: asyncio.Task[None] | None = None
cartesian_jog_task: asyncio.Task[None] | None = None
event_log = EventLog()
active_motion_run_id: str | None = None
simulation_trajectory_active = False
MAX_TRAJECTORY_UPLOAD_POINTS = 220
TASK_PREVIEW_TTL_S = 600.0
PREVIEW_START_TOLERANCE_DEG = 0.1
CONTROLLER_REBASE_TOLERANCE_DEG = 0.25
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


def stable_payload_fingerprint(payload: Any) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()[:16]


def _fingerprint_number(value: Any, *, digits: int = 3) -> float | None:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if not isfinite(numeric):
        return None
    return round(numeric, digits)


def _task_detection_signature(detection: Any, index: int) -> dict[str, Any]:
    if not isinstance(detection, dict):
        return {"index": index, "invalid": True, "type": type(detection).__name__}

    robot = detection.get("robot") or detection.get("target") or {}
    if not isinstance(robot, dict):
        robot = {}
    bbox = detection.get("bbox_px") or detection.get("bbox") or {}
    if not isinstance(bbox, dict):
        bbox = {}
    area_px = detection.get("area_px")
    if area_px is None and bbox:
        width = _fingerprint_number(bbox.get("width"))
        height = _fingerprint_number(bbox.get("height"))
        area_px = width * height if width is not None and height is not None else None

    detection_id = detection.get("id", detection.get("detection_id", detection.get("object_id")))
    return {
        "index": index,
        "id": str(detection_id) if detection_id is not None else f"detection-{index + 1}",
        "ok": bool(detection.get("ok", True)),
        "color": str(detection.get("label", detection.get("color", ""))).strip().lower(),
        "confidence": _fingerprint_number(detection.get("confidence", detection.get("quality", 1.0)), digits=4),
        "area_px": _fingerprint_number(area_px, digits=2),
        "drop_zone": str(detection.get("drop_zone") or ""),
        "robot": {
            "x_mm": _fingerprint_number(robot.get("x_mm", robot.get("x"))),
            "y_mm": _fingerprint_number(robot.get("y_mm", robot.get("y"))),
            "z_mm": _fingerprint_number(robot.get("z_mm", robot.get("z"))),
        },
    }


def task_detection_fingerprint(detections: list[Any]) -> str:
    """Fingerprint only the detection fields that can affect task planning.

    The raw vision payload may contain volatile detector metadata or browser
    round-trip representation differences. A task preview only needs to know
    whether the object queue that drives filtering and motion targets changed.
    """

    signatures = [
        _task_detection_signature(detection, index)
        for index, detection in enumerate(detections)
    ]
    return stable_payload_fingerprint(signatures)


def public_program_record(program: dict[str, Any]) -> dict[str, Any]:
    record = deepcopy(program)
    cached_plan = record.pop("cached_plan", None)
    if isinstance(cached_plan, dict):
        preview = cached_plan.get("preview") if isinstance(cached_plan.get("preview"), dict) else {}
        trajectory = preview.get("trajectory") if isinstance(preview.get("trajectory"), dict) else {}
        record["cached_plan"] = {
            "available": True,
            "saved_at": cached_plan.get("saved_at"),
            "start_reported_angles_deg": cached_plan.get("start_reported_angles_deg"),
            "duration_s": trajectory.get("duration_s"),
            "waypoint_count": trajectory.get("waypoint_count"),
        }
    return record


def task_runtime_config(
    base_config: RobotConfig,
    task_settings: dict[str, Any] | None = None,
) -> RobotConfig:
    settings = task_settings if isinstance(task_settings, dict) else {}
    raw = deepcopy(base_config.raw)
    profile_overrides = settings.get("color_profile_overrides")
    if isinstance(profile_overrides, dict):
        profiles = deepcopy(raw.get("color_profiles", {}))
        for name, profile in profile_overrides.items():
            normalized = str(name).strip().lower()
            if not normalized or not isinstance(profile, dict):
                continue
            merged = deepcopy(profiles.get(normalized, {}))
            merged.update({key: deepcopy(value) for key, value in profile.items() if key != "draft"})
            profiles[normalized] = merged
        raw["color_profiles"] = profiles

    destination_overrides = settings.get("task_destination_overrides")
    if isinstance(destination_overrides, dict):
        nested = destination_overrides.get("destinations")
        source = nested if isinstance(nested, dict) else destination_overrides
        destinations = {
            str(name): deepcopy(destination)
            for name, destination in source.items()
            if name not in {"schema_version", "updated_at"} and isinstance(destination, dict)
        }
        raw["task_destinations"] = task_destination_payload(destinations)
        draft_config = replace(base_config, raw=raw)
        resolved = resolve_task_destinations(draft_config, named_positions(draft_config))
        raw["drop_zones"] = legacy_drop_zones_from_task_destinations(resolved)

    return replace(base_config, raw=raw)


def task_mapping_fingerprint(task_settings: dict[str, Any] | None = None) -> str:
    effective_config = task_runtime_config(config, task_settings)
    try:
        destinations = drop_zones(effective_config)
    except TaskDestinationError:
        destinations = effective_config.raw.get(
            "task_destinations",
            effective_config.raw.get("drop_zones", {}),
        )
    return stable_payload_fingerprint(
        {
            "color_profiles": color_profiles(effective_config),
            "task_destinations": destinations,
        }
    )


def register_vision_snapshot(payload: dict[str, Any]) -> dict[str, Any]:
    captured_at = float(payload.get("captured_at") or time())
    detections = payload.get("detections") if isinstance(payload.get("detections"), list) else []
    snapshot_id = str(payload.get("detection_snapshot_id") or uuid4())
    snapshot = {
        "id": snapshot_id,
        "captured_at": captured_at,
        "fingerprint": stable_payload_fingerprint(detections),
        "task_fingerprint": task_detection_fingerprint(detections),
        "provider": payload.get("provider"),
        "calibration_source": payload.get("calibration_source"),
    }
    latest_vision_snapshot.clear()
    latest_vision_snapshot.update(snapshot)
    return {
        **payload,
        "captured_at": captured_at,
        "detection_snapshot_id": snapshot_id,
        "detection_fingerprint": snapshot["fingerprint"],
        "detection_task_fingerprint": snapshot["task_fingerprint"],
    }


def task_contract_fingerprint(
    *,
    task_settings: dict[str, Any],
    path_settings: dict[str, Any],
    branch: str,
    selected_detection_ids: list[str] | None,
) -> str:
    return stable_payload_fingerprint(
        {
            "task_settings": task_settings,
            "path_settings": path_settings,
            "branch": branch,
            "selected_detection_ids": [str(value) for value in selected_detection_ids or []],
        }
    )


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


def encoder_tracking_preview_tolerance_deg() -> float:
    tracking = encoder_settings(config).get("pose_tracking", {})
    if not isinstance(tracking, dict) or not bool(tracking.get("enabled")):
        return PREVIEW_START_TOLERANCE_DEG
    try:
        return max(
            PREVIEW_START_TOLERANCE_DEG,
            float(tracking.get("preview_stale_tolerance_deg", 2.0)),
        )
    except (TypeError, ValueError):
        return max(PREVIEW_START_TOLERANCE_DEG, 2.0)


def preview_start_allowed_deltas() -> list[float]:
    allowed = [PREVIEW_START_TOLERANCE_DEG] * len(state.reported_angles_deg)
    tracking_applied = (
        state.pose_source == "encoder_shoulder_tracking"
        and state.encoder_mismatch.get("pose_tracking_status") == "applied"
    )
    shoulder_authority = state.joint_authority[1] if len(state.joint_authority) > 1 else ""
    shoulder_measured = state.measured_angles_deg[1] if len(state.measured_angles_deg) > 1 else None
    shoulder_encoder_tracked = (
        tracking_applied
        or state.pose_source == "encoder_shoulder_tracking"
        or (
            shoulder_authority == "measured"
            and shoulder_measured is not None
            and encoder_settings(config).get("pose_tracking", {}).get("enabled")
        )
    )
    if shoulder_encoder_tracked and len(allowed) > 1:
        allowed[1] = encoder_tracking_preview_tolerance_deg()
    return allowed


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
    allowed_deltas = preview_start_allowed_deltas()
    stale_indices = [
        index
        for index, delta in enumerate(deltas)
        if delta > allowed_deltas[index] + 1e-9
    ]
    if stale_indices:
        max_delta = max(deltas, default=0.0)
        max_allowed = max(allowed_deltas, default=PREVIEW_START_TOLERANCE_DEG)
        return (
            "preview start pose is stale: "
            f"planned at revision {preview.get('start_pose_revision')} from "
            f"{[round(float(value), 3) for value in start_angles]}, "
            f"current revision {state.pose_revision} is "
            f"{[round(float(value), 3) for value in state.reported_angles_deg]} "
            f"(max delta {max_delta:.3f} deg, allowed {max_allowed:.3f} deg); preview again"
        )
    return None


def rebase_preview_start_to_current_if_encoder_tracked(preview: dict[str, Any]) -> bool:
    start_angles = preview.get("start_reported_angles_deg")
    trajectory = preview.get("trajectory")
    if (
        not isinstance(start_angles, list)
        or len(start_angles) != len(state.reported_angles_deg)
        or not isinstance(trajectory, dict)
    ):
        return False
    waypoints = trajectory.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        return False
    first = waypoints[0]
    if not isinstance(first, list) or len(first) != len(state.reported_angles_deg):
        return False
    allowed = preview_start_allowed_deltas()
    deltas = [
        abs(float(current) - float(start))
        for current, start in zip(state.reported_angles_deg, start_angles, strict=True)
    ]
    if any(delta > allowed[index] + 1e-9 for index, delta in enumerate(deltas)):
        return False
    shoulder_drift = deltas[1] if len(deltas) > 1 else 0.0
    if shoulder_drift <= PREVIEW_START_TOLERANCE_DEG:
        return False
    current_start = [float(value) for value in state.reported_angles_deg]
    trajectory["waypoints"] = [current_start, *waypoints[1:]]
    preview["start_pose_revision"] = int(state.pose_revision)
    preview["start_reported_angles_deg"] = current_start
    preview["start_reported_at"] = float(state.reported_at)
    preview["start_pose_source"] = state.pose_source
    trajectory["encoder_tracking_start_rebase"] = {
        "rebased_at": time(),
        "shoulder_drift_deg": float(state.reported_angles_deg[1]) - float(start_angles[1]),
        "previous_start_deg": [float(value) for value in start_angles],
        "current_start_deg": current_start,
    }
    log_event(
        "motion",
        "preview start rebased to encoder-tracked shoulder",
        preview_id=preview.get("id", ""),
        shoulder_drift_deg=trajectory["encoder_tracking_start_rebase"]["shoulder_drift_deg"],
    )
    return True


def cache_program_preview(
    program_id: str,
    requested_waypoints: list[dict[str, Any]],
    preview: dict[str, Any],
) -> dict[str, Any]:
    program = find_program(config, program_id)
    if program is None:
        raise ProgramLibraryError(f"program {program_id} was not found")
    if program.get("read_only"):
        raise ProgramLibraryError("copy the built-in template before saving a compiled plan")
    requested_fingerprint = program_motion_fingerprint(
        {
            "schema_version": program.get("schema_version", PROGRAM_SCHEMA_VERSION),
            "required_tool": program.get("required_tool"),
            "steps": requested_waypoints,
        }
    )
    expected_fingerprint = program_motion_fingerprint(program)
    if requested_fingerprint != expected_fingerprint:
        raise ProgramLibraryError("saved program differs from the previewed sequence")

    cached_preview = deepcopy(preview)
    cached_preview.pop("id", None)
    cached_preview.pop("created_at", None)
    cached_preview.pop("execution_started_at", None)
    cached_preview.pop("execution_start_pose_revision", None)
    cached_preview.pop("execution_count", None)
    cached_preview["program_revision"] = None
    saved = save_user_program_cached_plan(
        config,
        program_id,
        {
            "backend_build_id": RUNNING_BACKEND_BUILD_ID,
            "config_id": RUNNING_CONFIG_ID,
            "model_fingerprint": robot_model_fingerprint(),
            "start_reported_angles_deg": [
                float(value) for value in preview.get("start_reported_angles_deg", [])
            ],
            "preview": cached_preview,
        },
    )
    return public_program_record(saved).get("cached_plan", {})


def restore_cached_program_preview(
    program: dict[str, Any],
    program_revision: int | None,
) -> dict[str, Any]:
    cached_plan = program.get("cached_plan")
    if not isinstance(cached_plan, dict):
        return {"ok": False, "cache_miss": True, "error": "No saved plan is available yet."}
    if cached_plan.get("program_fingerprint") != program_motion_fingerprint(program):
        return {
            "ok": False,
            "cache_miss": True,
            "error": "The saved program changed after its plan was created.",
        }
    if cached_plan.get("backend_build_id") != RUNNING_BACKEND_BUILD_ID:
        return {
            "ok": False,
            "cache_miss": True,
            "error": "Planner code changed after this plan was saved.",
        }
    if cached_plan.get("config_id") != RUNNING_CONFIG_ID:
        return {
            "ok": False,
            "cache_miss": True,
            "error": "Robot configuration changed after this plan was saved.",
        }
    if cached_plan.get("model_fingerprint") != robot_model_fingerprint():
        return {
            "ok": False,
            "cache_miss": True,
            "error": "Robot model changed after this plan was saved.",
        }

    preview = deepcopy(cached_plan.get("preview"))
    if not isinstance(preview, dict) or preview.get("mode") != "program":
        return {"ok": False, "cache_miss": True, "error": "The saved plan is invalid."}
    stale_reason = preview_stale_reason(preview)
    if stale_reason:
        return {"ok": False, "cache_miss": True, "error": stale_reason}

    preview_id = str(uuid4())
    preview.update(
        {
            "id": preview_id,
            "created_at": time(),
            "source": "saved_program_plan",
            "program_id": program["id"],
            "program_revision": program_revision,
            **pose_snapshot_fields(),
        }
    )
    path_previews[preview_id] = preview
    for stale_id, stale in list(path_previews.items()):
        if time() - stale.get("created_at", 0.0) > 600:
            path_previews.pop(stale_id, None)
    log_event(
        "program",
        "saved plan restored",
        program_id=program["id"],
        preview_id=preview_id,
    )
    return {
        "ok": True,
        "restored": True,
        "preview_id": preview_id,
        "preview": preview,
        "cached_plan": public_program_record(program).get("cached_plan", {}),
    }


def task_preview_stale_reason(preview: dict[str, Any]) -> str | None:
    base_reason = preview_stale_reason(preview)
    if base_reason:
        return base_reason
    if preview.get("destination_revision") != task_mapping_fingerprint(preview.get("task_settings")):
        return "task destinations or color mappings changed after preview; preview the task again"
    expected_contract = preview.get("task_settings_revision")
    actual_contract = task_contract_fingerprint(
        task_settings=preview.get("task_settings", {}),
        path_settings=preview.get("settings", {}),
        branch=str(preview.get("branch", "auto")),
        selected_detection_ids=preview.get("selected_detection_ids"),
    )
    if expected_contract != actual_contract:
        return "task settings changed after preview; preview the task again"
    if preview.get("detection_snapshot_server_bound"):
        snapshot_id = str(preview.get("detection_snapshot_id") or "")
        if not latest_vision_snapshot or snapshot_id != str(latest_vision_snapshot.get("id") or ""):
            return "detection snapshot changed after preview; refresh detections and preview the task again"
        preview_task_fingerprint = preview.get("detection_task_fingerprint")
        latest_task_fingerprint = latest_vision_snapshot.get("task_fingerprint")
        if preview_task_fingerprint and latest_task_fingerprint:
            if preview_task_fingerprint != latest_task_fingerprint:
                return "detection contents changed after preview; refresh detections and preview the task again"
        elif preview.get("detection_fingerprint") != latest_vision_snapshot.get("fingerprint"):
            return "detection contents changed after preview; refresh detections and preview the task again"
    return None


def _task_failed_waypoint_index(preview_result: dict[str, Any]) -> int | None:
    trajectory = preview_result.get("trajectory")
    if not isinstance(trajectory, dict):
        preview = preview_result.get("preview")
        trajectory = preview.get("trajectory") if isinstance(preview, dict) else None
    if not isinstance(trajectory, dict):
        return None
    step_results = trajectory.get("step_results")
    if not isinstance(step_results, list):
        return None
    for result in step_results:
        if not isinstance(result, dict):
            continue
        status = str(result.get("status") or "").lower()
        errors = result.get("errors")
        if status == "invalid" or (isinstance(errors, list) and errors):
            try:
                return int(result.get("index"))
            except (TypeError, ValueError):
                return None
    return None


def _task_failed_object(preview_result: dict[str, Any], sequence: dict[str, Any]) -> dict[str, Any] | None:
    waypoint_index = _task_failed_waypoint_index(preview_result)
    waypoints = sequence.get("waypoints")
    if waypoint_index is None or not isinstance(waypoints, list) or not 0 <= waypoint_index < len(waypoints):
        return None
    waypoint = waypoints[waypoint_index]
    if not isinstance(waypoint, dict) or waypoint.get("object_index") is None:
        return None
    try:
        object_index = int(waypoint["object_index"])
    except (TypeError, ValueError):
        return None
    for item in sequence.get("objects", []):
        if isinstance(item, dict) and int(item.get("index", -1)) == object_index:
            return item
    return {"index": object_index}


def _task_motion_failure_message(preview_result: dict[str, Any]) -> str:
    error = str(preview_result.get("error") or "").strip()
    if error:
        return error
    trajectory = preview_result.get("trajectory")
    if isinstance(trajectory, dict):
        errors = trajectory.get("errors")
        if isinstance(errors, list) and errors:
            return "; ".join(str(item) for item in errors)
    return "object has no valid IK path"


def _drop_task_object_for_motion_failure(
    sequence: dict[str, Any],
    failed_object: dict[str, Any],
    reason: str,
) -> dict[str, Any]:
    skipped_original_index = int(failed_object.get("index", -1))
    updated = deepcopy(sequence)
    updated["steps"] = [
        step
        for step in updated.get("steps", [])
        if not isinstance(step, dict) or int(step.get("object_index", -1)) != skipped_original_index
    ]
    updated["waypoints"] = [
        waypoint
        for waypoint in updated.get("waypoints", [])
        if not isinstance(waypoint, dict) or int(waypoint.get("object_index", -1)) != skipped_original_index
    ]
    remaining_objects = [
        obj
        for obj in updated.get("objects", [])
        if isinstance(obj, dict) and int(obj.get("index", -1)) != skipped_original_index
    ]
    index_map = {
        int(obj.get("index")): new_index
        for new_index, obj in enumerate(remaining_objects, start=1)
        if obj.get("index") is not None
    }
    for obj in remaining_objects:
        old_index = int(obj.get("index", 0))
        obj["index"] = index_map.get(old_index, old_index)
    for collection_name in ("steps", "waypoints"):
        for item in updated.get(collection_name, []):
            if isinstance(item, dict) and item.get("object_index") is not None:
                old_index = int(item.get("object_index"))
                if old_index in index_map:
                    item["object_index"] = index_map[old_index]
    updated["objects"] = remaining_objects
    updated["object_count"] = len(remaining_objects)

    detection_id = str(failed_object.get("detection_id") or f"object-{skipped_original_index}")
    skip_message = f"skipped {detection_id}: {reason}"
    preview = updated.get("task_preview")
    if isinstance(preview, dict):
        preview["selected_objects"] = remaining_objects
        preview["next_object"] = remaining_objects[0] if remaining_objects else None
        warnings = list(preview.get("warnings") or [])
        warnings.append(skip_message)
        preview["warnings"] = warnings
        ignored = list(preview.get("ignored_detections") or [])
        ignored.append(
            {
                "detection_id": detection_id,
                "color": failed_object.get("color"),
                "ok": False,
                "reason_code": "ik_unreachable",
                "reason": reason,
                "message": skip_message,
            }
        )
        preview["ignored_detections"] = ignored
        candidate_objects = preview.get("candidate_objects")
        if isinstance(candidate_objects, list):
            preview["candidate_objects"] = [
                item
                for item in candidate_objects
                if not isinstance(item, dict) or str(item.get("detection_id")) != detection_id
            ]
        assigned_targets = preview.get("assigned_targets")
        if isinstance(assigned_targets, list):
            preview["assigned_targets"] = [
                item
                for item in assigned_targets
                if not isinstance(item, dict) or str(item.get("detection_id")) != detection_id
            ]
    return updated


def build_task_motion_preview_skipping_failed_objects(
    sequence: dict[str, Any],
    *,
    links: LinkConfig,
    settings: dict[str, Any],
    branch: str,
) -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]]]:
    current_sequence = deepcopy(sequence)
    skipped: list[dict[str, Any]] = []
    skipped_indexes: set[int] = set()
    while True:
        preview_result = build_preview(
            mode="program",
            target=None,
            waypoint_program=current_sequence.get("waypoints", []),
            links=links,
            settings=settings,
            branch=branch,
            source="task",
        )
        if preview_result.get("ok"):
            return preview_result, current_sequence, skipped
        failed_object = _task_failed_object(preview_result, current_sequence)
        if failed_object is None:
            return preview_result, current_sequence, skipped
        failed_index = int(failed_object.get("index", -1))
        if failed_index in skipped_indexes:
            return preview_result, current_sequence, skipped
        skipped_indexes.add(failed_index)
        reason = _task_motion_failure_message(preview_result)
        skipped.append({**deepcopy(failed_object), "reason": reason})
        current_sequence = _drop_task_object_for_motion_failure(current_sequence, failed_object, reason)
        if not current_sequence.get("objects") or not current_sequence.get("waypoints"):
            failure = {
                **preview_result,
                "ok": False,
                "error": (
                    "no task objects have a safe continuous IK path"
                    + (f"; last rejection: {reason}" if reason else "")
                ),
            }
            return failure, current_sequence, skipped


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
    tcp_speed_mm_s: float | None = None
    phi_speed_deg_s: float | None = None
    tcp_accel_mm_s2: float | None = None
    phi_accel_deg_s2: float | None = None
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
    apply_calibration: bool = True


class PathPreviewRequest(BaseModel):
    target: IkTargetRequest | None = None
    mode: str = "joint"
    links_mm: dict[str, float] | None = None
    branch: str = "auto"
    settings: PathSettingsRequest | None = None
    waypoints: list[dict[str, Any]] | None = None
    apply_calibration: bool = True
    purpose: str | None = None
    program_revision: int | None = None
    program_id: str | None = None


class PathExecuteRequest(BaseModel):
    preview_id: str
    program_revision: int | None = None


class PathGoRequest(BaseModel):
    waypoints: list[dict[str, Any]]
    branch: str = "auto"
    settings: PathSettingsRequest | None = None


class HomeRequest(BaseModel):
    settings: PathSettingsRequest | None = None


class ProgramSaveRequest(BaseModel):
    id: str | None = None
    name: str
    description: str = ""
    steps: list[dict[str, Any]]
    required_tool: str | None = None
    schema_version: int = PROGRAM_SCHEMA_VERSION


class ProgramCopyRequest(BaseModel):
    name: str | None = None


class ProgramRestorePlanRequest(BaseModel):
    program_revision: int | None = None


class ProgramStepPreviewRequest(BaseModel):
    waypoints: list[dict[str, Any]]
    step_index: int
    branch: str = "auto"
    settings: PathSettingsRequest | None = None
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
    position_library: dict[str, Any] | None = None
    named_positions: dict[str, dict[str, Any]] | None = None
    camera: dict[str, Any] | None = None
    color_profiles: dict[str, dict[str, Any]] | None = None
    task_destinations: dict[str, Any] | None = None
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


class EncoderFaultClearRequest(BaseModel):
    acknowledge_pose_unknown: bool = False


class EncoderCalibrationStartRequest(BaseModel):
    mounting_location: str = "joint_output"
    reference_description: str = ""
    joint_angle_deg: float | None = None
    capture_initial: bool = False


class EncoderCalibrationSampleRequest(BaseModel):
    session_id: str
    joint_angle_deg: float
    label: str = ""


class EncoderCalibrationSessionRequest(BaseModel):
    session_id: str


class EncoderCalibrationCommitRequest(BaseModel):
    session_id: str
    confirm: bool = False


class EncoderQuickCalibrationRequest(BaseModel):
    joint_angle_deg: float
    direction_sign: int = 1
    sensor_turns_per_joint_turn: float = 1.0
    mounting_location: str = "joint_output"
    reference_description: str = ""
    confirm_one_to_one_output_mount: bool = False


class EncoderCalibrationSweepStartRequest(BaseModel):
    start_joint_angle_deg: float
    sweep_min_deg: float
    sweep_max_deg: float
    step_deg: float = 15.0
    final_approach_direction: int = 1
    preload_deg: float = 8.0
    speed_deg_s: float = 6.0
    accel_deg_s2: float = 24.0
    settle_ms: int = 350
    mounting_location: str = "joint_output"
    reference_description: str = ""
    confirm_open_loop_sweep: bool = False


class EncoderCalibrationSweepSessionRequest(BaseModel):
    session_id: str


class EncoderBacklashCheckRequest(BaseModel):
    center_joint_angle_deg: float | None = None
    travel_deg: float = 10.0
    repeats: int = 1
    speed_deg_s: float = 6.0
    settle_ms: int = 350


class EncoderCorrectionPolicyRequest(BaseModel):
    enabled: bool
    confirm: bool = False


class EncoderShoulderAlignRequest(BaseModel):
    settings: PathSettingsRequest | dict[str, Any] | None = None


class ToolRequest(BaseModel):
    action: str
    value: float | None = None
    tool: str | None = None


class ToolsRequest(BaseModel):
    active: str
    presets: dict[str, dict[str, Any]] | None = None


class ToolDimensionsValidationRequest(BaseModel):
    validated: bool = True


class NamedPositionsRequest(BaseModel):
    positions: dict[str, dict[str, Any]]


class PositionLibraryRequest(BaseModel):
    positions: dict[str, dict[str, Any]]


class TaskMappingsRequest(BaseModel):
    color_profiles: dict[str, dict[str, Any]]
    task_destinations: dict[str, Any]


class VisionSettingsRequest(BaseModel):
    camera: dict[str, Any] | None = None
    color_profiles: dict[str, dict[str, Any]] | None = None
    task_destinations: dict[str, Any] | None = None
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
    max_frames: int = 120
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
    detection_snapshot_id: str | None = None
    detection_captured_at: float | None = None
    branch: str = "auto"


class TaskExecuteRequest(BaseModel):
    preview_id: str


class TaskSelectionRequest(BaseModel):
    run_id: str
    detection_id: str


class TaskContinueRequest(BaseModel):
    run_id: str


class KinematicsCalibrationTargetsRequest(BaseModel):
    count: int = 12
    z_mm: float = 45.0
    phi_deg: float = 0.0
    z_levels_mm: list[float] | None = None
    phi_levels_deg: list[float] | None = None
    validation_stride: int = 4
    apply_calibration: bool = False


class KinematicsCalibrationSampleRequest(BaseModel):
    intended_target: dict[str, Any]
    command_target: dict[str, Any] | None = None
    measured: dict[str, Any]
    role: str = "fit"
    quality: float = 1.0
    measurement_source: dict[str, Any] | None = None
    preview_id: str | None = None
    measured_point: str = "active_tcp"
    reference_frame: str = "robot_base"
    approach: dict[str, Any] | None = None
    joint_source: str = "reported"
    notes: str = ""


class KinematicsCalibrationFitRequest(BaseModel):
    model_type: str = DEFAULT_CALIBRATION_MODEL
    profile_key: str | None = None
    enable_after_fit: bool = False


class KinematicsCalibrationManualOffsetsRequest(BaseModel):
    reach_offset_mm: float = 0.0
    z_offset_mm: float = 0.0
    profile_key: str | None = None
    enabled: bool = False


class KinematicsCalibrationEnableRequest(BaseModel):
    enabled: bool
    profile_key: str | None = None


class PhysicalModelFitRequest(BaseModel):
    parameter_group: str = "joint_zeros"
    profile_key: str | None = None


class PhysicalModelApplyRequest(BaseModel):
    result_id: str
    profile_key: str | None = None
    confirm: bool = False


def public_config() -> dict[str, Any]:
    positions = named_positions(config)
    library = position_library_records(config, positions)
    destination_errors = task_destination_errors(config, positions)
    try:
        destinations = drop_zones(config)
    except TaskDestinationError:
        destinations = {}
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
        "position_library": {
            "schema_version": POSITION_LIBRARY_SCHEMA_VERSION,
            "positions": library,
        },
        "named_positions": positions,
        "camera": camera_settings(config),
        "color_profiles": color_profiles(config),
        "drop_zones": legacy_drop_zones_from_task_destinations(destinations),
        "task_destinations": {
            "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
            "destinations": destinations,
        },
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
        "model_truth": model_truth_summary(config),
        "geometry": geometry_settings(config),
        "validation": {
            "model_warnings": model_validation_warnings(config),
            "encoder_errors": validate_encoder_settings(config, encoder_settings(config)),
            "named_position_errors": named_position_errors(config),
            "position_library_errors": position_library_errors(config, library),
            "task_destination_errors": destination_errors,
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
    encoder_config = encoder_settings(config)
    errors.extend(validate_encoder_settings(config, encoder_config))

    pin_uses: dict[int, list[str]] = {}

    def register_pin(value: Any, label: str) -> None:
        try:
            if isinstance(value, bool):
                raise ValueError
            pin = int(value)
        except (TypeError, ValueError):
            errors.append(f"{label} must be an integer GPIO")
            return
        if pin < -1 or pin > 48:
            errors.append(f"{label} must be between -1 and 48")
            return
        if pin >= 0:
            pin_uses.setdefault(pin, []).append(label)

    for joint in config.joints:
        if joint.actuator == "stepper" and joint.hardware.stepper and joint.hardware.stepper.enabled:
            register_pin(joint.hardware.stepper.step_pin, f"{joint.name} STEP")
            register_pin(joint.hardware.stepper.dir_pin, f"{joint.name} DIR")
            register_pin(joint.hardware.stepper.enable_pin, f"{joint.name} ENABLE")
        elif joint.actuator == "servo" and joint.hardware.servo and joint.hardware.servo.enabled:
            register_pin(joint.hardware.servo.pwm_pin, f"{joint.name} PWM")

    tools = tools_settings(config)
    active_tool_name = str(tools.get("active", ""))
    active_tool = (
        tools.get("presets", {}).get(active_tool_name, {})
        if isinstance(tools.get("presets"), dict)
        else {}
    )
    if isinstance(active_tool, dict):
        tool_io = active_tool.get("io") if isinstance(active_tool.get("io"), dict) else {}
        if str(active_tool.get("type")) == "servo_gripper":
            register_pin(tool_io.get("pwm_pin", -1), f"tool {active_tool_name} PWM")
        elif str(active_tool.get("type")) == "electromagnet":
            register_pin(tool_io.get("pin", -1), f"tool {active_tool_name} GPIO")

    if bool(encoder_config.get("enabled")):
        bus = encoder_config.get("bus") if isinstance(encoder_config.get("bus"), dict) else {}
        register_pin(bus.get("sck_pin", -1), "encoder SPI SCK")
        register_pin(bus.get("miso_pin", -1), "encoder SPI MISO")
        register_pin(bus.get("mosi_pin", -1), "encoder SPI MOSI")
        raw_encoders = config.raw.get("encoders")
        raw_axes = raw_encoders.get("axes") if isinstance(raw_encoders, dict) else None
        if isinstance(raw_axes, list):
            for raw_axis in raw_axes:
                if not isinstance(raw_axis, dict) or not bool(raw_axis.get("enabled")):
                    continue
                try:
                    joint_number = int(raw_axis.get("joint", 0) or 0)
                except (TypeError, ValueError):
                    joint_number = 0
                joint_name = (
                    config.joints[joint_number - 1].name
                    if 1 <= joint_number <= len(config.joints)
                    else f"joint {joint_number}"
                )
                register_pin(
                    raw_axis.get("cs_pin", -1),
                    f"{joint_name} encoder CS",
                )
        else:
            shoulder_encoder = encoder_axis(encoder_config)
            if shoulder_encoder and bool(shoulder_encoder.get("enabled")):
                register_pin(shoulder_encoder.get("cs_pin", -1), "shoulder encoder CS")

    for pin, uses in sorted(pin_uses.items()):
        if len(uses) > 1:
            errors.append(f"GPIO {pin} conflict: {', '.join(uses)}")
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


def _tool_dimensions_signature(preset: dict[str, Any] | None) -> str:
    tool = preset if isinstance(preset, dict) else {}
    return stable_payload_fingerprint(
        {
            "type": tool.get("type", "generic"),
            "tcp_offset_mm": tool.get("tcp_offset_mm", {}),
        }
    )


def _invalidate_changed_tool_validations(
    previous_tools: dict[str, Any],
    next_tools: dict[str, Any],
) -> None:
    previous_presets = previous_tools.get("presets") if isinstance(previous_tools.get("presets"), dict) else {}
    next_presets = next_tools.get("presets") if isinstance(next_tools.get("presets"), dict) else {}
    for name, preset in next_presets.items():
        if not isinstance(preset, dict):
            continue
        previous = previous_presets.get(name) if isinstance(previous_presets.get(name), dict) else None
        if previous is None or _tool_dimensions_signature(previous) != _tool_dimensions_signature(preset):
            preset["dimensions_validated"] = False
            preset.pop("dimensions_validated_at", None)


def _active_tool_validation_from_payload(
    tools: dict[str, Any],
    fallback: bool = False,
) -> bool:
    active = str(tools.get("active", "gripper"))
    presets = tools.get("presets") if isinstance(tools.get("presets"), dict) else {}
    preset = presets.get(active) if isinstance(presets.get(active), dict) else {}
    if "dimensions_validated" in preset:
        return bool(preset.get("dimensions_validated"))
    return bool(fallback)


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


def _encoder_signatures(robot_config: RobotConfig) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    settings = normalize_encoder_settings(robot_config)
    axis = encoder_axis(settings) or {}
    io_signature = {
        "enabled": settings.get("enabled"),
        "bus": deepcopy(settings.get("bus", {})),
        "axis": {
            key: axis.get(key)
            for key in ["joint", "sensor", "enabled", "cs_pin"]
        },
    }
    calibration_signature = {
        key: axis.get(key)
        for key in [
            "reference_raw_deg",
            "reference_joint_deg",
            "direction_sign",
            "wrap_period_deg",
            "unwrap_policy",
            "mounting_location",
            "sensor_turns_per_joint_turn",
            "reference_description",
            "freshness_timeout_ms",
            "max_noise_deg",
            "calibration_validated",
            "calibration_id",
        ]
    }
    policy_signature = {
        "mode": settings.get("mode"),
        "verification": deepcopy(settings.get("verification", {})),
        "correction": deepcopy(settings.get("correction", {})),
    }
    return io_signature, calibration_signature, policy_signature


def classify_config_change(previous: RobotConfig, current: RobotConfig) -> dict[str, Any]:
    pose_mapping_changed = _joint_pose_mapping_signature(previous) != _joint_pose_mapping_signature(current)
    joint_controller_changed = _joint_controller_signature(previous) != _joint_controller_signature(current)
    io_changed = _joint_io_signature(previous) != _joint_io_signature(current)
    tool_controller_changed = _tool_controller_signature(previous) != _tool_controller_signature(current)
    previous_encoder_io, previous_encoder_calibration, previous_encoder_policy = _encoder_signatures(previous)
    current_encoder_io, current_encoder_calibration, current_encoder_policy = _encoder_signatures(current)
    encoder_io_changed = previous_encoder_io != current_encoder_io
    encoder_calibration_changed = previous_encoder_calibration != current_encoder_calibration
    encoder_policy_changed = previous_encoder_policy != current_encoder_policy
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
    encoder_controller_changed = encoder_io_changed or encoder_calibration_changed or encoder_policy_changed
    sync_required = joint_controller_changed or tool_controller_changed or encoder_controller_changed

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
    if encoder_io_changed:
        categories.append("encoder_io")
        reasons.append("encoder SPI or chip-select configuration changed")
    if encoder_calibration_changed:
        categories.append("encoder_calibration")
        reasons.append("encoder reference, direction, mounting, or validity limits changed")
    if encoder_policy_changed:
        categories.append("encoder_policy")
        reasons.append("encoder verification or correction policy changed")
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
        "encoder_measurement_invalidated": encoder_io_changed or encoder_calibration_changed,
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
        "tcp_speed_mm_s": 60.0,
        "phi_speed_deg_s": 45.0,
        "tcp_accel_mm_s2": 360.0,
        "phi_accel_deg_s2": 240.0,
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


def home_path_settings(settings_payload: PathSettingsRequest | dict[str, Any] | None = None) -> dict[str, Any]:
    settings = request_settings(settings_payload)
    settings["motion_purpose"] = "configured_home_pose_move"
    return settings


def request_settings(settings: PathSettingsRequest | dict[str, Any] | None) -> dict[str, Any]:
    merged = default_path_settings()
    if settings is None:
        return merged
    values = settings if isinstance(settings, dict) else settings.__dict__
    merged.update({key: value for key, value in values.items() if value is not None})
    return merged


def calibration_path_settings(settings: PathSettingsRequest | dict[str, Any] | None) -> dict[str, Any]:
    merged = request_settings(settings)
    merged["global_speed_deg_s"] = min(float(merged["global_speed_deg_s"]), 10.0)
    merged["global_accel_deg_s2"] = min(float(merged["global_accel_deg_s2"]), 20.0)
    merged["tcp_speed_mm_s"] = min(float(merged["tcp_speed_mm_s"]), 20.0)
    merged["tcp_accel_mm_s2"] = min(float(merged["tcp_accel_mm_s2"]), 60.0)
    merged["phi_speed_deg_s"] = min(float(merged["phi_speed_deg_s"]), 15.0)
    merged["phi_accel_deg_s2"] = min(float(merged["phi_accel_deg_s2"]), 45.0)
    merged["motion_purpose"] = "calibration_measurement_move"
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

    for key in (
        "global_speed_deg_s",
        "global_accel_deg_s2",
        "tcp_speed_mm_s",
        "phi_speed_deg_s",
        "tcp_accel_mm_s2",
        "phi_accel_deg_s2",
        "waypoint_rate_hz",
        "cartesian_step_mm",
    ):
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


def cartesian_jog_motion_contract(
    settings: dict[str, Any],
    *,
    tcp_speed_mm_s: float | None = None,
    phi_speed_deg_s: float | None = None,
) -> dict[str, Any]:
    joint_speed, joint_accel = _joint_limits_from_settings(settings)
    tcp_speed = max(1.0, float(tcp_speed_mm_s if tcp_speed_mm_s is not None else settings.get("tcp_speed_mm_s") or 60.0))
    phi_speed = max(1.0, float(phi_speed_deg_s if phi_speed_deg_s is not None else settings.get("phi_speed_deg_s") or 45.0))
    tcp_accel = max(1.0, float(settings.get("tcp_accel_mm_s2") or 360.0))
    phi_accel = max(1.0, float(settings.get("phi_accel_deg_s2") or 240.0))
    limits = {
        "schema": "motion_limit_summary_v1",
        "path_mode": "cartesian_jog",
        "target_type": "cartesian_velocity",
        "profile": "fixed_rate_servo",
        "duration_s": None,
        "limiting_constraint": {
            "type": "live_velocity",
            "joint_index": None,
            "joint_name": "",
            "duration_s": None,
        },
        "effective_joint_speed_deg_s": joint_speed,
        "effective_joint_accel_deg_s2": joint_accel,
        "tcp_speed_mm_s": tcp_speed,
        "phi_speed_deg_s": phi_speed,
        "tcp_accel_mm_s2": tcp_accel,
        "phi_accel_deg_s2": phi_accel,
        "notes": ["Cartesian jog is a fixed-rate velocity servo, not a precomputed trajectory"],
    }
    return {
        "schema": "motion_plan_contract_v1",
        "path_mode": "cartesian_jog",
        "target_type": "cartesian_velocity",
        "profile": "fixed_rate_servo",
        "interpolation": "fixed-rate Cartesian velocity servo with synchronized SERVOJ targets",
        "duration_s": None,
        "waypoint_count": 0,
        "limits": limits,
        "controller_command": controller_command_contract("SERVOJ", settings=settings),
    }


def controller_command_contract(command: str, *, settings: dict[str, Any] | None = None) -> dict[str, Any]:
    normalized = command.upper()
    settings = settings or {}
    if normalized == "MOVEJ":
        return {
            "schema": "controller_command_contract_v1",
            "command": "MOVEJ",
            "timing_authority": "controller_endpoint_profile",
            "uses_planned_timestamps": False,
            "speed_deg_s": settings.get("global_speed_deg_s"),
            "accel_deg_s2": settings.get("global_accel_deg_s2"),
            "notes": [
                "MOVEJ is a low-level endpoint command and does not receive planned waypoint timestamps",
                "firmware configuration and per-axis caps still apply on the controller",
                "per-joint override limits affect preview estimates but are not transmitted in MOVEJ",
            ],
        }
    if normalized == "TRAJ":
        return {
            "schema": "controller_command_contract_v1",
            "command": "TRAJ",
            "timing_authority": "pc_planned_timed_trajectory",
            "uses_planned_timestamps": True,
            "speed_deg_s": settings.get("global_speed_deg_s"),
            "accel_deg_s2": settings.get("global_accel_deg_s2"),
            "notes": [
                "queued trajectory uploads planned waypoint timestamps to the controller",
                "joint-space endpoints and Cartesian/program paths use the same timed upload path",
                "firmware still enforces configured hardware limits",
            ],
        }
    if normalized == "SERVOJ":
        return {
            "schema": "controller_command_contract_v1",
            "command": "SERVOJ",
            "timing_authority": "pc_fixed_rate_cartesian_servo",
            "uses_planned_timestamps": False,
            "notes": ["Cartesian jog sends finite synchronized SERVOJ targets at a fixed servo period"],
        }
    if normalized == "SIM_TRAJ":
        return {
            "schema": "controller_command_contract_v1",
            "command": "SIM_TRAJ",
            "timing_authority": "pc_simulation_timed_trajectory",
            "uses_planned_timestamps": True,
            "notes": ["simulation follows the planned trajectory timestamps and effective joint limits"],
        }
    return {
        "schema": "controller_command_contract_v1",
        "command": normalized,
        "timing_authority": "unknown",
        "uses_planned_timestamps": False,
        "notes": [],
    }


def motion_contract_for_controller(
    motion_contract: dict[str, Any] | None,
    command: str,
    settings: dict[str, Any] | None = None,
) -> dict[str, Any]:
    contract = deepcopy(motion_contract or {})
    contract["controller_command"] = controller_command_contract(command, settings=settings)
    return contract


def attach_controller_command_to_motion_contract(
    preview: dict[str, Any],
    command: str,
    settings: dict[str, Any] | None = None,
) -> None:
    contract = motion_contract_for_controller(preview.get("motion_contract"), command, settings)
    preview["motion_contract"] = contract
    preview["controller_command_contract"] = contract["controller_command"]


def anticipated_controller_command(trajectory_mode: str) -> str:
    if state.simulation:
        return "SIM_TRAJ"
    return "TRAJ"


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


ACTIVE_TASK_STATUSES = {
    "queued",
    "running",
    "capturing",
    "planning",
    "executing",
    "waiting_for_selection",
    "waiting_for_confirmation",
    "stopping",
}
TERMINAL_TASK_STATUSES = {"completed", "failed", "stopped"}
UNCERTAIN_HOLD_STATES = {"possibly_held", "confirmed_held", "release_unconfirmed"}


def update_task_execution(**updates: Any) -> dict[str, Any]:
    payload = dict(state.task_execution or {})
    payload.update(updates)
    payload["updated_at"] = time()
    state.task_execution = payload
    state.updated_at = time()
    return payload


def task_recovery_summary(execution: dict[str, Any] | None = None) -> dict[str, Any]:
    current = execution or state.task_execution or {}
    step = current.get("current_step") if isinstance(current.get("current_step"), dict) else {}
    hold_state = str(current.get("object_hold_state") or "none")
    retreat_target = step.get("recovery_target")
    safe_retreat_available = bool(
        step.get("safe_retreat_available")
        and isinstance(retreat_target, dict)
        and state.known_pose
        and state.motion_state not in {MotionState.ESTOP, MotionState.FAULT}
    )
    options = ["verify robot pose before any recovery motion"]
    if hold_state in UNCERTAIN_HOLD_STATES:
        options.append("inspect or secure the tool/object before moving")
        options.append("do not change the commanded tool state until object state is resolved")
    else:
        options.append("confirm that no object is held")
    if safe_retreat_available:
        options.append("preview and command the recorded clearance retreat")
    else:
        options.append("re-preview a safe recovery path; automatic retreat is unavailable")
    return {
        "object_hold_state": hold_state,
        "safe_retreat_available": safe_retreat_available,
        "retreat_target": deepcopy(retreat_target) if isinstance(retreat_target, dict) else None,
        "options": options,
    }


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
        "last_completed_step": None,
        "completed_count": 0,
        "remaining_count": max(0, total_objects),
        "total_count": max(0, total_objects),
        "latest_capture": None,
        "ignored_objects": [],
        "candidate_objects": [],
        "tool_feedback": {"available": False, "status": "not_implemented"},
        "object_hold_state": "none",
        "holding_uncertain": False,
        "safe_retreat_available": False,
        "recovery_options": [],
        "recovery": {},
        "warnings": [],
        "terminal_reason": None,
        "settings": settings,
        "started_at": time(),
        "updated_at": time(),
    }
    recovery = task_recovery_summary(state.task_execution)
    state.task_execution.update(
        recovery=recovery,
        safe_retreat_available=recovery["safe_retreat_available"],
        recovery_options=recovery["options"],
    )
    state.updated_at = time()


def finish_task_execution(
    status: str,
    reason: str,
    *,
    holding_uncertain: bool | None = None,
) -> None:
    if not state.task_execution:
        return
    if state.task_execution.get("status") in TERMINAL_TASK_STATUSES:
        return
    hold_state = str(state.task_execution.get("object_hold_state") or "none")
    uncertain = hold_state in UNCERTAIN_HOLD_STATES if holding_uncertain is None else holding_uncertain
    recovery = task_recovery_summary()
    update_task_execution(
        status=status,
        phase=status,
        terminal_reason=reason,
        holding_uncertain=uncertain,
        safe_retreat_available=recovery["safe_retreat_available"],
        recovery_options=recovery["options"],
        recovery=recovery,
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
        reason = hardware_trajectory_start_blocking_reason()
        if reason:
            return reason
        tool_errors = active_tool_hardware_errors()
        if tool_errors:
            return "; ".join(tool_errors)
        if not active_tool_dimensions_validated(config):
            return "task execution requires validated active-tool dimensions"
    can_move = validate_can_move(state)
    if not can_move.ok:
        return can_move.reason
    return None


def resume_fresh_task_motion_from_stopped() -> bool:
    execution = state.task_execution or {}
    if state.motion_state != MotionState.STOPPED:
        return False
    if int(execution.get("completed_count", 0) or 0) != 0 or execution.get("last_completed_step"):
        return False
    state.motion_state = MotionState.IDLE
    log_event(
        "task",
        "fresh task motion resumed from controller stopped state",
        run_id=execution.get("run_id"),
    )
    return True


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


def update_encoder_evidence(status: Any) -> None:
    settings = encoder_settings(config)
    shoulder = encoder_axis(settings)
    evidence = [
        empty_evidence(index + 1, joint.name)
        for index, joint in enumerate(config.joints)
    ]
    if shoulder is not None:
        index = 1
        raw_angles = status.encoder_raw_angles_deg or [None] * len(config.joints)
        raw_counts = status.encoder_raw_counts or [None] * len(config.joints)
        measured_angles = status.encoder_measured_angles_deg or [None] * len(config.joints)
        ages = status.encoder_age_ms or [None] * len(config.joints)
        noise = status.encoder_noise_deg or [None] * len(config.joints)
        valid_counts = (
            status.encoder_consecutive_valid_samples
            or [None] * len(config.joints)
        )
        flags = status.encoder_flags or [[] for _ in config.joints]
        sensor_available = (
            index < len(status.encoder_available)
            and status.encoder_available[index] == "1"
        )
        raw_angle = raw_angles[index] if sensor_available else None
        raw_count = raw_counts[index] if sensor_available else None
        firmware_measured = measured_angles[index] if sensor_available else None
        measured = None
        if raw_angle is not None:
            try:
                measured = calibrated_joint_angle(float(raw_angle), shoulder)
            except (TypeError, ValueError):
                measured = None
        elif sensor_available:
            measured = firmware_measured
        sensor_valid = (
            sensor_available
            and index < len(status.encoder_valid)
            and status.encoder_valid[index] == "1"
        )
        required_health_samples = max(
            1,
            int(settings.get("verification", {}).get("required_stable_samples", 3)),
        )
        valid_count = valid_counts[index]
        consecutive_health_valid = bool(
            sensor_valid
            and (
                valid_count is None
                or int(valid_count) >= required_health_samples
            )
        )
        calibrated_valid = bool(
            settings.get("enabled")
            and shoulder.get("enabled")
            and shoulder.get("calibration_validated")
            and shoulder.get("mounting_location") == "joint_output"
            and consecutive_health_valid
            and measured is not None
        )
        evidence_flags = list(flags[index])
        if sensor_valid and not shoulder.get("calibration_validated"):
            evidence_flags.append("uncalibrated")
        if sensor_valid and not consecutive_health_valid:
            evidence_flags.append("warming_up")
        if sensor_valid and shoulder.get("mounting_location") != "joint_output":
            evidence_flags.append("relative_only_mounting")
        evidence[index] = evidence_from_status(
            joint_number=2,
            name="shoulder",
            raw_count=raw_count,
            raw_angle_deg=raw_angle,
            measured_angle_deg=measured,
            valid=calibrated_valid,
            age_ms=ages[index],
            noise_deg=noise[index],
            flags=evidence_flags,
            freshness_timeout_ms=float(shoulder.get("freshness_timeout_ms", 500)),
            estimated_angle_deg=float(state.estimated_angles_deg[index]),
        )
        evidence[index]["sensor_valid"] = sensor_valid
        evidence[index]["sensor_available"] = sensor_available
        evidence[index]["consecutive_valid_samples"] = valid_count
        evidence[index]["required_health_samples"] = required_health_samples
        evidence[index]["calibration_validated"] = bool(shoulder.get("calibration_validated"))
        evidence[index]["mounting_location"] = shoulder.get("mounting_location")

    state.update_encoder_evidence(evidence)
    state.encoder_angles_deg = [
        item.get("measured_angle_deg")
        for item in evidence
    ]
    state.encoder_errors_deg = [
        item.get("mismatch_deg")
        for item in evidence
    ]


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


def controller_estimated_angles_from_last_status(max_age_s: float | None = None) -> list[float] | None:
    raw = state.encoder_mismatch.get("last_controller_estimated_deg")
    if isinstance(raw, list) and len(raw) == len(state.reported_angles_deg):
        try:
            if max_age_s is not None:
                updated_at = state.encoder_mismatch.get("last_controller_estimated_at")
                if updated_at is None or time() - float(updated_at) > float(max_age_s):
                    return None
            return [float(value) for value in raw]
        except (TypeError, ValueError):
            pass
    if max_age_s is not None:
        return None
    return [float(value) for value in state.reported_angles_deg]


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
    motion_contract: dict[str, Any] | None = None,
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
        "motion_contract": motion_contract or {},
        "limit_summary": (motion_contract or {}).get("limits", {}),
        "execution_state": "queued",
        "requested_target_deg": [float(value) for value in target_deg],
        "start_reported_deg": [float(value) for value in state.reported_angles_deg],
        "start_estimated_deg": [float(value) for value in state.estimated_angles_deg],
        "start_measured_deg": list(state.measured_angles_deg),
        "start_measurement_valid_mask": state.measurement_valid_mask,
        "start_joint_authority": list(state.joint_authority),
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
        "motion_contract": motion_contract or {},
        "limit_summary": (motion_contract or {}).get("limits", {}),
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
        settings = encoder_settings(config)
        shoulder = encoder_axis(settings)
        if (
            not state.simulation
            and settings.get("enabled")
            and shoulder
            and shoulder.get("enabled")
        ):
            update_motion_diagnostics(
                active_motion_run_id,
                execution_state="settling_verification",
                result="executing",
            )
            return
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
            "final_estimated_deg": [float(value) for value in state.estimated_angles_deg],
            "final_measured_deg": list(state.measured_angles_deg),
            "final_measurement_valid_mask": state.measurement_valid_mask,
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


def send_correctj_and_read_response(command: str) -> str:
    serial_client.clear_input()
    serial_client.send_line(command)
    response = read_serial_until_any(("OK command=CORRECTJ", "ERR"), timeout_s=1.0)
    state.last_controller_response = response
    if response.startswith("ERR"):
        raise SerialClientError(response)
    return response


def send_alignj_and_read_response(command: str) -> str:
    serial_client.clear_input()
    serial_client.send_line(command)
    response = read_serial_until_any(("OK command=ALIGNJ", "ERR"), timeout_s=1.0)
    state.last_controller_response = response
    if response.startswith("ERR"):
        raise SerialClientError(response)
    return response


def send_setpose_and_read_response(angles_deg: list[float]) -> str:
    serial_client.clear_input()
    serial_client.send_line(format_setpose([float(value) for value in angles_deg]))
    response = read_serial_until_any(("OK command=SETPOSE", "ERR"), timeout_s=1.0)
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
    last_controller_error: list[float] = []
    last_planning_error: list[float] = []
    while monotonic() < deadline:
        if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
            return False, f"motion stopped while waiting for hardware ({state.motion_state.value})"
        try:
            refresh_serial_status()
        except SerialClientError as exc:
            return False, str(exc)
        controller_estimate = controller_estimated_angles_from_last_status(max_age_s=2.0)
        if controller_estimate is None:
            return False, "controller status did not include fresh estimated joint angles"
        last_controller_error = joint_errors_deg(target_deg, controller_estimate)
        last_planning_error = joint_errors_deg(target_deg, state.reported_angles_deg)
        if state.motion_state == MotionState.IDLE and all(abs(error) <= tolerance_deg for error in last_controller_error):
            planning_error_text = ", ".join(f"{value:.2f}" for value in last_planning_error)
            return True, f"controller target reached; planning/encoder residual deg=[{planning_error_text}]"
        await asyncio.sleep(poll_interval_s)
    controller_error_text = (
        ", ".join(f"{value:.2f}" for value in last_controller_error)
        if last_controller_error
        else "unknown"
    )
    planning_error_text = (
        ", ".join(f"{value:.2f}" for value in last_planning_error)
        if last_planning_error
        else "unknown"
    )
    return (
        False,
        f"hardware target timeout after {timeout_s:.2f}s; "
        f"controller errors deg=[{controller_error_text}], planning/encoder errors deg=[{planning_error_text}]",
    )


def _shoulder_evidence() -> dict[str, Any]:
    if len(state.encoder_evidence) < 2:
        return empty_evidence(2, "shoulder")
    return state.encoder_evidence[1]


def latch_encoder_mismatch_fault(message: str, error_deg: float | None) -> None:
    state.encoder_fault = True
    state.encoder_mismatch = {
        **state.encoder_mismatch,
        "status": "fault",
        "error_deg": error_deg,
        "message": message,
        "latched_at": time(),
        "requires_setpose": True,
    }
    known_mask = state.pose_known_mask.ljust(len(config.joints), "0")
    if len(known_mask) >= 2:
        known_mask = f"{known_mask[0]}0{known_mask[2:]}"
    state.update_reported_pose(
        state.reported_angles_deg,
        source=state.pose_source,
        known_mask=known_mask,
        force_revision=True,
    )
    state.target_angles_deg = state.reported_angles_deg.copy()
    limiter.set_target(state.target_angles_deg)
    state.set_error(message, fault=True)
    if serial_client.is_connected and not state.simulation:
        try:
            serial_client.send_line(format_stop())
        except Exception as exc:
            log_event("encoder", "could not send STOP after encoder fault", error=str(exc))
    log_event("encoder", message, error_deg=error_deg, severity="fault")


def shoulder_controller_rebase_applicable() -> bool:
    if state.simulation or len(config.joints) < 2:
        return False
    shoulder = config.joints[1]
    return bool(
        shoulder.actuator == "stepper"
        and shoulder.hardware.stepper
        and shoulder.hardware.stepper.enabled
    )


def update_controller_rebase_state(
    *,
    required: bool,
    controller_deg: float | None = None,
    tracked_deg: float | None = None,
    delta_deg: float | None = None,
    reason: str = "",
) -> None:
    state.encoder_mismatch = {
        **state.encoder_mismatch,
        "controller_pose_rebase_required": bool(required),
        "controller_pose_rebase_reason": reason if required else "",
        "controller_pose_rebase_controller_shoulder_deg": controller_deg,
        "controller_pose_rebase_tracked_shoulder_deg": tracked_deg,
        "controller_pose_rebase_delta_deg": delta_deg,
        "controller_pose_rebase_checked_at": time(),
    }


def controller_pose_rebase_blocking_reason() -> str | None:
    if not state.encoder_mismatch.get("controller_pose_rebase_required"):
        return None
    delta = state.encoder_mismatch.get("controller_pose_rebase_delta_deg")
    delta_text = ""
    try:
        delta_text = f" ({float(delta):+.2f} deg)"
    except (TypeError, ValueError):
        pass
    return (
        "shoulder encoder updated the planning pose but the controller step position is not synced"
        f"{delta_text}; disarm and arm again to rebase the controller before moving"
    )


def sync_controller_pose_to_encoder_tracked_pose_if_needed() -> tuple[bool, str]:
    if not state.encoder_mismatch.get("controller_pose_rebase_required"):
        return True, ""
    if not shoulder_controller_rebase_applicable():
        update_controller_rebase_state(required=False)
        return True, ""
    if state.hardware_armed:
        return False, controller_pose_rebase_blocking_reason() or "controller pose rebase requires disarmed hardware"
    if state.motion_state not in {MotionState.IDLE, MotionState.STOPPED}:
        return False, "controller pose rebase requires idle/stopped hardware"
    result = validate_joint_targets(config, state.reported_angles_deg)
    if not result.ok:
        return False, f"cannot rebase controller pose: {result.reason}"
    send_setpose_and_read_response(state.reported_angles_deg)
    refresh_serial_status()
    align_target_to_reported()
    update_controller_rebase_state(required=False)
    log_event(
        "encoder",
        "controller pose rebased to encoder-tracked shoulder",
        angles_deg=state.reported_angles_deg,
    )
    return True, ""


def correction_motion_timeout_s(delta_deg: float, speed_deg_s: float, accel_deg_s2: float) -> float:
    """Estimate a safe PC-side wait for firmware CORRECTJ to finish."""
    distance = abs(float(delta_deg))
    speed = max(0.001, abs(float(speed_deg_s)))
    accel = max(0.001, abs(float(accel_deg_s2)))
    if distance <= 0.0001:
        duration = 0.0
    else:
        ramp_distance = (speed * speed) / (2.0 * accel)
        if distance <= 2.0 * ramp_distance:
            duration = 2.0 * sqrt(distance / accel)
        else:
            duration = 2.0 * speed / accel + (distance - 2.0 * ramp_distance) / speed
    return min(30.0, max(3.0, duration * 2.0 + 2.0))


def controller_rebase_tolerance_deg() -> float:
    tolerance = CONTROLLER_REBASE_TOLERANCE_DEG
    correction = encoder_settings(config).get("correction", {})
    if isinstance(correction, dict) and bool(correction.get("enabled")):
        try:
            tolerance = max(tolerance, float(correction.get("deadband_deg", 0.75)))
        except (TypeError, ValueError):
            tolerance = max(tolerance, 0.75)
    return tolerance


def should_hold_pose_tracking_after_correction(_delta_deg: float, _min_delta_deg: float) -> bool:
    correction = encoder_settings(config).get("correction", {})
    if not isinstance(correction, dict) or not bool(correction.get("enabled")):
        return False
    correction_state = str(state.correction_state.get("state") or "").strip().lower()
    transaction_id = str(state.correction_state.get("transaction_id") or "").strip().lower()
    bias = state.correction_state.get("bias_deg")
    shoulder_bias = 0.0
    if isinstance(bias, list) and len(bias) > 1 and bias[1] is not None:
        try:
            shoulder_bias = float(bias[1])
        except (TypeError, ValueError):
            shoulder_bias = 0.0
    correction_owns_pose = (
        abs(shoulder_bias) > 1e-6
        or correction_state in {"executing", "aligning"}
        or (correction_state == "completed" and transaction_id not in {"", "none"})
    )
    return correction_owns_pose


def _shoulder_alignment_target_deg(explicit_target_deg: float | None = None) -> float | None:
    if explicit_target_deg is not None:
        try:
            return float(explicit_target_deg)
        except (TypeError, ValueError):
            return None
    reference = state.target_angles_deg if len(state.target_angles_deg) > 1 else state.reported_angles_deg
    if len(reference) <= 1:
        return None
    try:
        return float(reference[1])
    except (TypeError, ValueError):
        return None


def shoulder_alignment_motion_blocking_reason(target_shoulder_deg: float | None = None) -> str | None:
    if state.simulation:
        return None
    settings = encoder_settings(config)
    tracking = settings.get("pose_tracking") if isinstance(settings.get("pose_tracking"), dict) else {}
    if bool(tracking.get("enabled")) and state.encoder_mismatch.get("pose_tracking_status") == "skipped":
        return str(
            state.encoder_mismatch.get("pose_tracking_skip_reason")
            or "shoulder encoder pose tracking was skipped; verify calibration before moving"
        )
    shoulder = encoder_axis(settings)
    if not settings.get("enabled") or not shoulder or not shoulder.get("enabled"):
        return None
    if not shoulder.get("calibration_validated"):
        return None
    if str(shoulder.get("mounting_location", "")) != "joint_output":
        return None
    correction = settings.get("correction", {}) if isinstance(settings.get("correction"), dict) else {}
    verification = settings.get("verification", {}) if isinstance(settings.get("verification"), dict) else {}
    policy = str(verification.get("policy", settings.get("mode", "diagnostic"))).strip().lower()
    correction_enabled = bool(correction.get("enabled"))
    if not correction_enabled and policy not in {"warning", "fault"}:
        return None
    evidence = _shoulder_evidence()
    measured = evidence.get("measured_angle_deg")
    noise = evidence.get("noise_deg")
    if not evidence.get("fresh") or measured is None:
        return None
    try:
        measured_deg = float(measured)
        noise_deg = float(noise) if noise is not None else 0.0
        target_deg = _shoulder_alignment_target_deg(target_shoulder_deg)
    except (TypeError, ValueError):
        return None
    if target_deg is None:
        return None
    max_noise = float(shoulder.get("max_noise_deg", 0.5))
    if noise_deg > max_noise:
        return None
    deadband = float(correction.get("deadband_deg", 0.75))
    warn = float(verification.get("warning_tolerance_deg", verification.get("warn_tolerance_deg", 2.0)))
    threshold = max(0.1, deadband if correction_enabled else warn)
    error = measured_deg - target_deg
    if abs(error) <= threshold:
        return None
    align_limit = max(
        float(correction.get("max_delta_deg", 8.0)),
        float(correction.get("align_max_delta_deg", correction.get("max_delta_deg", 8.0))),
    )
    if correction_enabled and "encoder_shoulder_align" in {str(value) for value in correction.get("allowed_sources", [])}:
        if abs(error) <= align_limit:
            return (
                f"shoulder is {error:+.2f} deg from the planning target; press Align before normal hardware moves"
            )
        return (
            f"shoulder is {error:+.2f} deg from the planning target, beyond the Align cap "
            f"{align_limit:.2f} deg; check calibration/mechanics before moving"
        )
    return (
        f"shoulder encoder disagrees with the planning target by {error:+.2f} deg; "
        "normal hardware motion is blocked by the encoder verification policy"
    )


def apply_shoulder_encoder_pose_tracking(status_state: str | None = None) -> bool:
    if state.simulation or state.encoder_fault:
        return False
    settings = encoder_settings(config)
    tracking = settings.get("pose_tracking") if isinstance(settings.get("pose_tracking"), dict) else {}
    if not bool(tracking.get("enabled")):
        return False
    shoulder = encoder_axis(settings)
    if (
        not settings.get("enabled")
        or not shoulder
        or not shoulder.get("enabled")
        or not shoulder.get("calibration_validated")
        or str(shoulder.get("mounting_location")) != "joint_output"
    ):
        return False
    normalized_state = str(status_state or state.motion_state.value).strip().lower()
    if normalized_state not in {"idle", "stopped"}:
        return False
    if str(tracking.get("mode") or "idle") == "disarmed_idle" and state.hardware_armed:
        return False
    evidence = _shoulder_evidence()
    measured = evidence.get("measured_angle_deg")
    noise = evidence.get("noise_deg")
    if not evidence.get("fresh") or measured is None:
        return False
    try:
        measured_deg = float(measured)
        noise_deg = float(noise) if noise is not None else 0.0
    except (TypeError, ValueError):
        return False
    if not isfinite(measured_deg) or not isfinite(noise_deg):
        return False
    if noise_deg > float(shoulder.get("max_noise_deg", 0.5)):
        return False
    shoulder_joint = config.joints[1]
    if not shoulder_joint.min_deg <= measured_deg <= shoulder_joint.max_deg:
        return False
    current = float(state.reported_angles_deg[1])
    delta = measured_deg - current
    min_delta = max(0.0, float(tracking.get("min_update_delta_deg", 0.10)))
    rebase_tolerance = max(min_delta, controller_rebase_tolerance_deg())
    if should_hold_pose_tracking_after_correction(delta, min_delta):
        state.encoder_mismatch = {
            **state.encoder_mismatch,
            "pose_tracking_status": "held_by_correction_bias",
            "pose_tracking_skip_reason": (
                f"controller correction bias owns the shoulder logical pose; "
                f"encoder residual is {delta:+.2f} deg"
            ),
            "pose_tracking_measured_deg": measured_deg,
            "pose_tracking_previous_deg": current,
            "pose_tracking_delta_deg": delta,
            "tracked_at": time(),
        }
        if shoulder_controller_rebase_applicable():
            update_controller_rebase_state(required=False)
        return False
    if abs(delta) < min_delta:
        if shoulder_controller_rebase_applicable() and abs(delta) <= rebase_tolerance:
            update_controller_rebase_state(required=False)
        return False
    max_jump = max(min_delta, float(tracking.get("max_jump_deg", 180.0)))
    if abs(delta) > max_jump:
        state.encoder_mismatch = {
            **state.encoder_mismatch,
            "pose_tracking_status": "skipped",
            "pose_tracking_skip_reason": (
                f"shoulder encoder jump {delta:+.2f} deg exceeds pose tracking max jump {max_jump:.2f} deg"
            ),
            "pose_tracking_measured_deg": measured_deg,
            "pose_tracking_previous_deg": current,
            "checked_at": time(),
        }
        return False

    tracked = [float(value) for value in state.reported_angles_deg]
    tracked[1] = measured_deg
    known_mask = state.pose_known_mask.ljust(len(config.joints), "0")
    if bool(tracking.get("set_shoulder_known", True)) and len(known_mask) >= 2:
        known_mask = f"{known_mask[0]}1{known_mask[2:]}"
    revised = state.update_reported_pose(
        tracked,
        source="encoder_shoulder_tracking",
        known_mask=known_mask,
        force_revision=True,
        tolerance_deg=min_delta,
    )
    if len(state.joint_authority) > 1:
        state.joint_authority[1] = "measured"
    if len(state.encoder_evidence) > 1:
        state.encoder_evidence[1]["mismatch_deg"] = 0.0
    if len(state.encoder_errors_deg) > 1:
        state.encoder_errors_deg[1] = 0.0
    target = [float(value) for value in state.target_angles_deg]
    if len(target) > 1:
        target[1] = measured_deg
        state.target_angles_deg = target
        limiter.current_deg = tracked.copy()
        limiter.set_target(target)
    state.encoder_mismatch = {
        **state.encoder_mismatch,
        "pose_tracking_status": "applied",
        "pose_tracking_measured_deg": measured_deg,
        "pose_tracking_previous_deg": current,
        "pose_tracking_delta_deg": delta,
        "pose_tracking_revised": revised,
        "tracked_at": time(),
    }
    if shoulder_controller_rebase_applicable():
        rebase_required = abs(delta) > rebase_tolerance
        update_controller_rebase_state(
            required=rebase_required,
            controller_deg=current,
            tracked_deg=measured_deg,
            delta_deg=delta,
            reason=(
                "encoder_tracked_shoulder_differs_from_controller_step_position"
                if rebase_required
                else ""
            ),
        )
    log_event(
        "encoder",
        "shoulder pose tracked from encoder",
        previous_deg=current,
        measured_deg=measured_deg,
        delta_deg=delta,
    )
    return True


async def _stable_shoulder_measurement(required_samples: int) -> tuple[float | None, str]:
    settings = encoder_settings(config)
    shoulder = encoder_axis(settings)
    if not shoulder:
        return None, "shoulder encoder is not configured"
    samples: list[float] = []
    max_attempts = max(required_samples * 3, required_samples)
    sample_interval_s = max(
        0.03,
        float(settings.get("bus", {}).get("sample_interval_ms", 100)) / 1000.0,
    )
    for _ in range(max_attempts):
        try:
            refresh_serial_status()
        except SerialClientError as exc:
            return None, str(exc)
        evidence = _shoulder_evidence()
        measured = evidence.get("measured_angle_deg")
        noise = evidence.get("noise_deg")
        if (
            evidence.get("fresh")
            and measured is not None
            and (noise is None or float(noise) <= float(shoulder.get("max_noise_deg", 0.5)))
        ):
            samples.append(float(measured))
            if len(samples) >= required_samples:
                recent = samples[-required_samples:]
                if max(recent) - min(recent) <= float(shoulder.get("max_noise_deg", 0.5)):
                    ordered = sorted(recent)
                    return ordered[len(ordered) // 2], "stable"
                samples.clear()
        await asyncio.sleep(sample_interval_s)
    evidence = _shoulder_evidence()
    return None, f"shoulder encoder is {evidence.get('health', 'unavailable')} or noisy"


async def verify_shoulder_after_motion(
    source: str,
    target_deg: list[float],
    *,
    allow_correction: bool = True,
) -> tuple[bool, str]:
    if source in {"encoder_calibration_sweep", "encoder_backlash_check"}:
        return True, "encoder verification skipped during encoder calibration helper motion"

    settings = encoder_settings(config)
    shoulder = encoder_axis(settings)
    if (
        state.simulation
        or not settings.get("enabled")
        or not shoulder
        or not shoulder.get("enabled")
    ):
        return True, "encoder verification disabled"

    verification = settings.get("verification", {})
    policy = str(verification.get("policy", "diagnostic"))
    await asyncio.sleep(max(0.0, float(verification.get("settle_delay_ms", 300)) / 1000.0))
    measured, reason = await _stable_shoulder_measurement(
        max(1, int(verification.get("required_stable_samples", 3)))
    )
    if measured is None:
        state.encoder_mismatch = {
            "status": "unavailable",
            "source": source,
            "message": reason,
            "checked_at": time(),
        }
        if bool(verification.get("require_encoder")) and policy == "fault":
            message = f"required shoulder encoder unavailable after motion: {reason}"
            latch_encoder_mismatch_fault(message, None)
            return False, message
        log_event(
            "encoder",
            "post-move shoulder verification unavailable",
            motion_source=source,
            reason=reason,
        )
        return True, reason

    commanded_reference = float(target_deg[1])
    estimated_reference = float(state.estimated_angles_deg[1])
    estimated_error = float(measured) - estimated_reference
    commanded_error = float(measured) - commanded_reference
    # Encoder verification answers a physical question: did the shoulder end up
    # where the completed command asked it to end up?  The open-loop estimate is
    # still useful diagnostic evidence, but it must not be treated as the truth
    # when deciding mismatch/correction; backlash and lost steps are exactly the
    # cases where the estimate is expected to disagree.
    error = commanded_error
    warning_tolerance = float(verification.get("warning_tolerance_deg", 2.0))
    fault_tolerance = float(verification.get("fault_tolerance_deg", 5.0))
    hysteresis = max(0.0, float(verification.get("hysteresis_deg", 0.25)))
    previous_status = str(state.encoder_mismatch.get("status", ""))
    status = "ok"
    if abs(error) > fault_tolerance or (
        previous_status in {"fault_threshold", "fault"}
        and abs(error) > max(0.0, fault_tolerance - hysteresis)
    ):
        status = "fault_threshold"
    elif abs(error) > warning_tolerance or (
        previous_status == "warning"
        and abs(error) > max(0.0, warning_tolerance - hysteresis)
    ):
        status = "warning"
    state.encoder_mismatch = {
        "status": status,
        "source": source,
        "estimated_deg": estimated_reference,
        "commanded_deg": commanded_reference,
        "measured_deg": measured,
        "error_deg": error,
        "estimated_error_deg": estimated_error,
        "commanded_error_deg": commanded_error,
        "warning_tolerance_deg": warning_tolerance,
        "fault_tolerance_deg": fault_tolerance,
        "checked_at": time(),
    }
    log_event(
        "encoder",
        "post-move shoulder verification",
        motion_source=source,
        status=status,
        commanded_deg=commanded_reference,
        estimated_deg=estimated_reference,
        measured_deg=measured,
        error_deg=error,
        estimated_error_deg=estimated_error,
    )

    correction = settings.get("correction", {})
    allowed_sources = {str(value) for value in correction.get("allowed_sources", [])}
    correction_delta = -error
    correction_bias = 0.0
    reported_bias = state.correction_state.get("bias_deg")
    if isinstance(reported_bias, list) and len(reported_bias) > 1 and reported_bias[1] is not None:
        correction_bias = float(reported_bias[1])
    try:
        correction_deadband = max(0.0, float(correction.get("deadband_deg", 0.75)))
    except (TypeError, ValueError):
        correction_deadband = 0.75
    try:
        correction_limit = float(correction.get("max_delta_deg", 1.0))
    except (TypeError, ValueError):
        correction_limit = 1.0
    limit_margin = max(0.0, float(correction.get("joint_limit_margin_deg", 2.0)))
    corrected_physical_angle = commanded_reference + correction_bias + correction_delta
    shoulder_joint = config.joints[1]
    shoulder_stepper = shoulder_joint.hardware.stepper
    correction_skip_reason = ""
    if not allow_correction:
        correction_skip_reason = "correction disabled for this motion path"
    elif not correction.get("enabled"):
        correction_skip_reason = "bounded shoulder correction is disabled"
    elif source not in allowed_sources:
        correction_skip_reason = f"motion source {source} is not allowed to run shoulder correction"
    elif abs(error) <= correction_deadband:
        correction_skip_reason = (
            f"shoulder error {abs(error):.2f} deg is within correction deadband "
            f"{correction_deadband:.2f} deg"
        )
    elif abs(error) > correction_limit:
        correction_skip_reason = (
            f"shoulder error {abs(error):.2f} deg exceeds correction max delta "
            f"{correction_limit:.2f} deg"
        )
    elif shoulder_joint.actuator != "stepper" or not shoulder_stepper or not shoulder_stepper.enabled:
        correction_skip_reason = "shoulder correction requires an enabled hardware stepper"
    elif len(state.hardware_axis_states) <= 1 or state.hardware_axis_states[1] != "hardware":
        correction_skip_reason = "shoulder axis is not in hardware mode"
    elif not (
        shoulder_joint.min_deg + limit_margin <= corrected_physical_angle
        and corrected_physical_angle <= shoulder_joint.max_deg - limit_margin
    ):
        correction_skip_reason = "shoulder correction would cross the configured joint-limit margin"
    elif not state.known_pose:
        correction_skip_reason = "Set Pose first; correction requires a known planning pose"
    elif not state.hardware_armed:
        correction_skip_reason = "hardware must stay armed for post-move correction"
    elif state.motion_state != MotionState.IDLE:
        correction_skip_reason = "robot must be idle before post-move correction"
    elif cartesian_jog_runtime.get("active"):
        correction_skip_reason = "Cartesian jog is active; correction is verification-only"

    state.encoder_mismatch = {
        **state.encoder_mismatch,
        "correction_enabled": bool(correction.get("enabled")),
        "correction_deadband_deg": correction_deadband,
        "correction_max_delta_deg": correction_limit,
        "correction_allowed_sources": sorted(allowed_sources),
        "correction_would_delta_deg": correction_delta,
        "correction_skip_reason": correction_skip_reason,
        "correction_status": (
            "eligible" if not correction_skip_reason else (
                "not_needed" if "within correction deadband" in correction_skip_reason else "skipped"
            )
        ),
    }
    if correction_skip_reason:
        log_event(
            "encoder",
            "post-move shoulder correction skipped",
            motion_source=source,
            reason=correction_skip_reason,
            error_deg=error,
            deadband_deg=correction_deadband,
            max_delta_deg=correction_limit,
        )
        recoverable_skip = (
            allow_correction
            and bool(correction.get("enabled"))
            and source in allowed_sources
            and abs(error) > correction_deadband
        )
        if recoverable_skip:
            recovered, recovery_message = await ensure_shoulder_alignment_before_motion(
                source,
                target_shoulder_deg=commanded_reference,
            )
            if recovered:
                return True, "corrected"
            message = recovery_message or (
                f"automatic shoulder alignment could not recover {error:.2f} deg mismatch"
            )
            if not state.encoder_fault:
                latch_encoder_mismatch_fault(message, error)
            return False, message
        if abs(error) > fault_tolerance and policy == "fault":
            message = f"shoulder encoder mismatch {error:.2f} deg exceeds {fault_tolerance:.2f} deg"
            latch_encoder_mismatch_fault(message, error)
            return False, message
        return True, status

    max_attempts = max(1, int(correction.get("max_attempts", 2)))
    transaction_id = str(uuid4())
    for attempt in range(1, max_attempts + 1):
        delta = -error
        candidate_angle = float(measured) + delta
        if not (
            shoulder_joint.min_deg + limit_margin
            <= candidate_angle
            <= shoulder_joint.max_deg - limit_margin
        ):
            message = "shoulder correction would cross the configured joint-limit margin"
            latch_encoder_mismatch_fault(message, error)
            return False, message
        correction_speed = float(correction.get("speed_deg_s", 2.0))
        correction_accel = float(correction.get("accel_deg_s2", 10.0))
        correction_timeout_s = correction_motion_timeout_s(delta, correction_speed, correction_accel)
        command = format_correctj(
            2,
            delta,
            correction_speed,
            correction_accel,
            transaction_id,
        )
        state.correction_state = {
            "state": "executing",
            "transaction_id": transaction_id,
            "attempt": attempt,
            "requested_delta_deg": delta,
            "speed_deg_s": correction_speed,
            "accel_deg_s2": correction_accel,
            "timeout_s": correction_timeout_s,
            "source": source,
        }
        state.last_command = command
        try:
            send_correctj_and_read_response(command)
        except SerialClientError as exc:
            message = f"shoulder correction command failed: {exc}"
            latch_encoder_mismatch_fault(message, error)
            return False, message

        deadline = monotonic() + correction_timeout_s
        while monotonic() < deadline:
            await asyncio.sleep(0.08)
            try:
                refresh_serial_status()
            except SerialClientError as exc:
                message = f"shoulder correction status failed: {exc}"
                latch_encoder_mismatch_fault(message, error)
                return False, message
            if state.motion_state in {MotionState.IDLE, MotionState.STOPPED}:
                break
            if state.motion_state in {MotionState.FAULT, MotionState.ESTOP}:
                message = f"shoulder correction interrupted: {state.motion_state.value}"
                latch_encoder_mismatch_fault(message, error)
                return False, message
        else:
            message = (
                f"shoulder correction timed out after {correction_timeout_s:.1f}s "
                f"for {delta:.2f} deg at {correction_speed:.2f} deg/s"
            )
            latch_encoder_mismatch_fault(message, error)
            return False, message

        await asyncio.sleep(max(0.0, float(verification.get("settle_delay_ms", 300)) / 1000.0))
        measured, reason = await _stable_shoulder_measurement(
            max(1, int(verification.get("required_stable_samples", 3)))
        )
        if measured is None:
            message = f"shoulder correction lost encoder authority: {reason}"
            latch_encoder_mismatch_fault(message, None)
            return False, message
        estimated_error = float(measured) - estimated_reference
        commanded_error = float(measured) - commanded_reference
        error = commanded_error
        state.encoder_mismatch = {
            **state.encoder_mismatch,
            "status": "corrected" if abs(error) <= correction_deadband else "correction_pending",
            "measured_deg": measured,
            "error_deg": error,
            "estimated_error_deg": estimated_error,
            "commanded_error_deg": commanded_error,
            "correction_attempt": attempt,
            "correction_status": "completed" if abs(error) <= correction_deadband else "executing",
            "correction_skip_reason": "",
            "corrected_at": time(),
        }
        if abs(error) <= correction_deadband:
            state.correction_state = {
                **state.correction_state,
                "state": "completed",
                "final_error_deg": error,
            }
            log_event("encoder", "bounded shoulder correction completed", attempt=attempt, final_error_deg=error)
            return True, "corrected"
        if abs(error) > fault_tolerance:
            break

    if allow_correction and bool(correction.get("enabled")) and source in allowed_sources:
        recovered, recovery_message = await ensure_shoulder_alignment_before_motion(
            source,
            target_shoulder_deg=commanded_reference,
        )
        if recovered:
            return True, "corrected"
        message = recovery_message or f"shoulder correction did not converge; final error {error:.2f} deg"
    else:
        message = f"shoulder correction did not converge; final error {error:.2f} deg"
    if not state.encoder_fault:
        latch_encoder_mismatch_fault(message, error)
    return False, message


def encoder_calibration_ready() -> tuple[bool, str]:
    if state.simulation:
        return False, "shoulder encoder calibration requires connected hardware"
    if not serial_client.is_connected or not state.connected:
        return False, "connect the controller before shoulder encoder calibration"
    if state.hardware_armed:
        return False, "disarm hardware before shoulder encoder calibration"
    if state.motion_state not in {MotionState.IDLE, MotionState.STOPPED}:
        return False, "stop all motion before shoulder encoder calibration"
    if state.live_motion_enabled or task_active() or cartesian_jog_runtime.get("active"):
        return False, "stop live motion, tasks, and Cartesian jog before encoder calibration"
    if any(task is not None and not task.done() for task in [path_task, live_task, task_task]):
        return False, "wait for active motion or task execution to finish"
    return True, ""


def current_raw_shoulder_sample() -> tuple[dict[str, Any] | None, str]:
    try:
        refresh_serial_status()
    except SerialClientError as exc:
        return None, str(exc)
    evidence = _shoulder_evidence()
    raw_angle = evidence.get("raw_angle_deg")
    age_ms = evidence.get("age_ms")
    settings = encoder_settings(config)
    shoulder = encoder_axis(settings) or {}
    freshness_limit = float(shoulder.get("freshness_timeout_ms", 500))
    if not evidence.get("sensor_valid") or raw_angle is None:
        flags = ", ".join(str(flag) for flag in evidence.get("flags", []))
        return None, f"shoulder sensor has no valid raw sample{f' ({flags})' if flags else ''}"
    if age_ms is None or float(age_ms) > freshness_limit:
        return None, f"shoulder raw sample is stale ({age_ms} ms)"
    return {
        "raw_count": evidence.get("raw_count"),
        "raw_angle_deg": float(raw_angle),
        "age_ms": int(age_ms),
        "noise_deg": evidence.get("noise_deg"),
        "captured_at": time(),
    }, ""


def _validate_encoder_known_joint_angle(value: float | None) -> tuple[float | None, str]:
    if value is None:
        return None, "known shoulder angle is required"
    shoulder = config.joints[1]
    joint_angle = float(value)
    if not isfinite(joint_angle) or not shoulder.min_deg <= joint_angle <= shoulder.max_deg:
        return (
            None,
            f"known shoulder angle must be within {shoulder.min_deg:.3f}..{shoulder.max_deg:.3f} deg",
        )
    return joint_angle, ""


def _encoder_calibration_capture(
    sample: dict[str, Any],
    joint_angle: float,
    label: str,
    approach_direction: int | None = None,
    use_for_fit: bool = True,
) -> dict[str, Any]:
    captured = {
        **sample,
        "joint_angle_deg": float(joint_angle),
        "label": label.strip(),
    }
    if approach_direction in {-1, 0, 1}:
        captured["approach_direction"] = int(approach_direction)
    if not use_for_fit:
        captured["use_for_fit"] = False
    return captured


def _encoder_runtime_ready() -> tuple[dict[str, Any] | None, dict[str, Any] | None, str]:
    settings = encoder_settings(config)
    shoulder = encoder_axis(settings)
    if not settings.get("enabled") or not shoulder or not shoulder.get("enabled"):
        return None, None, "enable shoulder encoder readback and sync the controller first"
    if not _controller_supports_encoder_config():
        return None, None, "controller firmware does not advertise encoder config support"
    if state.config_sync_status != "synced":
        return None, None, f"sync controller configuration first ({state.config_sync_status})"
    return settings, shoulder, ""


def encoder_calibration_sweep_ready() -> tuple[bool, str]:
    if state.simulation:
        return False, "assisted encoder sweep requires connected hardware"
    if not serial_client.is_connected or not state.connected:
        return False, "connect the controller before assisted encoder sweep"
    if not state.hardware_armed:
        return False, "arm hardware before running the assisted sweep"
    if state.motion_state == MotionState.MOVING:
        return False, "wait for the current motion to finish before assisted encoder sweep"
    if state.motion_state == MotionState.ESTOP:
        return False, "clear ESTOP before assisted encoder sweep"
    if state.motion_state == MotionState.FAULT:
        return False, state.last_error or "clear the robot fault before assisted encoder sweep"
    if state.motion_state not in {MotionState.IDLE, MotionState.STOPPED}:
        return False, f"robot must be idle or stopped before assisted encoder sweep (current: {state.motion_state.value})"
    if state.live_motion_enabled or task_active() or cartesian_jog_runtime.get("active"):
        return False, "stop live motion, tasks, and Cartesian jog before assisted encoder sweep"
    if any(task is not None and not task.done() for task in [path_task, live_task, task_task]):
        return False, "wait for active motion or task execution to finish"
    ready, reason = hardware_ready_for_motion()
    if not ready:
        return False, reason
    _settings, _shoulder, reason = _encoder_runtime_ready()
    if reason:
        return False, reason
    return True, ""


def _angle_close(a: float, b: float, tolerance: float = 1e-4) -> bool:
    return abs(float(a) - float(b)) <= tolerance


def _encoder_sweep_targets(
    *,
    start_deg: float,
    sweep_min_deg: float,
    sweep_max_deg: float,
    step_deg: float,
) -> list[float]:
    shoulder = config.joints[1]
    values = [float(start_deg), float(sweep_min_deg), float(sweep_max_deg), float(step_deg)]
    if not all(isfinite(value) for value in values):
        raise ValueError("sweep angles and step must be finite numbers")
    if step_deg < 1.0 or step_deg > 45.0:
        raise ValueError("sweep step must be between 1 and 45 degrees")
    if sweep_min_deg >= sweep_max_deg:
        raise ValueError("sweep minimum must be below sweep maximum")
    if sweep_max_deg - sweep_min_deg > 180.0 + 1e-6:
        raise ValueError("assisted sweep is limited to at most 180 degrees")
    for label, value in [
        ("start", start_deg),
        ("minimum", sweep_min_deg),
        ("maximum", sweep_max_deg),
    ]:
        if not shoulder.min_deg <= value <= shoulder.max_deg:
            raise ValueError(
                f"sweep {label} angle {value:.3f} deg is outside "
                f"{shoulder.min_deg:.3f}..{shoulder.max_deg:.3f} deg"
            )
    if not sweep_min_deg <= start_deg <= sweep_max_deg:
        raise ValueError("start angle must be inside the sweep range")

    targets: list[float] = []
    value = start_deg - step_deg
    while value > sweep_min_deg:
        targets.append(round(value, 6))
        value -= step_deg
    if not _angle_close(start_deg, sweep_min_deg) and (
        not targets or not _angle_close(targets[-1], sweep_min_deg)
    ):
        targets.append(round(sweep_min_deg, 6))

    value = sweep_min_deg + step_deg
    while value < sweep_max_deg:
        targets.append(round(value, 6))
        value += step_deg
    if not targets or not _angle_close(targets[-1], sweep_max_deg):
        targets.append(round(sweep_max_deg, 6))
    return targets


def _encoder_unidirectional_sweep_targets(
    *,
    sweep_min_deg: float,
    sweep_max_deg: float,
    step_deg: float,
    final_approach_direction: int,
) -> list[float]:
    shoulder = config.joints[1]
    values = [float(sweep_min_deg), float(sweep_max_deg), float(step_deg)]
    if not all(isfinite(value) for value in values):
        raise ValueError("sweep angles and step must be finite numbers")
    if final_approach_direction not in {-1, 1}:
        raise ValueError("final approach direction must be +1 or -1")
    if step_deg < 1.0 or step_deg > 45.0:
        raise ValueError("sweep step must be between 1 and 45 degrees")
    if sweep_min_deg >= sweep_max_deg:
        raise ValueError("sweep minimum must be below sweep maximum")
    if sweep_max_deg - sweep_min_deg > 180.0 + 1e-6:
        raise ValueError("assisted sweep is limited to at most 180 degrees")
    for label, value in [
        ("minimum", sweep_min_deg),
        ("maximum", sweep_max_deg),
    ]:
        if not shoulder.min_deg <= value <= shoulder.max_deg:
            raise ValueError(
                f"sweep {label} angle {value:.3f} deg is outside "
                f"{shoulder.min_deg:.3f}..{shoulder.max_deg:.3f} deg"
            )

    if final_approach_direction > 0:
        targets = [round(float(sweep_min_deg), 6)]
        value = float(sweep_min_deg) + float(step_deg)
        while value < sweep_max_deg:
            targets.append(round(value, 6))
            value += float(step_deg)
        if not _angle_close(targets[-1], sweep_max_deg):
            targets.append(round(float(sweep_max_deg), 6))
        return targets

    targets = [round(float(sweep_max_deg), 6)]
    value = float(sweep_max_deg) - float(step_deg)
    while value > sweep_min_deg:
        targets.append(round(value, 6))
        value -= float(step_deg)
    if not _angle_close(targets[-1], sweep_min_deg):
        targets.append(round(float(sweep_min_deg), 6))
    return targets


def _encoder_sweep_preload_target(
    first_target_deg: float,
    *,
    final_approach_direction: int,
    preload_deg: float,
) -> float | None:
    shoulder = config.joints[1]
    preload = float(preload_deg)
    if not isfinite(preload) or preload < 0.0 or preload > 45.0:
        raise ValueError("backlash preload must be between 0 and 45 degrees")
    if preload <= 1e-9:
        return None
    if final_approach_direction > 0:
        target = float(first_target_deg) - preload
        if target < shoulder.min_deg:
            raise ValueError(
                "sweep minimum is too close to the shoulder limit for the configured "
                f"{preload:.3f} deg preload; raise the sweep minimum, reduce preload, "
                "or approach from above"
            )
    elif final_approach_direction < 0:
        target = float(first_target_deg) + preload
        if target > shoulder.max_deg:
            raise ValueError(
                "sweep maximum is too close to the shoulder limit for the configured "
                f"{preload:.3f} deg preload; lower the sweep maximum, reduce preload, "
                "or approach from below"
            )
    else:
        raise ValueError("final approach direction must be +1 or -1")
    return round(target, 6)


def _encoder_sweep_path_settings(speed_deg_s: float, accel_deg_s2: float) -> dict[str, Any]:
    speed = float(speed_deg_s)
    accel = float(accel_deg_s2)
    if not isfinite(speed) or not isfinite(accel) or speed <= 0.0 or accel <= 0.0:
        raise ValueError("sweep speed and acceleration must be positive finite numbers")
    speed = min(speed, 8.0)
    accel = min(accel, 30.0)
    settings = default_path_settings()
    settings["global_speed_deg_s"] = speed
    settings["global_accel_deg_s2"] = accel
    settings["per_joint_speed_deg_s"] = [
        min(float(value), speed) for value in settings.get("per_joint_speed_deg_s", [])
    ]
    settings["per_joint_accel_deg_s2"] = [
        min(float(value), accel) for value in settings.get("per_joint_accel_deg_s2", [])
    ]
    while len(settings["per_joint_speed_deg_s"]) < len(config.joints):
        settings["per_joint_speed_deg_s"].append(speed)
    while len(settings["per_joint_accel_deg_s2"]) < len(config.joints):
        settings["per_joint_accel_deg_s2"].append(accel)
    settings["per_joint_speed_deg_s"][1] = speed
    settings["per_joint_accel_deg_s2"][1] = accel
    return settings


def _set_encoder_sweep_status(session: dict[str, Any], **updates: Any) -> None:
    sweep = session.setdefault("sweep", {})
    sweep.update(updates)
    sweep["updated_at"] = time()


def _fail_encoder_sweep(session: dict[str, Any], message: str) -> None:
    _set_encoder_sweep_status(
        session,
        status="failed",
        error=message,
        finished_at=time(),
    )
    log_event("encoder", "assisted shoulder encoder sweep failed", error=message)


async def _capture_encoder_sweep_sample(
    session: dict[str, Any],
    joint_angle_deg: float,
    *,
    label: str,
    approach_direction: int | None = None,
) -> dict[str, Any]:
    sample, reason = current_raw_shoulder_sample()
    if sample is None:
        raise RuntimeError(reason)
    captured = _encoder_calibration_capture(
        sample,
        float(joint_angle_deg),
        label,
        approach_direction=approach_direction,
    )
    session.setdefault("samples", []).append(captured)
    session["validation"] = validate_encoder_calibration_session(session)
    log_event(
        "encoder",
        "assisted shoulder encoder sweep sample captured",
        session_id=session.get("id"),
        joint_angle_deg=joint_angle_deg,
        raw_angle_deg=sample["raw_angle_deg"],
        sample_count=len(session.get("samples", [])),
    )
    return captured


async def run_encoder_calibration_sweep(session_id: str) -> None:
    global encoder_calibration_sweep_task
    current_task = asyncio.current_task()
    session = encoder_calibration_sessions.get(session_id)
    if not session:
        return
    sweep = session.get("sweep", {})
    targets = [float(value) for value in sweep.get("targets_deg", [])]
    settings = sweep.get("path_settings", {})
    settle_s = max(0.0, float(sweep.get("settle_ms", 350)) / 1000.0)
    preload_target = sweep.get("preload_target_deg")
    final_approach_direction = int(sweep.get("final_approach_direction", 0) or 0)
    previous_shoulder_target = (
        float(session.get("samples", [{}])[-1].get("joint_angle_deg", state.reported_angles_deg[1]))
        if session.get("samples")
        else float(state.reported_angles_deg[1])
    )
    try:
        _set_encoder_sweep_status(
            session,
            status="running",
            started_at=sweep.get("started_at") or time(),
            completed=0,
            total=len(targets),
        )
        await broadcast_state()
        if preload_target is not None:
            if session.get("sweep", {}).get("cancel_requested"):
                _set_encoder_sweep_status(session, status="cancelled", finished_at=time())
                return
            preload_target = float(preload_target)
            current = state.reported_angles_deg.copy()
            if len(current) < len(config.joints):
                current = config.home_pose.copy()
            current[1] = preload_target
            _set_encoder_sweep_status(
                session,
                status="preloading",
                current_target_deg=preload_target,
                completed=0,
            )
            response = await start_joint_target_trajectory(
                current,
                "encoder_calibration_sweep",
                settings,
            )
            if not response.get("ok"):
                _fail_encoder_sweep(session, response.get("error") or "sweep preload move failed")
                return
            task = path_task
            if task is not None:
                await task
            if state.motion_state == MotionState.FAULT or state.last_error:
                _fail_encoder_sweep(session, state.last_error or "sweep preload move faulted")
                return
            diagnostics = state.motion_diagnostics or {}
            if diagnostics.get("result") not in {None, "reached"}:
                _fail_encoder_sweep(
                    session,
                    str(
                        diagnostics.get("error")
                        or diagnostics.get("result")
                        or "sweep preload move did not complete"
                    ),
                )
                return
            previous_shoulder_target = preload_target
            await broadcast_state()
        for index, target in enumerate(targets, start=1):
            if session.get("sweep", {}).get("cancel_requested"):
                _set_encoder_sweep_status(session, status="cancelled", finished_at=time())
                return
            current = state.reported_angles_deg.copy()
            if len(current) < len(config.joints):
                current = config.home_pose.copy()
            current[1] = float(target)
            _set_encoder_sweep_status(
                session,
                status="moving",
                current_target_deg=float(target),
                active_index=index,
                completed=index - 1,
            )
            response = await start_joint_target_trajectory(
                current,
                "encoder_calibration_sweep",
                settings,
            )
            if not response.get("ok"):
                _fail_encoder_sweep(session, response.get("error") or "sweep move failed")
                return
            task = path_task
            if task is not None:
                await task
            if state.motion_state == MotionState.FAULT or state.last_error:
                _fail_encoder_sweep(session, state.last_error or "sweep move faulted")
                return
            diagnostics = state.motion_diagnostics or {}
            if diagnostics.get("result") not in {None, "reached"}:
                _fail_encoder_sweep(
                    session,
                    str(diagnostics.get("error") or diagnostics.get("result") or "sweep move did not complete"),
                )
                return
            _set_encoder_sweep_status(session, status="settling", completed=index - 1)
            await asyncio.sleep(settle_s)
            _set_encoder_sweep_status(session, status="sampling", completed=index - 1)
            approach_direction = final_approach_direction if final_approach_direction in {-1, 1} else 0
            if approach_direction == 0:
                if target > previous_shoulder_target + 1e-6:
                    approach_direction = 1
                elif target < previous_shoulder_target - 1e-6:
                    approach_direction = -1
            await _capture_encoder_sweep_sample(
                session,
                float(target),
                label=f"sweep {index}/{len(targets)}",
                approach_direction=approach_direction,
            )
            previous_shoulder_target = float(target)
            _set_encoder_sweep_status(session, completed=index)
            await broadcast_state()
        validation = validate_encoder_calibration_session(session)
        session["validation"] = validation
        _set_encoder_sweep_status(
            session,
            status="completed" if validation.get("ok") else "needs_review",
            validation_ok=bool(validation.get("ok")),
            finished_at=time(),
            completed=len(targets),
        )
        log_event(
            "encoder",
            "assisted shoulder encoder sweep completed",
            session_id=session_id,
            validation_ok=bool(validation.get("ok")),
            sample_count=len(session.get("samples", [])),
        )
    except asyncio.CancelledError:
        _set_encoder_sweep_status(session, status="cancelled", finished_at=time())
        raise
    except Exception as exc:
        _fail_encoder_sweep(session, str(exc))
    finally:
        if encoder_calibration_sweep_task is current_task:
            encoder_calibration_sweep_task = None
        await broadcast_state()


async def validate_encoder_correction_enablement() -> tuple[bool, str, dict[str, Any]]:
    settings, shoulder, reason = _encoder_runtime_ready()
    if reason:
        return False, reason, {}
    if shoulder is None or settings is None:
        return False, "shoulder encoder runtime is not configured", {}
    if state.simulation:
        return False, "bounded correction requires connected hardware, not simulation", {}
    if not serial_client.is_connected or not state.connected:
        return False, "connect the controller before enabling bounded correction", {}
    if state.hardware_armed:
        return False, "disarm hardware before changing bounded correction policy", {}
    if state.motion_state not in {MotionState.IDLE, MotionState.STOPPED}:
        return False, "stop motion before changing bounded correction policy", {}
    if state.live_motion_enabled or task_active() or cartesian_jog_runtime.get("active"):
        return False, "stop live motion, tasks, and Cartesian jog before enabling bounded correction", {}
    if any(task is not None and not task.done() for task in [path_task, live_task, task_task]):
        return False, "wait for active motion or task execution to finish", {}
    evaluation = apply_hardware_evaluation()
    if evaluation["mode"] == "invalid":
        return False, "; ".join(evaluation["errors"]) or "hardware config is invalid", {}
    if len(state.hardware_axis_states) <= 1 or state.hardware_axis_states[1] != "hardware":
        return False, "bounded correction requires the shoulder stepper axis to be hardware-enabled", {}
    if not state.known_pose:
        return False, "Set Pose first; correction cannot be enabled while the planning pose is unknown", {}
    if state.encoder_fault:
        return False, "clear the encoder fault and Set Pose before enabling bounded correction", {}
    if str(shoulder.get("mounting_location")) != "joint_output":
        return False, "bounded correction requires joint-output encoder mounting", {}
    if not bool(shoulder.get("calibration_validated")):
        return False, "calibrate and commit the shoulder encoder before enabling correction", {}

    correction = settings.get("correction", {})
    shoulder_joint = config.joints[1]
    if shoulder_joint.actuator != "stepper" or not (
        shoulder_joint.hardware.stepper and shoulder_joint.hardware.stepper.enabled
    ):
        return False, "bounded correction requires an enabled hardware shoulder stepper", {}

    try:
        deadband = float(correction.get("deadband_deg", 0.75))
        max_delta = float(correction.get("max_delta_deg", 1.0))
        speed = float(correction.get("speed_deg_s", 2.0))
        accel = float(correction.get("accel_deg_s2", 10.0))
        attempts = int(correction.get("max_attempts", 2))
    except (TypeError, ValueError):
        return False, "correction limits must be numeric", {}
    if deadband < 0 or max_delta <= 0 or speed <= 0 or accel <= 0 or attempts < 1:
        return False, "correction deadband must be non-negative; limits, speed, acceleration, and attempts must be positive", {}
    if max_delta <= deadband:
        return False, "correction max delta must be greater than the correction deadband", {}

    verification = settings.get("verification", {})
    measured, reason = await _stable_shoulder_measurement(
        max(1, int(verification.get("required_stable_samples", 3)))
    )
    if measured is None:
        return False, f"shoulder encoder is not stable enough for correction enablement: {reason}", {}
    estimated = float(state.estimated_angles_deg[1])
    error = float(measured) - estimated
    warning_tolerance = float(verification.get("warning_tolerance_deg", 2.0))
    correction_limit = float(correction.get("max_delta_deg", 8.0))
    if abs(error) > correction_limit:
        return (
            False,
            (
                "shoulder encoder and planning estimate disagree by "
                f"{error:.2f} deg, which exceeds the configured correction limit "
                f"{correction_limit:.2f} deg; use Set Pose or recalibrate before enabling correction"
            ),
            {
                "measured_deg": measured,
                "estimated_deg": estimated,
                "error_deg": error,
                "deadband_deg": deadband,
                "warning_tolerance_deg": warning_tolerance,
                "max_delta_deg": correction_limit,
            },
        )
    return (
        True,
        (
            "bounded correction validation passed"
            if abs(error) <= warning_tolerance
            else "bounded correction validation passed with existing correctable shoulder mismatch"
        ),
        {
            "measured_deg": measured,
            "estimated_deg": estimated,
            "error_deg": error,
            "deadband_deg": deadband,
            "warning_tolerance_deg": warning_tolerance,
            "max_delta_deg": correction_limit,
            "initial_mismatch_correctable": abs(error) > deadband,
            "mounting_location": shoulder.get("mounting_location"),
            "calibration_id": shoulder.get("calibration_id"),
        },
    )


def _solve_least_squares(rows: list[list[float]], values: list[float]) -> list[float] | None:
    if not rows or len(rows) != len(values):
        return None
    columns = len(rows[0])
    if columns == 0 or any(len(row) != columns for row in rows):
        return None
    normal = [[0.0 for _ in range(columns)] for _ in range(columns)]
    rhs = [0.0 for _ in range(columns)]
    for row, value in zip(rows, values, strict=True):
        for i in range(columns):
            rhs[i] += row[i] * value
            for j in range(columns):
                normal[i][j] += row[i] * row[j]

    for pivot in range(columns):
        best = max(range(pivot, columns), key=lambda index: abs(normal[index][pivot]))
        if abs(normal[best][pivot]) < 1e-9:
            return None
        if best != pivot:
            normal[pivot], normal[best] = normal[best], normal[pivot]
            rhs[pivot], rhs[best] = rhs[best], rhs[pivot]
        divisor = normal[pivot][pivot]
        for column in range(pivot, columns):
            normal[pivot][column] /= divisor
        rhs[pivot] /= divisor
        for row_index in range(columns):
            if row_index == pivot:
                continue
            factor = normal[row_index][pivot]
            if abs(factor) < 1e-12:
                continue
            for column in range(pivot, columns):
                normal[row_index][column] -= factor * normal[pivot][column]
            rhs[row_index] -= factor * rhs[pivot]
    return rhs


def _linear_encoder_fit(raw_unwrapped: list[float], joints: list[float]) -> dict[str, Any] | None:
    coefficients = _solve_least_squares(
        [[1.0, float(raw)] for raw in raw_unwrapped],
        [float(joint) for joint in joints],
    )
    if coefficients is None:
        return None
    intercept, slope = coefficients
    residuals = [
        float(joint) - (intercept + slope * float(raw))
        for raw, joint in zip(raw_unwrapped, joints, strict=True)
    ]
    return {
        "model": "linear",
        "intercept": intercept,
        "slope": slope,
        "residuals": residuals,
        "max_residual_deg": max((abs(value) for value in residuals), default=0.0),
    }


def _sample_approach_directions(samples: list[dict[str, Any]], joints: list[float]) -> list[int]:
    directions: list[int] = []
    previous_joint: float | None = None
    for index, sample in enumerate(samples):
        raw_direction = sample.get("approach_direction")
        direction = 0
        try:
            candidate = int(raw_direction)
        except (TypeError, ValueError):
            candidate = 0
        if candidate in {-1, 1}:
            direction = candidate
        elif index > 0 and previous_joint is not None:
            delta = float(joints[index]) - float(previous_joint)
            if delta > 1e-6:
                direction = 1
            elif delta < -1e-6:
                direction = -1
        directions.append(direction)
        previous_joint = float(joints[index])
    return directions


def _backlash_encoder_fit(
    raw_unwrapped: list[float],
    joints: list[float],
    directions: list[int],
) -> dict[str, Any] | None:
    usable = [
        (float(raw), float(joint), int(direction))
        for raw, joint, direction in zip(raw_unwrapped, joints, directions, strict=True)
        if int(direction) in {-1, 1}
    ]
    if len(usable) < 4 or {direction for _raw, _joint, direction in usable} != {-1, 1}:
        return None
    coefficients = _solve_least_squares(
        [[1.0, raw, float(direction)] for raw, _joint, direction in usable],
        [joint for _raw, joint, _direction in usable],
    )
    if coefficients is None:
        return None
    intercept, slope, approach_bias = coefficients
    residuals = [
        joint - (intercept + slope * raw + approach_bias * float(direction))
        for raw, joint, direction in usable
    ]
    return {
        "model": "linear_with_backlash",
        "intercept": intercept,
        "slope": slope,
        "approach_bias_deg": approach_bias,
        "backlash_estimate_deg": abs(2.0 * approach_bias),
        "residuals": residuals,
        "max_residual_deg": max((abs(value) for value in residuals), default=0.0),
        "sample_count": len(usable),
    }


def _piecewise_encoder_fit(
    raw_unwrapped: list[float],
    joints: list[float],
    directions: list[int],
    *,
    linear_fit: dict[str, Any] | None,
) -> dict[str, Any] | None:
    used_directions = {int(direction) for direction in directions if int(direction) in {-1, 1}}
    if len(used_directions) > 1:
        return None
    if len(raw_unwrapped) < 5 or len(raw_unwrapped) != len(joints):
        return None
    pairs = sorted(
        [(float(raw), float(joint)) for raw, joint in zip(raw_unwrapped, joints, strict=True)],
        key=lambda item: item[0],
    )
    for first, second in zip(pairs, pairs[1:], strict=False):
        if second[0] - first[0] < 0.05:
            return None
    joint_trend = pairs[-1][1] - pairs[0][1]
    if abs(joint_trend) < 2.0:
        return None
    joint_sign = 1 if joint_trend > 0 else -1
    for first, second in zip(pairs, pairs[1:], strict=False):
        joint_step = second[1] - first[1]
        if joint_sign * joint_step < -0.25:
            return None
    slope = float(linear_fit["slope"]) if linear_fit is not None else joint_trend / (pairs[-1][0] - pairs[0][0])
    intercept = (
        float(linear_fit["intercept"])
        if linear_fit is not None
        else pairs[0][1] - slope * pairs[0][0]
    )
    return {
        "model": "piecewise_linear",
        "intercept": intercept,
        "slope": slope,
        "residuals": [0.0 for _raw, _joint in pairs],
        "max_residual_deg": 0.0,
        "sample_count": len(pairs),
    }


def _localized_backlash_diagnostics(
    samples: list[dict[str, Any]],
    *,
    sensor_turns_per_joint_turn: float | None = None,
) -> dict[str, Any]:
    grouped: dict[float, dict[int, list[float]]] = {}
    for sample in samples:
        try:
            direction = int(sample.get("approach_direction"))
            joint_angle = float(sample["joint_angle_deg"])
            raw_angle = float(sample["raw_angle_deg"])
        except (TypeError, ValueError, KeyError):
            continue
        if direction not in {-1, 1}:
            continue
        key = round(joint_angle, 3)
        grouped.setdefault(key, {}).setdefault(direction, []).append(raw_angle)

    best_raw_sep: float | None = None
    best_joint_angle: float | None = None
    pair_count = 0
    for joint_angle, branches in grouped.items():
        if -1 not in branches or 1 not in branches:
            continue
        pair_count += 1
        reference_raw = branches[-1][0]

        def branch_mean(values: list[float]) -> float:
            unwrapped = [
                reference_raw + wrapped_delta_deg(float(value), reference_raw, 360.0)
                for value in values
            ]
            return sum(unwrapped) / len(unwrapped)

        raw_sep = abs(branch_mean(branches[1]) - branch_mean(branches[-1]))
        if best_raw_sep is None or raw_sep > best_raw_sep:
            best_raw_sep = raw_sep
            best_joint_angle = float(joint_angle)

    joint_sep = None
    if (
        best_raw_sep is not None
        and sensor_turns_per_joint_turn is not None
        and abs(float(sensor_turns_per_joint_turn)) > 1e-9
    ):
        joint_sep = best_raw_sep / abs(float(sensor_turns_per_joint_turn))
    return {
        "localized_backlash_pair_count": pair_count,
        "localized_backlash_estimate_raw_deg": best_raw_sep,
        "localized_backlash_estimate_deg": joint_sep,
        "localized_backlash_at_joint_deg": best_joint_angle,
    }


def validate_encoder_calibration_session(session: dict[str, Any]) -> dict[str, Any]:
    all_samples = list(session.get("samples") or [])
    samples = [sample for sample in all_samples if sample.get("use_for_fit", True) is not False]
    errors: list[str] = []
    warnings: list[str] = []
    if len(samples) < 2:
        errors.append("capture at least two known shoulder reference samples")
        return {
            "ok": False,
            "errors": errors,
            "warnings": warnings,
            "sample_count": len(all_samples),
            "fit_sample_count": len(samples),
            "minimum_raw_span_deg": 1.0,
            "minimum_joint_span_deg": 2.0,
        }

    shoulder_joint = config.joints[1]
    for index, sample in enumerate(samples):
        joint_angle = float(sample["joint_angle_deg"])
        if not shoulder_joint.min_deg <= joint_angle <= shoulder_joint.max_deg:
            errors.append(
                f"sample {index + 1} shoulder angle {joint_angle:.3f} deg is outside "
                f"{shoulder_joint.min_deg:.3f}..{shoulder_joint.max_deg:.3f} deg"
            )

    raw_unwrapped = [float(samples[0]["raw_angle_deg"])]
    for previous, sample in zip(samples, samples[1:], strict=False):
        raw_unwrapped.append(
            raw_unwrapped[-1]
            + wrapped_delta_deg(
                float(sample["raw_angle_deg"]),
                float(previous["raw_angle_deg"]),
                360.0,
            )
        )
    joints = [float(sample["joint_angle_deg"]) for sample in samples]
    raw_span = max(raw_unwrapped) - min(raw_unwrapped)
    joint_span = max(joints) - min(joints)
    minimum_raw_span_deg = 1.0
    minimum_joint_span_deg = 2.0
    if raw_span < minimum_raw_span_deg:
        errors.append(
            "reference samples do not span enough encoder motion "
            f"({raw_span:.3f} deg raw, need at least {minimum_raw_span_deg:.3f} deg)"
        )
    if joint_span < minimum_joint_span_deg:
        errors.append(
            "reference samples do not span enough shoulder motion "
            f"({joint_span:.3f} deg, need at least {minimum_joint_span_deg:.3f} deg)"
        )

    shoulder = encoder_axis(encoder_settings(config)) or {}
    residual_limit = max(0.25, float(shoulder.get("max_noise_deg", 0.5)) * 2.0)
    linear_fit = _linear_encoder_fit(raw_unwrapped, joints)
    fit = linear_fit
    directions = _sample_approach_directions(samples, joints)
    backlash_fit: dict[str, Any] | None = None
    fit_model = "linear"
    approach_bias_deg: float | None = None
    backlash_estimate_deg: float | None = None
    calibration_map: list[dict[str, float]] = []
    provisional_turns = None
    if linear_fit is not None and abs(float(linear_fit["slope"])) >= 1e-9:
        provisional_turns = 1.0 / abs(float(linear_fit["slope"]))
    localized_backlash = _localized_backlash_diagnostics(
        samples,
        sensor_turns_per_joint_turn=provisional_turns,
    )
    fit_sample_count = len(samples)
    if fit is not None and abs(float(fit["slope"])) >= 1e-9:
        linear_max_residual = float(fit["max_residual_deg"])
        candidate = _backlash_encoder_fit(raw_unwrapped, joints, directions)
        if (
            linear_max_residual > residual_limit
            and candidate is not None
            and abs(float(candidate["slope"])) >= 1e-9
            and float(candidate["max_residual_deg"]) <= residual_limit
        ):
            fit = candidate
            backlash_fit = candidate
            fit_model = "linear_with_backlash"
            approach_bias_deg = float(candidate["approach_bias_deg"])
            backlash_estimate_deg = float(candidate["backlash_estimate_deg"])
            fit_sample_count = int(candidate["sample_count"])
            warnings.append(
                "direction-dependent shoulder backlash/lost motion detected; "
                f"estimated branch separation is {backlash_estimate_deg:.3f} deg. "
                "Calibration uses the neutral sensor slope and keeps backlash as diagnostics/correction evidence."
            )
        elif linear_max_residual > residual_limit:
            piecewise_candidate = _piecewise_encoder_fit(
                raw_unwrapped,
                joints,
                directions,
                linear_fit=linear_fit,
            )
            if piecewise_candidate is not None:
                fit = piecewise_candidate
                fit_model = "piecewise_linear"
                fit_sample_count = int(piecewise_candidate["sample_count"])
                warnings.append(
                    "repeatable nonlinear shoulder encoder map detected; using a piecewise calibration "
                    "over the captured range. This compensates sensor/linkage nonlinearity for readback, "
                    "but it does not hide backlash or enable continuous correction."
                )

    if fit is None or abs(float(fit["slope"])) < 1e-9:
        errors.append("reference samples do not establish encoder direction and scale")
        turns = None
        direction_sign = None
        residuals: list[float] = []
        slope = 0.0
        intercept = 0.0
        max_residual = None
        reference_index = 0
        reference_raw_for_calibration = float(samples[0]["raw_angle_deg"])
        reference_joint_for_calibration = float(samples[0]["joint_angle_deg"])
    else:
        slope = float(fit["slope"])
        intercept = float(fit["intercept"])
        direction_sign = 1 if slope > 0 else -1
        turns = 1.0 / abs(slope)
        residuals = [float(value) for value in fit["residuals"]]
        max_residual = float(fit["max_residual_deg"])
        if max_residual > residual_limit:
            backlash_hint = ""
            if any(direction in {-1, 1} for direction in directions):
                localized_raw = localized_backlash.get("localized_backlash_estimate_raw_deg")
                localized_joint = localized_backlash.get("localized_backlash_estimate_deg")
                localized_at = localized_backlash.get("localized_backlash_at_joint_deg")
                if localized_raw is not None:
                    localized_text = (
                        f"{localized_joint:.3f} deg"
                        if localized_joint is not None
                        else f"{localized_raw:.3f} raw deg"
                    )
                    backlash_hint = (
                        f"; localized bidirectional backlash/lost motion of {localized_text} "
                        f"detected near {localized_at:.3f} deg. "
                        "Run a same-direction/preloaded sweep or calibrate from physical fixture marks"
                    )
                else:
                    backlash_hint = (
                        "; direction-aware backlash fit was not consistent enough. "
                        "Run a same-direction/preloaded sweep or calibrate from physical fixture marks"
                    )
            errors.append(
                f"reference samples are inconsistent; maximum fit residual "
                f"{max_residual:.3f} deg exceeds {residual_limit:.3f} deg{backlash_hint}"
            )

        mounting = str(session.get("mounting_location") or "joint_output")
        reference_index = 0
        reference_joint_for_calibration = (
            joints[0] if fit_model == "piecewise_linear" else intercept + slope * raw_unwrapped[0]
        )
        for index, raw in enumerate(raw_unwrapped):
            candidate_joint = joints[index] if fit_model == "piecewise_linear" else intercept + slope * raw
            if shoulder_joint.min_deg <= candidate_joint <= shoulder_joint.max_deg:
                reference_index = index
                reference_joint_for_calibration = candidate_joint
                break
        if mounting == "joint_output" and turns is not None:
            first_reference_excursion = max(
                abs((shoulder_joint.min_deg - reference_joint_for_calibration) * turns),
                abs((shoulder_joint.max_deg - reference_joint_for_calibration) * turns),
            )
            if first_reference_excursion >= 180.0 or fit_model == "piecewise_linear":
                center = (shoulder_joint.min_deg + shoulder_joint.max_deg) / 2.0
                candidates: list[tuple[float, int, float]] = []
                for index, raw in enumerate(raw_unwrapped):
                    candidate_joint = joints[index] if fit_model == "piecewise_linear" else intercept + slope * raw
                    if shoulder_joint.min_deg <= candidate_joint <= shoulder_joint.max_deg:
                        candidates.append((abs(candidate_joint - center), index, candidate_joint))
                if candidates:
                    _distance, reference_index, reference_joint_for_calibration = min(candidates)
        reference_raw_for_calibration = float(samples[reference_index]["raw_angle_deg"])
        if not shoulder_joint.min_deg <= reference_joint_for_calibration <= shoulder_joint.max_deg:
            errors.append(
                "fitted encoder reference is outside shoulder limits; choose reference samples away from the limits"
            )
        if mounting == "joint_output":
            max_sensor_excursion = max(
                abs((shoulder_joint.min_deg - reference_joint_for_calibration) * turns),
                abs((shoulder_joint.max_deg - reference_joint_for_calibration) * turns),
            )
            if max_sensor_excursion >= 180.0:
                errors.append(
                    "the configured shoulder range is ambiguous around this single-turn reference; "
                    "choose another reference or establish multi-turn tracking"
                )
        else:
            warnings.append(
                "motor/gear-side mounting remains diagnostic-only; it cannot provide absolute "
                "joint-output authority or validate gearbox backlash"
            )
        if fit_model == "piecewise_linear":
            reference_raw_unwrapped = raw_unwrapped[reference_index]
            calibration_map = [
                {
                    "raw_delta_deg": round(float(raw) - float(reference_raw_unwrapped), 9),
                    "raw_deg": round(float(sample["raw_angle_deg"]), 9),
                    "joint_deg": round(float(joint), 9),
                }
                for raw, joint, sample in sorted(
                    zip(raw_unwrapped, joints, samples, strict=True),
                    key=lambda item: float(item[0]),
                )
            ]

    fit_points: list[dict[str, Any]] = []
    if fit is not None and abs(float(fit.get("slope", 0.0))) >= 1e-9:
        for index, sample in enumerate(samples):
            joint = float(sample["joint_angle_deg"])
            if fit_model == "piecewise_linear":
                predicted = joint
            else:
                predicted = intercept + slope * raw_unwrapped[index]
                if fit_model == "linear_with_backlash" and directions[index] in {-1, 1}:
                    predicted += float(approach_bias_deg or 0.0) * float(directions[index])
            fit_points.append(
                {
                    "index": index,
                    "raw_angle_deg": float(sample["raw_angle_deg"]),
                    "joint_angle_deg": joint,
                    "predicted_joint_deg": float(predicted),
                    "error_deg": float(joint - predicted),
                    "approach_direction": directions[index] if index < len(directions) else 0,
                }
            )

    return {
        "ok": not errors,
        "errors": errors,
        "warnings": warnings,
        "sample_count": len(all_samples),
        "fit_sample_count": fit_sample_count,
        "raw_span_deg": raw_span,
        "joint_span_deg": joint_span,
        "minimum_raw_span_deg": minimum_raw_span_deg,
        "minimum_joint_span_deg": minimum_joint_span_deg,
        "fit_model": fit_model,
        "direction_sign": direction_sign,
        "sensor_turns_per_joint_turn": turns,
        "max_residual_deg": max((abs(value) for value in residuals), default=None),
        "linear_max_residual_deg": float(linear_fit["max_residual_deg"]) if linear_fit is not None else None,
        "residual_limit_deg": residual_limit,
        "approach_directions": directions,
        "approach_bias_deg": approach_bias_deg,
        "backlash_estimate_deg": backlash_estimate_deg,
        **localized_backlash,
        "calibration_map": calibration_map,
        "calibration_map_point_count": len(calibration_map),
        "fit_points": fit_points,
        "reference_sample_index": reference_index,
        "reference_raw_deg": reference_raw_for_calibration,
        "reference_joint_deg": float(reference_joint_for_calibration),
        "mounting_location": session.get("mounting_location"),
    }


async def _move_shoulder_for_encoder_helper(
    target_shoulder_deg: float,
    *,
    source: str,
    settings: dict[str, Any],
) -> tuple[bool, str]:
    current = state.reported_angles_deg.copy()
    if len(current) < len(config.joints):
        current = config.home_pose.copy()
    current[1] = float(target_shoulder_deg)
    response = await start_joint_target_trajectory(current, source, settings)
    if not response.get("ok"):
        return False, response.get("error") or "encoder helper move failed"
    task = path_task
    if task is not None:
        await task
    if state.motion_state == MotionState.FAULT or state.last_error:
        return False, state.last_error or "encoder helper move faulted"
    diagnostics = state.motion_diagnostics or {}
    if diagnostics.get("result") not in {None, "reached"}:
        return False, str(diagnostics.get("error") or diagnostics.get("result") or "encoder helper move failed")
    return True, "reached"


def encoder_backlash_check_ready() -> tuple[bool, str]:
    if state.simulation:
        return False, "backlash check requires connected hardware"
    if not serial_client.is_connected or not state.connected:
        return False, "connect the controller before backlash check"
    if not state.hardware_armed:
        return False, "arm hardware before running the backlash check"
    if state.motion_state == MotionState.MOVING:
        return False, "wait for the current motion to finish before backlash check"
    if state.motion_state == MotionState.ESTOP:
        return False, "clear ESTOP before backlash check"
    if state.motion_state == MotionState.FAULT:
        return False, state.last_error or "clear the robot fault before backlash check"
    if state.live_motion_enabled or task_active() or cartesian_jog_runtime.get("active"):
        return False, "stop live motion, tasks, and Cartesian jog before backlash check"
    if any(task is not None and not task.done() for task in [path_task, live_task, task_task]):
        return False, "wait for active motion or task execution to finish"
    if not state.known_pose:
        return False, "Set Pose first; backlash check needs a known planning pose"
    _settings, shoulder, reason = _encoder_runtime_ready()
    if reason:
        return False, reason
    if not shoulder or not shoulder.get("calibration_validated"):
        return False, "quick-calibrate or commit shoulder encoder calibration before backlash check"
    if str(shoulder.get("mounting_location")) != "joint_output":
        return False, "backlash check requires joint-output encoder mounting"
    ready, reason = hardware_ready_for_motion()
    if not ready:
        return False, reason
    return True, ""


async def run_encoder_backlash_check(request: EncoderBacklashCheckRequest) -> dict[str, Any]:
    ready, reason = encoder_backlash_check_ready()
    if not ready:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    center = (
        float(request.center_joint_angle_deg)
        if request.center_joint_angle_deg is not None
        else float(state.reported_angles_deg[1])
    )
    shoulder = config.joints[1]
    travel = float(request.travel_deg)
    repeats = int(request.repeats)
    settle_ms = int(request.settle_ms)
    if not isfinite(center) or not shoulder.min_deg <= center <= shoulder.max_deg:
        return {"ok": False, "error": "backlash check center is outside shoulder limits", "state": state.to_dict()}
    if not isfinite(travel) or travel < 2.0 or travel > 30.0:
        return {"ok": False, "error": "backlash check travel must be between 2 and 30 degrees", "state": state.to_dict()}
    if repeats < 1 or repeats > 5:
        return {"ok": False, "error": "backlash check repeats must be between 1 and 5", "state": state.to_dict()}
    if settle_ms < 100 or settle_ms > 5000:
        return {"ok": False, "error": "settle time must be between 100 and 5000 ms", "state": state.to_dict()}
    low = center - travel
    high = center + travel
    if low < shoulder.min_deg or high > shoulder.max_deg:
        return {
            "ok": False,
            "error": (
                f"backlash check needs {travel:.1f} deg on both sides of {center:.1f} deg; "
                f"choose a center within {shoulder.min_deg + travel:.1f}..{shoulder.max_deg - travel:.1f} deg"
            ),
            "state": state.to_dict(),
        }
    settings = _encoder_sweep_path_settings(float(request.speed_deg_s), 24.0)
    verification = encoder_settings(config).get("verification", {})
    required_samples = max(1, int(verification.get("required_stable_samples", 3)))
    settle_s = settle_ms / 1000.0
    samples: list[dict[str, Any]] = []

    async def move_and_capture(preload_target: float, approach_direction: int, label: str) -> tuple[bool, str]:
        ok, message = await _move_shoulder_for_encoder_helper(
            preload_target,
            source="encoder_backlash_check",
            settings=settings,
        )
        if not ok:
            return False, message
        ok, message = await _move_shoulder_for_encoder_helper(
            center,
            source="encoder_backlash_check",
            settings=settings,
        )
        if not ok:
            return False, message
        await asyncio.sleep(settle_s)
        measured, stable_reason = await _stable_shoulder_measurement(required_samples)
        if measured is None:
            return False, stable_reason
        evidence = _shoulder_evidence()
        samples.append(
            {
                "label": label,
                "center_joint_angle_deg": center,
                "approach_direction": approach_direction,
                "preload_target_deg": preload_target,
                "measured_angle_deg": measured,
                "error_deg": measured - center,
                "raw_angle_deg": evidence.get("raw_angle_deg"),
                "raw_count": evidence.get("raw_count"),
                "age_ms": evidence.get("age_ms"),
                "noise_deg": evidence.get("noise_deg"),
                "captured_at": time(),
            }
        )
        return True, "captured"

    for repeat in range(1, repeats + 1):
        ok, message = await move_and_capture(low, 1, f"repeat {repeat} from below")
        if not ok:
            return {"ok": False, "error": message, "samples": samples, "state": state.to_dict()}
        ok, message = await move_and_capture(high, -1, f"repeat {repeat} from above")
        if not ok:
            return {"ok": False, "error": message, "samples": samples, "state": state.to_dict()}

    below = [sample["measured_angle_deg"] for sample in samples if sample["approach_direction"] == 1]
    above = [sample["measured_angle_deg"] for sample in samples if sample["approach_direction"] == -1]
    below_mean = sum(below) / len(below)
    above_mean = sum(above) / len(above)
    backlash = abs(above_mean - below_mean)
    midpoint = (above_mean + below_mean) / 2.0
    result = {
        "status": "measured",
        "center_joint_angle_deg": center,
        "travel_deg": travel,
        "repeats": repeats,
        "from_below_mean_deg": below_mean,
        "from_above_mean_deg": above_mean,
        "backlash_estimate_deg": backlash,
        "midpoint_error_deg": midpoint - center,
        "samples": samples,
        "checked_at": time(),
        "interpretation": (
            "backlash/lost motion is large enough that post-move correction is useful"
            if backlash >= 1.0
            else "backlash at this target is small"
        ),
    }
    state.encoder_mismatch = {
        **state.encoder_mismatch,
        "status": "backlash_measured",
        "source": "encoder_backlash_check",
        "measured_deg": midpoint,
        "error_deg": midpoint - center,
        "backlash_estimate_deg": backlash,
        "checked_at": result["checked_at"],
    }
    log_event(
        "encoder",
        "shoulder backlash check completed",
        center_joint_angle_deg=center,
        travel_deg=travel,
        backlash_estimate_deg=backlash,
        midpoint_error_deg=midpoint - center,
    )
    await broadcast_state()
    return {"ok": True, "backlash": result, "samples": samples, "state": state.to_dict()}


def persist_encoder_calibration(settings: dict[str, Any]) -> dict[str, Any]:
    config_path = ensure_local_config()
    with tempfile.TemporaryDirectory() as tmp_dir:
        draft_path = Path(tmp_dir) / "robot.local.yaml"
        shutil.copyfile(config_path, draft_path)
        save_calibration_updates(draft_path, {"encoders": settings})
        draft_config = load_config(draft_path)
        errors = validate_encoder_settings(draft_config, encoder_settings(draft_config))
        if errors:
            raise ValueError("; ".join(errors))
        change = classify_config_change(config, draft_config)
        ready, reason = config_change_ready(change)
        if not ready:
            raise ValueError(reason)
    save_calibration_updates(config_path, {"encoders": settings})
    saved_config = load_config(config_path)
    reload_runtime_config(saved_config, change)
    return change


def _controller_supports_encoder_config() -> bool:
    capabilities = state.controller_capabilities or {}
    return bool(capabilities.get("encoder_config"))


def _encoder_runtime_requested(settings: dict[str, Any]) -> bool:
    return bool(settings.get("enabled"))


def _encoder_runtime_should_be_active(settings: dict[str, Any] | None = None) -> bool:
    return _controller_supports_encoder_config() and _encoder_runtime_requested(settings or encoder_settings(config))


def _encoder_runtime_off_message() -> str:
    return (
        "controller reports encoder runtime off even though shoulder encoder readback is enabled; "
        "disarm and sync controller configuration"
    )


def _encoder_unsupported_message(settings: dict[str, Any]) -> str:
    capabilities = state.controller_capabilities or {}
    protocol = capabilities.get("protocol")
    raw = capabilities.get("raw") or state.last_command or "unknown controller"
    if _encoder_runtime_requested(settings):
        return (
            "controller firmware does not advertise protocol-v4 encoder config support; "
            "disable the encoder bus or flash the current arm_controller firmware. "
            f"Controller hello: {raw}"
        )
    protocol_text = f"protocol {protocol}" if protocol is not None else "legacy protocol"
    return (
        f"encoder config is disabled locally and was skipped for {protocol_text} controller compatibility"
    )


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

    encoders = encoder_settings(config)
    encoder_supported = _controller_supports_encoder_config()
    if _encoder_runtime_requested(encoders) and not encoder_supported:
        state.config_sync_status = "unsupported"
        state.config_sync_message = _encoder_unsupported_message(encoders)
        log_event("controller", "encoder config sync unsupported", detail=state.config_sync_message)
        return {
            "ok": False,
            "status": state.config_sync_status,
            "evaluation": evaluation,
            "message": state.config_sync_message,
        }
    encoders_for_sync = encoders if encoder_supported else None

    try:
        serial_client.clear_input()
        for line in format_config_lines(config.joints, tools_settings(config), encoders_for_sync):
            serial_client.send_line(line)
        response = read_serial_until_any(("OK command=CONFIG", "ERR"), timeout_s=2.0)
    except (SerialClientError, ValueError) as exc:
        state.config_sync_status = "failed"
        state.config_sync_message = str(exc)
        return {"ok": False, "status": state.config_sync_status, "evaluation": evaluation, "message": state.config_sync_message}

    if response.startswith("OK command=CONFIG"):
        if _encoder_runtime_should_be_active(encoders):
            try:
                serial_client.clear_input()
                serial_client.send_line(format_status())
                status_line = serial_client.read_until_prefix("STATUS", timeout_s=1.0)
                apply_controller_status(status_line)
            except Exception as exc:
                state.config_sync_status = "failed"
                state.config_sync_message = f"could not verify encoder runtime after config sync: {exc}"
                return {
                    "ok": False,
                    "status": state.config_sync_status,
                    "evaluation": evaluation,
                    "response": response,
                    "message": state.config_sync_message,
                }
            if state.closed_loop_mode == "off":
                state.config_sync_status = "failed"
                state.config_sync_message = _encoder_runtime_off_message()
                state.encoder_mismatch = {
                    **state.encoder_mismatch,
                    "status": "encoder_config_inactive",
                    "message": state.config_sync_message,
                    "checked_at": time(),
                }
                log_event("controller", "encoder runtime inactive after config sync", status_line=status_line)
                return {
                    "ok": False,
                    "status": state.config_sync_status,
                    "evaluation": evaluation,
                    "response": response,
                    "message": state.config_sync_message,
                }
        state.config_sync_status = "synced"
        if state.config_change.get("pose_revalidation_required") and not state.known_pose:
            state.config_sync_message = (
                "controller configuration synced; current pose remains unknown. "
                "Use Set Pose while disarmed before arming."
            )
        else:
            state.config_sync_message = response
        if not encoder_supported:
            compatibility_note = _encoder_unsupported_message(encoders)
            state.config_sync_message = f"{state.config_sync_message}; {compatibility_note}"
        state.last_command = response
        state.clear_error()
        log_event(
            "controller",
            response,
            pose_revalidation_required=bool(state.config_change.get("pose_revalidation_required")),
            known_pose=state.known_pose,
            encoder_config_sent=encoder_supported,
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
    calibration_trial: bool = False,
    program_revision: int | None = None,
) -> dict[str, Any]:
    mode = mode.lower()
    ik_result: dict[str, Any] | None = None
    calibration_compatible = links == config.links
    calibration_requested = bool(apply_calibration and calibration_compatible)
    calibration_metadata: dict[str, Any] | list[dict[str, Any]] | None = None
    command_target: dict[str, Any] = {}

    if waypoint_program:
        configured_tools = tools_settings(config)
        active_tool = str(configured_tools.get("active", "gripper"))
        active_preset = configured_tools.get("presets", {}).get(active_tool, {})
        active_tool_type = str(active_preset.get("type", "generic")) if isinstance(active_preset, dict) else "generic"
        for step_index, waypoint in enumerate(waypoint_program):
            if waypoint.get("enabled", True) is False:
                continue
            kind = str(waypoint.get("type") or waypoint.get("kind") or "cartesian").lower()
            if kind != "tool":
                continue
            requested_tool = str(waypoint.get("tool") or active_tool)
            action = str(waypoint.get("action") or "").lower()
            if requested_tool != active_tool:
                return {
                    "ok": False,
                    "error": f"program step {step_index + 1} requires {requested_tool}; select that end effector before previewing",
                    "diagnostic_category": "tool_configuration",
                }
            if active_tool_type == "servo_gripper" and action not in {"open", "close", "set"}:
                return {
                    "ok": False,
                    "error": f"program step {step_index + 1}: {active_tool} does not support {action or 'an empty action'}",
                    "diagnostic_category": "tool_configuration",
                }
            if active_tool_type == "electromagnet" and action not in {"on", "off"}:
                return {
                    "ok": False,
                    "error": f"program step {step_index + 1}: {active_tool} does not support {action or 'an empty action'}",
                    "diagnostic_category": "tool_configuration",
                }
        prepared_program, corrections = correct_waypoint_program(
            waypoint_program,
            config,
            apply_enabled=calibration_requested,
            validation_trial=calibration_trial,
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
            validation_trial=calibration_trial,
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
            selection_policy=ik_selection_policy(settings),
        )
        if not ik_result["ok"] or not ik_result["selected"]:
            return {
                "ok": False,
                "error": ik_result.get("failure_reason") or "IK target has no valid solution",
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
        "branch": branch,
        "ik": ik_result,
        "trajectory": trajectory,
        "motion_contract": trajectory.get("motion_contract", {}),
        "limit_summary": trajectory.get("limit_summary", {}),
        "completion_feedback": "timed + STATUS estimate for hardware",
    }
    attach_controller_command_to_motion_contract(
        preview,
        anticipated_controller_command(str(trajectory.get("mode", preview_mode))),
        settings,
    )
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
        reason = controller_pose_rebase_blocking_reason()
        if reason:
            state.set_error(reason)
            return {"ok": False, "error": reason, "state": state.to_dict()}
        reason = shoulder_alignment_motion_blocking_reason()
        if reason:
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
        aligned, reason = await ensure_shoulder_alignment_before_motion(command_label, settings_payload)
        if not aligned:
            state.set_error(reason)
            await broadcast_state()
            return {"ok": False, "error": reason, "state": state.to_dict()}
        reason = hardware_trajectory_start_blocking_reason()
        if reason:
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
        "motion_contract": trajectory.get("motion_contract", {}),
        "limit_summary": trajectory.get("limit_summary", {}),
        "completion_feedback": "timed + STATUS estimate for hardware",
    }
    attach_controller_command_to_motion_contract(
        preview,
        anticipated_controller_command(str(trajectory.get("mode", "joint"))),
        settings,
    )
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
    command_contract_name = "SIM_TRAJ" if state.simulation else "TRAJ"
    runtime_motion_contract = motion_contract_for_controller(
        trajectory.get("motion_contract") or preview.get("motion_contract"),
        command_contract_name,
        settings,
    )
    run_id = start_motion_diagnostics(
        source=str(preview.get("source", "joint")),
        mode="joint_timed_simulation" if state.simulation else "joint_timed_trajectory",
        target_deg=target,
        expected_duration_s=expected_duration,
        waypoint_count=len(waypoints),
        motion_contract=runtime_motion_contract,
        step_label=str(preview.get("task_step_label", "")),
        step_index=int(preview.get("task_step_index", 0) or 0),
        step_total=int(preview.get("task_step_total", 0) or 0),
    )
    update_motion_diagnostics(
        run_id,
        execution_state="executing",
        result="executing",
        current_waypoint_index=1,
        current_waypoint_total=len(waypoints),
        active_target_deg=target,
    )

    try:
        if state.simulation:
            state.last_command = f"SIM_TRAJ {preview.get('source', 'joint')}"
            await execute_simulated_waypoint_trajectory(trajectory, run_id)
            return

        if len(waypoints) < 2:
            if has_reached_target(state.reported_angles_deg, target, tolerance_deg=0.08):
                state.target_angles_deg = target.copy()
                limiter.reset(target)
                state.motion_state = MotionState.IDLE
                state.clear_error()
                finish_motion_diagnostics("reached", run_id=run_id)
            else:
                message = "hardware joint trajectory requires at least two timed waypoints"
                state.set_error(message, fault=True)
                finish_motion_diagnostics("failed", message, run_id)
            return

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
        if not serial_client.is_connected:
            raise SerialClientError("trajectory execution requires serial hardware connection")
        reason = hardware_trajectory_start_blocking_reason(preview)
        if reason:
            state.set_error(reason)
            finish_motion_diagnostics("failed", reason, run_id)
            return

        update_motion_diagnostics(
            run_id,
            execution_state="uploading",
            result="executing",
            current_waypoint_index=0,
            current_waypoint_total=len(waypoints),
            active_target_deg=target,
        )
        state.motion_state = MotionState.MOVING
        state.last_command = f"TRAJ_UPLOAD {len(waypoints)}"
        state.clear_error()
        await broadcast_state()

        upload = send_trajectory_and_read_response(trajectory, speed_value, accel_value)
        state.target_angles_deg = target.copy()
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
            active_target_deg=target,
            uploaded_waypoint_count=upload["point_count"],
            uploaded_duration_s=upload["duration_s"],
        )
        await broadcast_state()

        ok, message = await wait_for_hardware_target(
            target,
            timeout_s=max(1.0, expected_duration * 2.0 + 2.0),
            tolerance_deg=float(calibration_settings(config).get("movement_tolerance_deg", 0.2)),
            poll_interval_s=0.20,
        )
        if ok:
            verified, verification_message = await verify_shoulder_after_motion(
                str(preview.get("source", "joint")),
                target,
            )
            if verified:
                finish_motion_diagnostics("reached", verification_message, run_id)
            else:
                finish_motion_diagnostics("failed", verification_message, run_id)
        else:
            state.set_error(message, fault=True)
            finish_motion_diagnostics("failed", message, run_id)
    except (SerialClientError, ValueError) as exc:
        state.set_error(str(exc), fault=True)
        finish_motion_diagnostics("failed", str(exc), run_id)
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
    command_contract_name = "SIM_TRAJ" if state.simulation else "TRAJ"
    runtime_motion_contract = motion_contract_for_controller(
        trajectory.get("motion_contract") or preview.get("motion_contract"),
        command_contract_name,
        settings,
    )
    run_id = start_motion_diagnostics(
        source=str(preview.get("source", "path")),
        mode=str(trajectory.get("mode", preview.get("mode", "path"))),
        target_deg=final_target,
        expected_duration_s=float(trajectory.get("duration_s", 0.0)),
        waypoint_count=len(waypoints),
        motion_contract=runtime_motion_contract,
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
            reason = hardware_trajectory_start_blocking_reason(preview)
            if reason:
                state.set_error(reason)
                finish_motion_diagnostics("failed", reason, run_id)
                return
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
                verified, verification_message = await verify_shoulder_after_motion(
                    str(preview.get("source", "path")),
                    final_target,
                )
                if verified:
                    finish_motion_diagnostics("reached", verification_message, run_id)
                else:
                    finish_motion_diagnostics("failed", verification_message, run_id)
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
                    verified, verification_message = await verify_shoulder_after_motion(
                        str(preview.get("source", "path")),
                        waypoint_values,
                    )
                    if not verified:
                        finish_motion_diagnostics("failed", verification_message, run_id)
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
        next_tool_state = "open" if normalized == "open" else "closed"
        next_tool_value = preset.get("open_value" if normalized == "open" else "closed_value")
    elif normalized in {"on", "off"}:
        command = format_tool(normalized)
        next_tool_state = normalized
        next_tool_value = 1.0 if normalized == "on" else 0.0
    else:
        command = format_tool("set", value)
        next_tool_state = "set"
        next_tool_value = max(0.0, min(1.0, float(value if value is not None else 0.0)))

    state.last_command = command
    if state.simulation:
        state.tool_state = next_tool_state
        state.tool_value = next_tool_value
        state.clear_error()
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
        response_fields = {
            key: raw_value
            for token in response.split()[1:]
            if "=" in token
            for key, raw_value in [token.split("=", 1)]
        }
        state.last_controller_response = response
        state.tool_state = response_fields.get("state", next_tool_state)
        try:
            state.tool_value = float(response_fields["value"]) if "value" in response_fields else next_tool_value
        except ValueError:
            state.tool_value = next_tool_value
        state.clear_error()
        log_event("tool", command, response=response)
    except SerialClientError as exc:
        state.set_error(str(exc), fault=True)
        await broadcast_state()
        return {"ok": False, "error": str(exc), "state": state.to_dict()}
    await broadcast_state()
    return {"ok": True, "command": command, "state": state.to_dict()}


async def execute_program_sequence(preview: dict[str, Any]) -> None:
    execution_steps = list(preview.get("trajectory", {}).get("execution_steps") or [])
    if not execution_steps:
        state.set_error("program preview has no executable steps")
        finish_motion_diagnostics("failed", state.last_error)
        await broadcast_state()
        return

    total = len(execution_steps)
    try:
        for step_number, execution_step in enumerate(execution_steps, start=1):
            if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
                finish_motion_diagnostics("stopped", state.motion_state.value)
                await broadcast_state()
                return
            label = str(execution_step.get("label") or f"Step {step_number}")
            if execution_step.get("kind") == "tool":
                duration_s = max(0.0, float(execution_step.get("duration_s", 0.0)))
                run_id = start_motion_diagnostics(
                    source="program",
                    mode="tool_action",
                    target_deg=state.reported_angles_deg.copy(),
                    expected_duration_s=duration_s,
                    waypoint_count=0,
                    step_label=label,
                    step_index=step_number,
                    step_total=total,
                )
                update_motion_diagnostics(
                    run_id,
                    execution_state="executing",
                    result="executing",
                    active_step_label=label,
                    active_step_index=step_number,
                    active_step_total=total,
                )
                await broadcast_state()
                result = await apply_tool_action(
                    str(execution_step.get("action") or ""),
                    execution_step.get("value"),
                    execution_step.get("tool"),
                )
                if not result.get("ok"):
                    finish_motion_diagnostics(
                        "failed",
                        str(result.get("error") or "end-effector action failed"),
                        run_id,
                    )
                    await broadcast_state()
                    return
                if duration_s > 0:
                    await asyncio.sleep(duration_s)
                finish_motion_diagnostics("reached", run_id=run_id)
                await broadcast_state()
                continue

            trajectory = execution_step.get("trajectory")
            if not isinstance(trajectory, dict):
                state.set_error(f"program step {step_number} is missing its planned trajectory")
                finish_motion_diagnostics("failed", state.last_error)
                await broadcast_state()
                return
            aligned, alignment_reason = await ensure_shoulder_alignment_before_motion(
                "program",
                execution_step.get("settings") or preview.get("settings", {}),
            )
            if not aligned:
                state.set_error(alignment_reason)
                finish_motion_diagnostics("failed", alignment_reason)
                await broadcast_state()
                return
            step_preview = {
                **preview,
                "source": "program",
                "settings": execution_step.get("settings") or preview.get("settings", {}),
                "trajectory": trajectory,
                "task_step_label": label,
                "task_step_index": step_number,
                "task_step_total": total,
            }
            if str(trajectory.get("mode", "")).lower() == "joint":
                await execute_joint_endpoint_move(step_preview)
            else:
                await execute_waypoint_path(step_preview)
            if state.motion_execution_state in {"failed", "stopped"}:
                return
            if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
                return
    except asyncio.CancelledError:
        if state.motion_execution_state not in {"failed", "stopped"}:
            finish_motion_diagnostics("stopped", "cancelled")
            await broadcast_state()
        raise
    except Exception as exc:
        message = f"program execution failed: {exc}"
        state.set_error(message, fault=not state.simulation)
        finish_motion_diagnostics("failed", message)
        log_event("motion", "program execution failed", error=str(exc))
        await broadcast_state()


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
            current_step = {
                "label": label,
                "index": step_index,
                "total": len(steps),
                "kind": step.get("kind"),
                "phase": step.get("phase"),
                "target_frame": step.get("target_frame"),
                "movement_mode": step.get("movement_mode"),
                "height_mm": step.get("height_mm"),
                "safe_retreat_available": bool(step.get("safe_retreat_available")),
                "recovery_target": deepcopy(step.get("recovery_target")),
            }
            hold_state = str((state.task_execution or {}).get("object_hold_state") or "none")
            hold_transition = str(step.get("hold_transition") or "")
            if hold_transition and (hold_transition != "none" or hold_state == "none"):
                hold_state = hold_transition
            update_task_execution(
                status="executing",
                phase=str(step.get("phase") or "executing_sequence"),
                current_step=current_step,
                current_object={
                    "index": step.get("object_index"),
                    "detection_id": step.get("detection_id"),
                    "color": step.get("color"),
                    "drop_zone": step.get("drop_zone"),
                    "grid_slot": step.get("grid_slot"),
                },
                object_hold_state=hold_state,
                holding_uncertain=hold_state in UNCERTAIN_HOLD_STATES,
            )
            recovery = task_recovery_summary()
            update_task_execution(
                safe_retreat_available=recovery["safe_retreat_available"],
                recovery_options=recovery["options"],
                recovery=recovery,
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
                update_task_execution(
                    last_completed_step={**current_step, "completed_at": time()},
                )
                continue
            waypoint = step.get("waypoint")
            if not isinstance(waypoint, dict):
                state.set_error(f"task step {label} is missing a waypoint")
                failed_reason = state.last_error
                break
            aligned, alignment_reason = await ensure_shoulder_alignment_before_motion(
                "task",
                settings,
            )
            if not aligned:
                state.set_error(alignment_reason)
                failed_reason = alignment_reason
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
            if preview.get("mode") == "program" and preview.get("trajectory", {}).get("execution_steps"):
                await execute_program_sequence(preview)
            elif trajectory_mode == "joint":
                await execute_joint_endpoint_move(preview)
            else:
                await execute_waypoint_path(preview)
            if state.motion_state == MotionState.FAULT or state.motion_execution_state == "failed":
                failed_reason = state.last_error or "task motion step failed"
                break
            if state.motion_state in {MotionState.ESTOP, MotionState.STOPPED}:
                stopped = True
                break
            update_task_execution(
                last_completed_step={**current_step, "completed_at": time()},
            )

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
                if (
                    next_object_index is not None
                    and settings.get("cycle_confirmation") == "confirm_each_object"
                ):
                    run_id = str((state.task_execution or {}).get("run_id") or "")
                    if not run_id:
                        failed_reason = "task confirmation run ID is missing"
                        break
                    await wait_for_task_confirmation(run_id)
        return {
            "ok": failed_reason is None and not stopped and state.motion_state != MotionState.FAULT,
            "error": failed_reason or (state.motion_state.value if stopped else ""),
        }
    except asyncio.CancelledError:
        cancelled = True
        log_event("task", "task cancelled")
        if terminal_on_finish:
            finish_task_execution("stopped", "task cancelled")
        raise
    except Exception as exc:
        failed_reason = f"task execution error: {exc}"
        state.set_error(failed_reason)
        log_event(
            "task",
            "task execution failed",
            error=str(exc),
            step=(state.task_execution or {}).get("current_step"),
        )
        return {"ok": False, "error": failed_reason}
    finally:
        if terminal_on_finish and not cancelled:
            if failed_reason:
                finish_task_execution("failed", failed_reason)
            elif stopped:
                finish_task_execution("stopped", state.motion_state.value)
            elif state.motion_state == MotionState.FAULT:
                finish_task_execution("failed", state.last_error or "task motion fault")
            else:
                total = int((state.task_execution or {}).get("total_count", 0))
                update_task_execution(completed_count=total, remaining_count=0)
                finish_task_execution("completed", "sequence complete")
        await broadcast_state()


def _named_position_resolution_error(name: str) -> str:
    position_name = str(name or "").strip()
    if not position_name:
        return "named position is empty"
    position = named_positions(config).get(position_name)
    if not isinstance(position, dict):
        return f"named position {position_name} is missing"
    try:
        errors = validate_named_position(config, position_name, position)
    except Exception as exc:
        return f"named position {position_name} is invalid: {exc}"
    if errors:
        return f"named position {position_name} is invalid: {'; '.join(str(error) for error in errors)}"
    if str(position.get("type", "joint")).lower() == "joint" and not isinstance(position.get("angles_deg"), list):
        return f"named position {position_name} is missing angles_deg"
    return f"named position {position_name} could not be converted to a task waypoint"


def check_task_named_position_motion(name: str, settings: dict[str, Any], branch: str, label: str) -> dict[str, Any]:
    waypoint = named_position_waypoint(config, name)
    if waypoint is None:
        return {"ok": False, "error": _named_position_resolution_error(name)}
    try:
        prepared_program, corrections = correct_waypoint_program(
            [waypoint],
            config,
            apply_enabled=True,
            validation_trial=False,
        )
        trajectory = build_program_trajectory(
            state.reported_angles_deg,
            prepared_program,
            config.links,
            config.joints,
            settings,
            branch,
        )
    except Exception as exc:
        return {"ok": False, "error": f"{label} could not be planned: {exc}"}
    if not trajectory.get("ok"):
        return {
            "ok": False,
            "error": f"{label} cannot be planned: {'; '.join(str(error) for error in trajectory.get('errors', [])) or 'motion preview failed'}",
            "trajectory": trajectory,
            "calibration": corrections,
        }
    return {
        "ok": True,
        "position": str(name),
        "label": label,
        "trajectory": {
            "mode": trajectory.get("mode"),
            "duration_s": trajectory.get("duration_s", 0.0),
            "waypoint_count": trajectory.get("waypoint_count", 0),
        },
        "calibration": corrections,
    }


async def move_task_named_position(name: str, settings: dict[str, Any], branch: str, label: str) -> dict[str, Any]:
    waypoint = named_position_waypoint(config, name)
    if waypoint is None:
        return {"ok": False, "error": _named_position_resolution_error(name)}
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


async def wait_for_task_confirmation(run_id: str) -> None:
    event = asyncio.Event()
    task_confirmation_events[run_id] = event
    update_task_execution(
        status="waiting_for_confirmation",
        phase="waiting_for_confirmation",
        current_step={"label": "waiting for operator", "kind": "operator"},
    )
    await broadcast_state()
    try:
        await event.wait()
    finally:
        task_confirmation_events.pop(run_id, None)


async def execute_closed_loop_sorting(preview: dict[str, Any]) -> None:
    run_id = str((state.task_execution or {}).get("run_id") or uuid4())
    task_settings = normalize_color_sorting_settings(config, preview.get("task_settings"))
    runtime_config = task_runtime_config(config, task_settings)
    path_settings = preview.get("settings", {})
    branch = preview.get("branch", "auto")
    completed = 0
    grid_zone_counts: dict[str, int] = {}
    terminal_reason = ""
    try:
        while completed < int(task_settings.get("max_objects", 1)):
            aligned, alignment_reason = await ensure_shoulder_alignment_before_motion(
                "task",
                path_settings,
            )
            if not aligned:
                state.set_error(alignment_reason)
                finish_task_execution("failed", alignment_reason)
                await broadcast_state()
                return
            gate_reason = task_motion_gate_reason()
            if gate_reason:
                state.set_error(gate_reason)
                finish_task_execution("failed", gate_reason)
                await broadcast_state()
                return
            if completed == 0:
                resume_fresh_task_motion_from_stopped()

            update_task_execution(
                status="running",
                phase="moving_camera_clear",
                completed_count=completed,
                remaining_count=max(0, int(task_settings.get("max_objects", 1)) - completed),
                current_step={"label": "camera clear", "kind": "move"},
            )
            await broadcast_state()
            clear_result = await move_task_named_position(
                str(task_settings.get("camera_clear_position") or task_settings.get("safe_position") or "home"),
                path_settings,
                branch,
                "camera clear",
            )
            if not clear_result.get("ok"):
                reason = clear_result.get("error") or "camera-clear move failed"
                if not str(reason).lower().startswith("camera clear"):
                    reason = f"camera clear failed: {reason}"
                state.set_error(reason)
                finish_task_execution("failed", reason)
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
                finish_task_execution("failed", reason)
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
                runtime_config,
                capture.get("detections", []),
                color_profiles(runtime_config),
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
                    finish_task_execution("stopped", terminal_reason)
                    await broadcast_state()
                    return
                plan = build_color_sorting_plan(
                    runtime_config,
                    capture.get("detections", []),
                    color_profiles(runtime_config),
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
                finish_task_execution("failed", reason)
                await broadcast_state()
                return

            aligned, alignment_reason = await ensure_shoulder_alignment_before_motion(
                "task",
                path_settings,
            )
            if not aligned:
                state.set_error(alignment_reason)
                finish_task_execution("failed", alignment_reason)
                await broadcast_state()
                return
            gate_reason = task_motion_gate_reason()
            if gate_reason:
                state.set_error(gate_reason)
                finish_task_execution("failed", gate_reason)
                await broadcast_state()
                return

            skipped_preflight_ids: set[str] = set()
            while True:
                preflight, plan, skipped_objects = build_task_motion_preview_skipping_failed_objects(
                    plan,
                    links=runtime_config.links,
                    settings=path_settings,
                    branch=branch,
                )
                metadata = plan.get("task_preview", {})
                for skipped in skipped_objects:
                    if skipped.get("detection_id") is not None:
                        skipped_preflight_ids.add(str(skipped["detection_id"]))
                if preflight.get("ok"):
                    break
                candidate_ids = [
                    str(item.get("detection_id"))
                    for item in metadata.get("candidate_objects", [])
                    if isinstance(item, dict)
                    and item.get("detection_id") is not None
                    and str(item.get("detection_id")) not in skipped_preflight_ids
                ]
                if not candidate_ids:
                    reason = preflight.get("error", "closed-loop cycle preflight failed")
                    if reason.startswith("no task objects have a safe continuous IK path"):
                        terminal_reason = "no reachable task objects"
                        finish_task_execution("completed", terminal_reason)
                        await broadcast_state()
                        return
                    state.set_error(reason)
                    finish_task_execution("failed", reason)
                    await broadcast_state()
                    return
                plan = build_color_sorting_plan(
                    runtime_config,
                    capture.get("detections", []),
                    color_profiles(runtime_config),
                    task_settings={
                        **task_settings,
                        "execution_strategy": "closed_loop",
                        "_initial_zone_counts": grid_zone_counts,
                    },
                    selected_detection_ids=[candidate_ids[0]],
                )
                metadata = plan.get("task_preview", {})
                update_task_execution(
                    ignored_objects=metadata.get("ignored_detections", []),
                    candidate_objects=metadata.get("candidate_objects", []),
                    warnings=metadata.get("warnings", []),
                    current_object=metadata.get("next_object"),
                )
                await broadcast_state()

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
                finish_task_execution("failed", reason)
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
            if (
                completed < int(task_settings.get("max_objects", 1))
                and task_settings.get("cycle_confirmation") == "confirm_each_object"
            ):
                await wait_for_task_confirmation(run_id)

        terminal_reason = "max_objects reached"
        finish_task_execution("completed", terminal_reason)
        await broadcast_state()
    except asyncio.CancelledError:
        finish_task_execution("stopped", "task cancelled")
        await broadcast_state()
        raise
    finally:
        task_selection_events.pop(run_id, None)
        task_selection_choices.pop(run_id, None)
        task_confirmation_events.pop(run_id, None)


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
    state.closed_loop_mode = str(encoder_settings(config).get("mode", "diagnostic"))
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
    latest_vision_snapshot.clear()
    state.config_change = {
        **config_change,
        "applied_at": time(),
    }
    if config_change.get("encoder_measurement_invalidated"):
        state.update_encoder_evidence(
            [
                empty_evidence(index + 1, joint.name)
                for index, joint in enumerate(config.joints)
            ]
        )
        state.encoder_available = "0000"
        state.encoder_angles_deg = [None] * len(config.joints)
        state.encoder_errors_deg = [None] * len(config.joints)
        state.encoder_fault = False
        state.encoder_mismatch = {}
        encoder_calibration_sessions.clear()
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
    state.encoder_mismatch = {
        **state.encoder_mismatch,
        "last_controller_estimated_deg": reported_angles.copy(),
        "last_controller_estimated_at": time(),
    }
    known_pose = status.known_pose
    pose_source = status.pose_source
    legacy_encoder_authority = (
        "known_mask=" not in status_line
        and pose_source in {"encoder", "mixed"}
    )
    if legacy_encoder_authority and not status.homed:
        trusted_existing_estimate = (
            state.known_pose
            and state.pose_source in {"setpose", "open_loop_estimate", "manual", "home"}
        )
        known_pose = trusted_existing_estimate
        status_known_mask = state.pose_known_mask if trusted_existing_estimate else "0000"
        pose_source = state.pose_source if trusted_existing_estimate else "unknown"
    else:
        status_known_mask = status.known_mask
    state.hardware_armed = status.armed
    if status.hardware_mode != "unknown":
        state.hardware_mode = status.hardware_mode
    if status.enabled_axes:
        state.hardware_enabled_axes = status.enabled_axes
    state.encoder_available = status.encoder_available
    known_mask = status_known_mask
    if state.encoder_fault and len(known_mask) >= 2:
        known_mask = f"{known_mask[0]}0{known_mask[2:]}"
        known_pose = False
    state.update_reported_pose(
        reported_angles,
        source=pose_source,
        known_pose=known_pose,
        known_mask=known_mask,
    )
    state.closed_loop_mode = status.closed_loop_mode
    if status.tool_type != "unknown":
        state.tool_type = status.tool_type
    state.tool_state = status.tool_state
    if status.tool_value is not None:
        state.tool_value = status.tool_value
    if not state.encoder_fault and status.state in {item.value for item in MotionState}:
        state.motion_state = MotionState(status.state)
    if not state.encoder_fault:
        state.last_error = "" if status.fault == "OK" else status.fault
    update_encoder_evidence(status)
    state.correction_state = {
        "state": status.correction_state,
        "transaction_id": status.correction_transaction_id,
        "requested_delta_deg": status.correction_requested_delta_deg,
        "emitted_steps": status.correction_emitted_steps,
        "attempts": status.correction_attempts,
        "bias_deg": status.correction_bias_deg or [None] * len(config.joints),
    }
    if (
        state.config_sync_status == "synced"
        and _encoder_runtime_should_be_active()
        and status.closed_loop_mode == "off"
    ):
        message = _encoder_runtime_off_message()
        state.config_sync_status = "stale"
        state.config_sync_message = message
        state.encoder_mismatch = {
            **state.encoder_mismatch,
            "status": "encoder_config_inactive",
            "message": message,
            "checked_at": time(),
        }
        log_event("controller", "encoder runtime became inactive", status_line=status_line)
    pose_tracked_from_encoder = apply_shoulder_encoder_pose_tracking(status.state)
    if (
        not pose_tracked_from_encoder
        and state.encoder_mismatch.get("controller_pose_rebase_required")
        and shoulder_controller_rebase_applicable()
        and len(reported_angles) > 1
        and len(state.reported_angles_deg) > 1
        and abs(float(state.reported_angles_deg[1]) - float(reported_angles[1])) <= controller_rebase_tolerance_deg()
    ):
        update_controller_rebase_state(required=False)
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


def refresh_idle_planning_pose_from_hardware() -> str | None:
    if state.simulation or not serial_client.is_connected:
        return None
    if state.motion_state not in {MotionState.IDLE, MotionState.STOPPED}:
        return None
    try:
        refresh_serial_status()
    except SerialClientError as exc:
        return str(exc)
    return None


def hardware_trajectory_start_blocking_reason(preview: dict[str, Any] | None = None) -> str | None:
    if state.simulation or not serial_client.is_connected:
        return None
    try:
        refresh_serial_status()
    except SerialClientError as exc:
        return str(exc)
    if state.motion_state == MotionState.MOVING:
        return "controller is still moving; wait for STATUS state=idle or press Stop before starting a trajectory"
    rebase_reason = controller_pose_rebase_blocking_reason()
    if rebase_reason:
        return rebase_reason
    if preview is not None and "start_pose_revision" in preview:
        rebase_preview_start_to_current_if_encoder_tracked(preview)
        stale_reason = preview_stale_reason(preview)
        if stale_reason:
            return stale_reason
    alignment_reason = shoulder_alignment_motion_blocking_reason()
    if alignment_reason:
        return alignment_reason
    return None


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


@app.get("/api/programs")
async def get_programs() -> dict[str, Any]:
    try:
        programs = all_programs(config)
    except ProgramLibraryError as exc:
        return {"ok": False, "error": str(exc), "programs": []}
    return {
        "ok": True,
        "schema_version": PROGRAM_SCHEMA_VERSION,
        "programs": [public_program_record(program) for program in programs],
    }


@app.get("/api/programs/{program_id}")
async def get_program(program_id: str) -> dict[str, Any]:
    try:
        program = find_program(config, program_id)
    except ProgramLibraryError as exc:
        return {"ok": False, "error": str(exc)}
    if program is None:
        return {"ok": False, "error": f"program {program_id} was not found"}
    return {"ok": True, "program": public_program_record(program)}


@app.post("/api/programs")
async def create_or_update_program(request: ProgramSaveRequest) -> dict[str, Any]:
    try:
        program = save_user_program(config, request.__dict__)
    except ProgramLibraryError as exc:
        return {"ok": False, "error": str(exc)}
    log_event("program", "program saved", program_id=program["id"], name=program["name"])
    return {"ok": True, "program": public_program_record(program)}


@app.put("/api/programs/{program_id}")
async def update_program(program_id: str, request: ProgramSaveRequest) -> dict[str, Any]:
    try:
        existing = find_program(config, program_id)
        if existing is None:
            return {"ok": False, "error": f"program {program_id} was not found"}
        if existing.get("read_only"):
            return {"ok": False, "error": "built-in templates are read-only; copy the template first"}
        program = save_user_program(config, {**request.__dict__, "id": program_id})
    except ProgramLibraryError as exc:
        return {"ok": False, "error": str(exc)}
    log_event("program", "program updated", program_id=program["id"], name=program["name"])
    return {"ok": True, "program": public_program_record(program)}


@app.delete("/api/programs/{program_id}")
async def remove_program(program_id: str) -> dict[str, Any]:
    try:
        removed = delete_user_program(config, program_id)
    except ProgramLibraryError as exc:
        return {"ok": False, "error": str(exc)}
    if not removed:
        return {"ok": False, "error": f"program {program_id} was not found"}
    log_event("program", "program deleted", program_id=program_id)
    return {"ok": True, "program_id": program_id}


@app.post("/api/programs/{program_id}/copy")
async def copy_program(program_id: str, request: ProgramCopyRequest) -> dict[str, Any]:
    try:
        program = copy_program_to_user(config, program_id, name=request.name)
    except ProgramLibraryError as exc:
        return {"ok": False, "error": str(exc)}
    log_event(
        "program",
        "program copied",
        program_id=program["id"],
        copied_from=program_id,
        name=program["name"],
    )
    return {"ok": True, "program": public_program_record(program)}


@app.post("/api/programs/{program_id}/restore-plan")
async def restore_program_plan(
    program_id: str,
    request: ProgramRestorePlanRequest,
) -> dict[str, Any]:
    try:
        program = find_program(config, program_id)
    except ProgramLibraryError as exc:
        return {"ok": False, "cache_miss": True, "error": str(exc)}
    if program is None:
        return {
            "ok": False,
            "cache_miss": True,
            "error": f"program {program_id} was not found",
        }
    return restore_cached_program_preview(program, request.program_revision)


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
        "model_truth": model_truth_summary(config, state.fk),
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


@app.get("/api/position-library")
async def get_position_library() -> dict[str, Any]:
    positions = named_positions(config)
    library = position_library_records(config, positions)
    return {
        "ok": True,
        "schema_version": POSITION_LIBRARY_SCHEMA_VERSION,
        "positions": library,
        "errors": position_library_errors(config, library),
    }


@app.post("/api/position-library")
async def save_position_library(request: PositionLibraryRequest) -> dict[str, Any]:
    normalized: dict[str, dict[str, Any]] = {}
    errors: dict[str, list[str]] = {}
    existing_records = position_library_records(config, include_legacy=False)
    saved_at = datetime.now(timezone.utc).isoformat()
    for position_id, raw_position in request.positions.items():
        try:
            record = normalize_position_record(config, str(position_id), raw_position)
        except PositionLibraryError as exc:
            errors[str(position_id)] = [str(exc)]
            continue
        if record["id"] != str(position_id):
            errors[str(position_id)] = [
                f"position id {record['id']} must match its stable library key {position_id}"
            ]
            continue
        if record["id"] in normalized:
            errors[str(position_id)] = [f"duplicate position id {record['id']}"]
            continue
        validation = validate_position_record(config, str(position_id), record)
        if validation:
            errors[str(position_id)] = validation
            continue
        existing = existing_records.get(record["id"], {})
        record["created_at"] = str(existing.get("created_at") or saved_at)
        record["updated_at"] = saved_at
        normalized[record["id"]] = record

    if errors:
        first_position_id, first_errors = next(iter(errors.items()))
        error = f"{first_position_id}: {'; '.join(first_errors)}"
        state.set_error(error)
        await broadcast_state()
        return {"ok": False, "error": error, "errors": errors, "state": state.to_dict()}

    library_payload = {
        "schema_version": POSITION_LIBRARY_SCHEMA_VERSION,
        "positions": normalized,
    }
    legacy_positions = {
        position_id: position_record_to_legacy_position(record)
        for position_id, record in normalized.items()
    }
    updates = {
        "position_library": library_payload,
        "named_positions": legacy_positions,
    }
    try:
        config_path = ensure_local_config()
        with tempfile.TemporaryDirectory() as tmp_dir:
            draft_path = Path(tmp_dir) / "robot.local.yaml"
            shutil.copyfile(config_path, draft_path)
            save_calibration_updates(draft_path, updates)
            draft_config = load_config(draft_path)
            destination_errors = task_destination_errors(draft_config, named_positions(draft_config))
            if destination_errors:
                state.set_error("position library changes would break task destination references")
                await broadcast_state()
                return {
                    "ok": False,
                    "errors": {"task_destinations": destination_errors},
                    "error": state.last_error,
                    "state": state.to_dict(),
                }
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
        state.set_error(f"could not save position library: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    positions = named_positions(config)
    library = position_library_records(config, positions)
    log_event("config", "position library saved", count=len(library))
    await broadcast_state()
    return {
        "ok": True,
        "schema_version": POSITION_LIBRARY_SCHEMA_VERSION,
        "positions": library,
        "errors": position_library_errors(config, library),
        "config": public_config(),
        "state": state.to_dict(),
    }


@app.post("/api/named-positions")
async def save_named_positions(request: NamedPositionsRequest) -> dict[str, Any]:
    result = await save_position_library(PositionLibraryRequest(positions=request.positions))
    if not result.get("ok"):
        return result
    log_event("config", "legacy named positions saved through position library", count=len(request.positions))
    return {
        "ok": True,
        "positions": named_positions(config),
        "config": result["config"],
        "state": result["state"],
    }


def _task_destinations_from_request(raw: dict[str, Any]) -> dict[str, dict[str, Any]]:
    nested = raw.get("destinations")
    if "destinations" in raw and not isinstance(nested, dict):
        raise ValueError("task_destinations.destinations must be an object")
    source = nested if isinstance(nested, dict) else raw
    destinations: dict[str, dict[str, Any]] = {}
    for destination_id, destination in source.items():
        if destination_id in {"schema_version", "updated_at"}:
            continue
        if not isinstance(destination, dict):
            raise ValueError(f"task destination {destination_id} must be an object")
        destinations[str(destination_id)] = deepcopy(destination)
    return destinations


def _persisted_color_profiles(profiles: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return {
        str(name): {
            key: deepcopy(value)
            for key, value in profile.items()
            if key != "draft"
        }
        for name, profile in profiles.items()
    }


@app.post("/api/task-mappings")
async def save_task_mappings(request: TaskMappingsRequest) -> dict[str, Any]:
    config_path = ensure_local_config()
    try:
        destinations = _task_destinations_from_request(request.task_destinations)
    except ValueError as exc:
        state.set_error(str(exc))
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    profiles = _persisted_color_profiles(request.color_profiles)
    initial_updates = {
        "color_profiles": profiles,
        "task_destinations": task_destination_payload(destinations),
        "drop_zones": legacy_drop_zones_from_task_destinations(destinations),
    }
    try:
        with tempfile.TemporaryDirectory() as tmp_dir:
            draft_path = Path(tmp_dir) / "robot.local.yaml"
            shutil.copyfile(config_path, draft_path)
            save_calibration_updates(draft_path, initial_updates)
            draft_config = load_config(draft_path)
            destination_errors = task_destination_errors(draft_config, named_positions(draft_config))
            if destination_errors:
                state.set_error("one or more task destinations are invalid")
                await broadcast_state()
                return {
                    "ok": False,
                    "errors": destination_errors,
                    "error": state.last_error,
                    "state": state.to_dict(),
                }
            resolved_destinations = resolve_task_destinations(draft_config, named_positions(draft_config))
            final_updates = {
                **initial_updates,
                "drop_zones": legacy_drop_zones_from_task_destinations(resolved_destinations),
            }
            save_calibration_updates(draft_path, final_updates)
            draft_config = load_config(draft_path)
            change = classify_config_change(config, draft_config)
            ready, reason = config_change_ready(change)
            if not ready:
                state.set_error(reason)
                await broadcast_state()
                return {"ok": False, "error": reason, "config_change": change, "state": state.to_dict()}

        save_calibration_updates(config_path, final_updates)
        reload_runtime_config(load_config(config_path), change)
        log_validation_warnings()
    except Exception as exc:
        state.set_error(f"could not save task mappings: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    state.last_command = "SAVE_TASK_MAPPINGS"
    state.clear_error()
    log_event("config", "task mappings saved", destinations=len(destinations), colors=len(profiles))
    await broadcast_state()
    return {
        "ok": True,
        "config": public_config(),
        "config_change": state.config_change,
        "state": state.to_dict(),
    }


@app.post("/api/tool")
async def tool_command(request: ToolRequest) -> dict[str, Any]:
    return await apply_tool_action(request.action, request.value, request.tool)


@app.get("/api/tools")
async def get_tools() -> dict[str, Any]:
    return {"ok": True, "tools": tools_settings(config), "active": tools_settings(config).get("active", "gripper")}


@app.post("/api/tools")
async def save_tools(request: ToolsRequest) -> dict[str, Any]:
    previous_tools = tools_settings(config)
    tools = deepcopy(previous_tools)
    tools["active"] = request.active
    if request.presets:
        presets = tools.setdefault("presets", {})
        for name, preset in request.presets.items():
            merged = presets.get(name, {})
            merged.update(preset)
            presets[name] = merged
    _invalidate_changed_tool_validations(previous_tools, tools)
    errors = validate_tools_payload(tools)
    if errors:
        state.set_error("; ".join(errors))
        await broadcast_state()
        return {"ok": False, "errors": errors, "error": state.last_error, "state": state.to_dict()}
    try:
        calibration = calibration_settings(config)
        calibration["tool_dimensions_validated"] = _active_tool_validation_from_payload(
            tools,
            bool(calibration.get("tool_dimensions_validated", False)),
        )
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
    destination_errors = task_destination_errors(config, named_positions(config))
    try:
        destinations = drop_zones(config)
    except TaskDestinationError:
        destinations = {}
    return {
        "ok": True,
        "camera": camera_settings(config),
        "color_profiles": color_profiles(config),
        "drop_zones": legacy_drop_zones_from_task_destinations(destinations),
        "task_destinations": task_destination_payload(destinations),
        "task_destination_errors": destination_errors,
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
        if isinstance(updates.get("color_profiles"), dict):
            updates["color_profiles"] = _persisted_color_profiles(updates["color_profiles"])
        task_destinations_changed = _coerce_task_destination_updates(updates)
        if task_destinations_changed:
            with tempfile.TemporaryDirectory() as tmp_dir:
                draft_path = Path(tmp_dir) / "robot.local.yaml"
                shutil.copyfile(ensure_local_config(), draft_path)
                save_calibration_updates(draft_path, updates)
                draft_config = load_config(draft_path)
                destination_errors = task_destination_errors(draft_config, named_positions(draft_config))
                if destination_errors:
                    state.set_error("one or more task destinations are invalid")
                    await broadcast_state()
                    return {
                        "ok": False,
                        "errors": destination_errors,
                        "error": state.last_error,
                        "state": state.to_dict(),
                    }
        save_calibration_updates(ensure_local_config(), updates)
        reload_runtime_config()
    except ValueError as exc:
        state.set_error(str(exc))
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
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
    return register_vision_snapshot({
        "ok": True,
        "captured_at": time(),
        "detections": result["detections"],
        "workspace": result["workspace"],
        "provider": result["provider"],
        "calibration_source": result["calibration_source"],
    })


@app.get("/api/vision/frame")
async def get_vision_frame() -> dict[str, Any]:
    try:
        if state.simulation:
            return register_vision_snapshot(simulation_vision_payload(consume=False))
        camera = camera_settings(config)
        image = await asyncio.to_thread(capture_camera_frame, camera)
        result = vision_pipeline.process(
            image,
            camera,
            color_profiles(config),
        )
        return register_vision_snapshot({
            "ok": True,
            "captured_at": time(),
            "raw_image_b64": encode_image_b64(image),
            "image_b64": encode_image_b64(result["annotated"]),
            "detections": result["detections"],
            "workspace": result["workspace"],
            "provider": result["provider"],
            "calibration_source": result["calibration_source"],
        })
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
    latest_vision_snapshot.clear()
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
    latest_vision_snapshot.clear()
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
    if (
        requested
        and not state.simulation
        and serial_client.is_connected
        and state.config_sync_status == "synced"
        and shoulder_controller_rebase_applicable()
    ):
        try:
            refresh_serial_status()
            synced, sync_reason = sync_controller_pose_to_encoder_tracked_pose_if_needed()
            if not synced:
                state.hardware_armed = False
                state.set_error(sync_reason)
                await broadcast_state()
                return {"ok": False, "error": sync_reason, "state": state.to_dict()}
        except SerialClientError as exc:
            state.hardware_armed = False
            state.set_error(str(exc), fault=True)
            await broadcast_state()
            return {"ok": False, "error": str(exc), "state": state.to_dict()}
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
    if state.encoder_fault:
        state.set_error("clear the latched encoder fault before Set Pose")
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


@app.post("/api/encoder/fault/clear")
async def clear_encoder_fault(request: EncoderFaultClearRequest) -> dict[str, Any]:
    if not state.encoder_fault:
        return {"ok": True, "state": state.to_dict()}
    if state.simulation:
        state.set_error("encoder fault clearing is a hardware-only operation")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if state.hardware_armed:
        state.set_error("disarm hardware before clearing an encoder fault")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if state.motion_state == MotionState.ESTOP:
        state.set_error("clear ESTOP before clearing the encoder fault")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not request.acknowledge_pose_unknown:
        state.set_error("acknowledge that the shoulder planning pose remains unknown until Set Pose")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    state.encoder_fault = False
    state.encoder_mismatch = {
        **state.encoder_mismatch,
        "status": "cleared",
        "cleared_at": time(),
        "requires_setpose": True,
    }
    state.motion_state = MotionState.STOPPED
    state.last_error = ""
    state.last_command = "ENCODER_FAULT_CLEAR"
    log_event(
        "encoder",
        "latched shoulder encoder fault cleared; Set Pose is still required",
    )
    await broadcast_state()
    return {"ok": True, "requires_setpose": True, "state": state.to_dict()}


@app.post("/api/encoder/calibration/quick")
async def quick_calibrate_encoder(request: EncoderQuickCalibrationRequest) -> dict[str, Any]:
    ready, reason = encoder_calibration_ready()
    if not ready:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    if not request.confirm_one_to_one_output_mount:
        return {
            "ok": False,
            "error": (
                "quick calibration assumes the AS5048A is mounted at the shoulder output with "
                "one sensor turn per joint turn; explicit confirmation is required"
            ),
            "state": state.to_dict(),
        }
    if request.mounting_location != "joint_output":
        return {
            "ok": False,
            "error": "quick calibration and backlash correction require joint-output encoder mounting",
            "state": state.to_dict(),
        }
    joint_angle, reason = _validate_encoder_known_joint_angle(request.joint_angle_deg)
    if joint_angle is None:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    direction_sign = int(request.direction_sign)
    if direction_sign not in {-1, 1}:
        return {"ok": False, "error": "direction sign must be +1 or -1", "state": state.to_dict()}
    turns = float(request.sensor_turns_per_joint_turn)
    if not isfinite(turns) or turns <= 0.0:
        return {"ok": False, "error": "sensor turns per joint turn must be positive", "state": state.to_dict()}
    sample, reason = current_raw_shoulder_sample()
    if sample is None:
        return {"ok": False, "error": reason, "state": state.to_dict()}

    settings = encoder_settings(config)
    shoulder = encoder_axis(settings)
    if shoulder is None:
        return {"ok": False, "error": "shoulder encoder configuration is missing", "state": state.to_dict()}
    calibration_id = str(uuid4())
    shoulder.update(
        {
            "reference_raw_deg": sample["raw_angle_deg"],
            "reference_joint_deg": joint_angle,
            "direction_sign": direction_sign,
            "sensor_turns_per_joint_turn": turns,
            "mounting_location": request.mounting_location,
            "reference_description": request.reference_description.strip(),
            "calibration_validated": True,
            "calibration_id": calibration_id,
            "calibration_validated_at": datetime.now(timezone.utc).isoformat(),
            "calibration_model": "single_point",
            "calibration_map": [],
            "calibration_map_extrapolate_deg": 2.0,
            "fit_max_residual_deg": None,
            "backlash_estimate_deg": None,
            "backlash_approach_bias_deg": None,
            "localized_backlash_estimate_deg": None,
            "localized_backlash_at_joint_deg": None,
        }
    )
    settings["mode"] = "diagnostic"
    settings.setdefault("correction", {})
    settings["correction"]["enabled"] = False
    settings["correction"]["validation_id"] = ""
    try:
        change = persist_encoder_calibration(settings)
    except Exception as exc:
        state.set_error(f"could not save quick shoulder encoder calibration: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    state.last_command = "QUICK_SHOULDER_ENCODER_CALIBRATION"
    state.clear_error()
    validation = {
        "ok": True,
        "fit_model": "single_point",
        "reference_raw_deg": sample["raw_angle_deg"],
        "reference_joint_deg": joint_angle,
        "direction_sign": direction_sign,
        "sensor_turns_per_joint_turn": turns,
        "sample_count": 1,
        "fit_sample_count": 1,
        "warnings": [
            "quick calibration uses one known shoulder angle plus the configured direction and 1:1 scale; "
            "run backlash check or assisted sweep to verify the range"
        ],
        "errors": [],
    }
    log_event(
        "encoder",
        "quick shoulder encoder calibration committed",
        calibration_id=calibration_id,
        reference_raw_deg=sample["raw_angle_deg"],
        reference_joint_deg=joint_angle,
        direction_sign=direction_sign,
        sensor_turns_per_joint_turn=turns,
    )
    await broadcast_state()
    return {
        "ok": True,
        "calibration_id": calibration_id,
        "sample": sample,
        "validation": validation,
        "config_change": change,
        "config": public_config(),
        "state": state.to_dict(),
    }


@app.post("/api/encoder/backlash/check")
async def check_encoder_backlash(request: EncoderBacklashCheckRequest) -> dict[str, Any]:
    return await run_encoder_backlash_check(request)


@app.post("/api/encoder/calibration/start")
async def start_encoder_calibration(request: EncoderCalibrationStartRequest) -> dict[str, Any]:
    ready, reason = encoder_calibration_ready()
    if not ready:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    if request.mounting_location not in {"joint_output", "gearbox_input", "motor_shaft"}:
        return {"ok": False, "error": "unsupported encoder mounting location", "state": state.to_dict()}
    initial_joint_angle: float | None = None
    if request.capture_initial:
        initial_joint_angle, reason = _validate_encoder_known_joint_angle(request.joint_angle_deg)
        if initial_joint_angle is None:
            return {"ok": False, "error": reason, "state": state.to_dict()}
    sample, reason = current_raw_shoulder_sample()
    if sample is None:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    session_id = str(uuid4())
    session = {
        "id": session_id,
        "joint": 2,
        "created_at": time(),
        "mounting_location": request.mounting_location,
        "reference_description": request.reference_description.strip(),
        "samples": [],
        "initial_raw_sample": sample,
    }
    captured = None
    validation = None
    if request.capture_initial and initial_joint_angle is not None:
        captured = _encoder_calibration_capture(sample, initial_joint_angle, "reference 1")
        session["samples"].append(captured)
        validation = validate_encoder_calibration_session(session)
    encoder_calibration_sessions[session_id] = session
    log_event(
        "encoder",
        "shoulder encoder calibration session started",
        session_id=session_id,
        mounting_location=request.mounting_location,
        initial_sample_captured=bool(captured),
    )
    response = {
        "ok": True,
        "session": deepcopy(encoder_calibration_sessions[session_id]),
        "state": state.to_dict(),
    }
    if captured is not None:
        response["sample"] = captured
    if validation is not None:
        response["validation"] = validation
    return response


@app.post("/api/encoder/calibration/sample")
async def capture_encoder_calibration_sample(request: EncoderCalibrationSampleRequest) -> dict[str, Any]:
    ready, reason = encoder_calibration_ready()
    if not ready:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    session = encoder_calibration_sessions.get(request.session_id)
    if session is None:
        return {"ok": False, "error": "encoder calibration session not found", "state": state.to_dict()}
    if time() - float(session.get("created_at", 0.0)) > 1800.0:
        encoder_calibration_sessions.pop(request.session_id, None)
        return {"ok": False, "error": "encoder calibration session expired", "state": state.to_dict()}
    joint_angle, reason = _validate_encoder_known_joint_angle(request.joint_angle_deg)
    if joint_angle is None:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    sample, reason = current_raw_shoulder_sample()
    if sample is None:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    captured = _encoder_calibration_capture(sample, joint_angle, request.label)
    session["samples"].append(captured)
    validation = validate_encoder_calibration_session(session)
    log_event(
        "encoder",
        "shoulder encoder calibration sample captured",
        session_id=request.session_id,
        sample_count=len(session["samples"]),
        raw_angle_deg=sample["raw_angle_deg"],
        joint_angle_deg=joint_angle,
    )
    return {
        "ok": True,
        "sample": captured,
        "session": deepcopy(session),
        "validation": validation,
        "state": state.to_dict(),
    }


@app.post("/api/encoder/calibration/validate")
async def validate_encoder_calibration(request: EncoderCalibrationSessionRequest) -> dict[str, Any]:
    session = encoder_calibration_sessions.get(request.session_id)
    if session is None:
        return {"ok": False, "error": "encoder calibration session not found", "state": state.to_dict()}
    validation = validate_encoder_calibration_session(session)
    return {
        "ok": validation["ok"],
        "validation": validation,
        "session": deepcopy(session),
        "error": "; ".join(validation["errors"]) if validation["errors"] else "",
        "state": state.to_dict(),
    }


@app.post("/api/encoder/calibration/commit")
async def commit_encoder_calibration(request: EncoderCalibrationCommitRequest) -> dict[str, Any]:
    ready, reason = encoder_calibration_ready()
    if not ready:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    if not request.confirm:
        return {
            "ok": False,
            "error": "explicit confirmation is required to commit shoulder encoder calibration",
            "state": state.to_dict(),
        }
    session = encoder_calibration_sessions.get(request.session_id)
    if session is None:
        return {"ok": False, "error": "encoder calibration session not found", "state": state.to_dict()}
    validation = validate_encoder_calibration_session(session)
    if not validation["ok"]:
        return {
            "ok": False,
            "error": "; ".join(validation["errors"]),
            "validation": validation,
            "state": state.to_dict(),
        }

    settings = encoder_settings(config)
    shoulder = encoder_axis(settings)
    if shoulder is None:
        return {"ok": False, "error": "shoulder encoder configuration is missing", "state": state.to_dict()}
    calibration_id = str(uuid4())
    shoulder.update(
        {
            "reference_raw_deg": validation["reference_raw_deg"],
            "reference_joint_deg": validation["reference_joint_deg"],
            "direction_sign": validation["direction_sign"],
            "sensor_turns_per_joint_turn": validation["sensor_turns_per_joint_turn"],
            "mounting_location": session["mounting_location"],
            "reference_description": session.get("reference_description", ""),
            "calibration_validated": True,
            "calibration_id": calibration_id,
            "calibration_validated_at": datetime.now(timezone.utc).isoformat(),
            "calibration_model": validation.get("fit_model", "linear"),
            "calibration_map": validation.get("calibration_map") or [],
            "calibration_map_extrapolate_deg": 2.0,
            "fit_max_residual_deg": validation.get("max_residual_deg"),
            "backlash_estimate_deg": validation.get("backlash_estimate_deg"),
            "backlash_approach_bias_deg": validation.get("approach_bias_deg"),
            "localized_backlash_estimate_deg": validation.get("localized_backlash_estimate_deg"),
            "localized_backlash_at_joint_deg": validation.get("localized_backlash_at_joint_deg"),
        }
    )
    settings["mode"] = "diagnostic"
    settings["correction"]["enabled"] = False
    settings["correction"]["validation_id"] = ""
    try:
        change = persist_encoder_calibration(settings)
    except Exception as exc:
        state.set_error(f"could not save shoulder encoder calibration: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    encoder_calibration_sessions.pop(request.session_id, None)
    state.last_command = "SAVE_SHOULDER_ENCODER_CALIBRATION"
    state.clear_error()
    log_event(
        "encoder",
        "shoulder encoder calibration committed",
        calibration_id=calibration_id,
        mounting_location=session["mounting_location"],
        direction_sign=validation["direction_sign"],
        sensor_turns_per_joint_turn=validation["sensor_turns_per_joint_turn"],
    )
    await broadcast_state()
    return {
        "ok": True,
        "calibration_id": calibration_id,
        "validation": validation,
        "config_change": change,
        "config": public_config(),
        "state": state.to_dict(),
    }


@app.post("/api/encoder/calibration/sweep/start")
async def start_encoder_calibration_sweep(request: EncoderCalibrationSweepStartRequest) -> dict[str, Any]:
    global encoder_calibration_sweep_task
    if not request.confirm_open_loop_sweep:
        return {
            "ok": False,
            "error": (
                "assisted sweep uses normal open-loop planned shoulder targets after the first "
                "physical reference; explicit confirmation is required"
            ),
            "state": state.to_dict(),
        }
    if encoder_calibration_sweep_task is not None and not encoder_calibration_sweep_task.done():
        return {"ok": False, "error": "an assisted encoder sweep is already running", "state": state.to_dict()}
    ready, reason = encoder_calibration_sweep_ready()
    if not ready:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    if request.mounting_location not in {"joint_output", "gearbox_input", "motor_shaft"}:
        return {"ok": False, "error": "unsupported encoder mounting location", "state": state.to_dict()}
    start_angle, reason = _validate_encoder_known_joint_angle(request.start_joint_angle_deg)
    if start_angle is None:
        return {"ok": False, "error": reason, "state": state.to_dict()}
    planning_start = float(state.reported_angles_deg[1])
    if abs(planning_start - start_angle) > 0.25:
        return {
            "ok": False,
            "error": (
                "current planning shoulder angle does not match the known sweep start "
                f"({planning_start:.2f} deg planning vs {start_angle:.2f} deg entered). "
                "Disarm and Set Pose to the known start angle before running the sweep."
            ),
            "state": state.to_dict(),
        }
    try:
        final_approach_direction = int(request.final_approach_direction)
        preload_deg = float(request.preload_deg)
        targets = _encoder_unidirectional_sweep_targets(
            sweep_min_deg=float(request.sweep_min_deg),
            sweep_max_deg=float(request.sweep_max_deg),
            step_deg=float(request.step_deg),
            final_approach_direction=final_approach_direction,
        )
        preload_target = _encoder_sweep_preload_target(
            targets[0],
            final_approach_direction=final_approach_direction,
            preload_deg=preload_deg,
        )
        settings = _encoder_sweep_path_settings(
            float(request.speed_deg_s),
            float(request.accel_deg_s2),
        )
        settle_ms = int(request.settle_ms)
        if settle_ms < 100 or settle_ms > 5000:
            raise ValueError("settle time must be between 100 and 5000 ms")
    except ValueError as exc:
        return {"ok": False, "error": str(exc), "state": state.to_dict()}
    sample, reason = current_raw_shoulder_sample()
    if sample is None:
        return {"ok": False, "error": reason, "state": state.to_dict()}

    session_id = str(uuid4())
    captured = _encoder_calibration_capture(sample, start_angle, "start check", use_for_fit=False)
    session = {
        "id": session_id,
        "joint": 2,
        "created_at": time(),
        "mounting_location": request.mounting_location,
        "reference_description": request.reference_description.strip(),
        "samples": [captured],
        "initial_raw_sample": sample,
        "assisted_sweep": True,
        "sweep": {
            "status": "queued",
            "mode": "same_direction_preloaded",
            "targets_deg": targets,
            "final_approach_direction": final_approach_direction,
            "preload_deg": preload_deg,
            "preload_target_deg": preload_target,
            "settle_ms": settle_ms,
            "path_settings": settings,
            "started_at": time(),
            "completed": 0,
            "total": len(targets),
            "note": (
                "preloads backlash once, then captures stopped samples from one final approach direction; "
                "the start sample is a sanity check and is not used for the fit"
            ),
        },
    }
    session["validation"] = validate_encoder_calibration_session(session)
    encoder_calibration_sessions[session_id] = session
    encoder_calibration_sweep_task = asyncio.create_task(run_encoder_calibration_sweep(session_id))
    log_event(
        "encoder",
        "assisted shoulder encoder sweep queued",
        session_id=session_id,
        start_joint_angle_deg=start_angle,
        target_count=len(targets),
    )
    await broadcast_state()
    return {
        "ok": True,
        "session": deepcopy(session),
        "validation": session["validation"],
        "state": state.to_dict(),
    }


@app.get("/api/encoder/calibration/session/{session_id}")
async def get_encoder_calibration_session(session_id: str) -> dict[str, Any]:
    session = encoder_calibration_sessions.get(session_id)
    if session is None:
        return {"ok": False, "error": "encoder calibration session not found", "state": state.to_dict()}
    validation = validate_encoder_calibration_session(session)
    session["validation"] = validation
    return {
        "ok": True,
        "session": deepcopy(session),
        "validation": validation,
        "state": state.to_dict(),
    }


@app.post("/api/encoder/calibration/sweep/cancel")
async def cancel_encoder_calibration_sweep(request: EncoderCalibrationSweepSessionRequest) -> dict[str, Any]:
    global encoder_calibration_sweep_task
    session = encoder_calibration_sessions.get(request.session_id)
    if session is None:
        return {"ok": False, "error": "encoder calibration session not found", "state": state.to_dict()}
    _set_encoder_sweep_status(session, cancel_requested=True, status="cancel_requested")
    task = encoder_calibration_sweep_task
    if task is not None and not task.done():
        task.cancel()
    if state.motion_state == MotionState.MOVING:
        await stop()
    await broadcast_state()
    return {
        "ok": True,
        "session": deepcopy(session),
        "validation": validate_encoder_calibration_session(session),
        "state": state.to_dict(),
    }


@app.post("/api/encoder/correction/policy")
async def set_encoder_correction_policy(request: EncoderCorrectionPolicyRequest) -> dict[str, Any]:
    if request.enabled and not request.confirm:
        return {
            "ok": False,
            "error": "explicit confirmation is required to enable bounded shoulder correction",
            "state": state.to_dict(),
        }
    settings = encoder_settings(config)
    settings.setdefault("correction", {})
    settings.setdefault("verification", {})
    if request.enabled:
        ok, reason, details = await validate_encoder_correction_enablement()
        if not ok:
            return {"ok": False, "error": reason, "details": details, "state": state.to_dict()}
        settings["correction"]["enabled"] = True
        settings["correction"]["validation_id"] = f"shoulder-correction-{uuid4()}"
        settings["mode"] = "bounded_correction"
        if str(settings["verification"].get("policy", "diagnostic")) == "diagnostic":
            settings["verification"]["policy"] = "warning"
    else:
        if not request.confirm:
            return {
                "ok": False,
                "error": "explicit confirmation is required to change bounded shoulder correction",
                "state": state.to_dict(),
            }
        settings["correction"]["enabled"] = False
        settings["correction"]["validation_id"] = ""
        settings["mode"] = (
            "diagnostic"
            if str(settings["verification"].get("policy", "diagnostic")) == "diagnostic"
            else "verification"
        )
        details = {"disabled": True}
    try:
        change = persist_encoder_calibration(settings)
    except Exception as exc:
        state.set_error(f"could not save shoulder encoder correction policy: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    state.last_command = "SAVE_SHOULDER_ENCODER_CORRECTION_POLICY"
    state.clear_error()
    log_event(
        "encoder",
        "shoulder encoder bounded correction policy changed",
        enabled=bool(settings["correction"].get("enabled")),
        validation_id=settings["correction"].get("validation_id"),
    )
    await broadcast_state()
    return {
        "ok": True,
        "enabled": bool(settings["correction"].get("enabled")),
        "details": details,
        "config_change": change,
        "config": public_config(),
        "state": state.to_dict(),
    }


async def align_shoulder_to_planning_internal(
    request: EncoderShoulderAlignRequest | None = None,
    *,
    allow_active_execution: bool = False,
    motion_source: str = "encoder_shoulder_align",
    automatic: bool = False,
    target_shoulder_deg: float | None = None,
) -> dict[str, Any]:
    """Run bounded shoulder-only alignment against the current logical target."""
    settings, shoulder, reason = _encoder_runtime_ready()
    if reason:
        state.set_error(reason)
        await broadcast_state()
        return {"ok": False, "error": reason, "state": state.to_dict()}
    if shoulder is None or settings is None:
        state.set_error("shoulder encoder runtime is not configured")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if state.simulation:
        state.set_error("shoulder encoder alignment requires connected hardware, not simulation")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if not serial_client.is_connected or not state.connected:
        state.set_error("connect the controller before shoulder encoder alignment")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if state.encoder_fault:
        state.set_error("clear the encoder fault and Set Pose before shoulder encoder alignment")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if automatic and not state.known_pose:
        state.set_error("automatic shoulder alignment requires a known planning pose")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if automatic and not state.hardware_armed:
        state.set_error("automatic shoulder alignment requires armed hardware")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    alignj_supported = bool(state.controller_capabilities.get("alignj"))
    if not alignj_supported:
        if not state.known_pose:
            state.set_error(
                "startup Align needs firmware with ALIGNJ support; flash/sync the controller, or Set Pose first "
                "to use the older armed correction path"
            )
            await broadcast_state()
            return {"ok": False, "error": state.last_error, "state": state.to_dict()}
        if not state.hardware_armed:
            state.set_error("arm hardware before shoulder encoder alignment with this firmware")
            await broadcast_state()
            return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if state.motion_state not in {MotionState.IDLE, MotionState.STOPPED}:
        state.set_error(
            f"robot must be idle or stopped before shoulder encoder alignment (current: {state.motion_state.value})"
        )
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if (
        state.live_motion_enabled
        or cartesian_jog_runtime.get("active")
        or (task_active() and not allow_active_execution)
    ):
        state.set_error("stop live motion, tasks, and Cartesian jog before shoulder encoder alignment")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    current_task = asyncio.current_task()
    active_tasks = [
        task
        for task in [path_task, live_task, task_task]
        if task is not None and not task.done() and task is not current_task
    ]
    if active_tasks:
        state.set_error("wait for active motion or task execution to finish")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    target = [float(value) for value in state.target_angles_deg]
    if len(target) < len(config.joints):
        target = [float(value) for value in state.reported_angles_deg]
    if target_shoulder_deg is not None and len(target) > 1:
        target[1] = float(target_shoulder_deg)
    state.last_command = "ALIGN_SHOULDER_TO_PLANNING"
    correction = settings.get("correction", {}) if isinstance(settings.get("correction"), dict) else {}
    if not bool(correction.get("enabled")):
        reason = "bounded shoulder correction is disabled"
        state.set_error(reason)
        await broadcast_state()
        return {"ok": False, "error": reason, "state": state.to_dict()}
    allowed_sources = {str(value) for value in correction.get("allowed_sources", [])}
    if motion_source not in allowed_sources:
        reason = f"{motion_source} is not allowed by the correction source policy"
        state.set_error(reason)
        await broadcast_state()
        return {"ok": False, "error": reason, "state": state.to_dict()}
    if len(state.hardware_axis_states) <= 1 or state.hardware_axis_states[1] != "hardware":
        reason = "bounded shoulder alignment requires the shoulder axis to be hardware-enabled"
        state.set_error(reason)
        await broadcast_state()
        return {"ok": False, "error": reason, "state": state.to_dict()}

    verification = settings.get("verification", {}) if isinstance(settings.get("verification"), dict) else {}
    required_samples = max(1, int(verification.get("required_stable_samples", 3)))
    correction_deadband = max(0.0, float(correction.get("deadband_deg", 0.75)))
    chunk_limit = max(0.001, float(correction.get("max_delta_deg", 8.0)))
    total_limit = max(chunk_limit, float(correction.get("align_max_delta_deg", 60.0)))
    correction_speed = float(correction.get("speed_deg_s", 2.0))
    correction_accel = float(correction.get("accel_deg_s2", 10.0))
    align_path_settings = request_settings(request.settings if request else None)
    align_speed_limits, align_accel_limits = _joint_limits_from_settings(align_path_settings)
    first_speed = float(align_speed_limits[1]) if len(align_speed_limits) > 1 else config.joints[1].max_speed_deg_s
    first_accel = float(align_accel_limits[1]) if len(align_accel_limits) > 1 else config.joints[1].max_accel_deg_s2
    if automatic or not alignj_supported:
        first_speed = min(first_speed, correction_speed)
        first_accel = min(first_accel, correction_accel)
    limit_margin = max(0.0, float(correction.get("joint_limit_margin_deg", 2.0)))
    target_shoulder = float(target[1])
    shoulder_joint = config.joints[1]
    measured, measurement_reason = await _stable_shoulder_measurement(required_samples)
    if measured is None:
        reason = f"shoulder encoder is not stable enough for alignment: {measurement_reason}"
        state.set_error(reason)
        await broadcast_state()
        return {"ok": False, "error": reason, "state": state.to_dict()}
    initial_error = float(measured) - target_shoulder
    if abs(initial_error) <= correction_deadband:
        state.encoder_mismatch = {
            **state.encoder_mismatch,
            "status": "aligned",
            "source": motion_source,
            "commanded_deg": target_shoulder,
            "measured_deg": measured,
            "error_deg": initial_error,
            "correction_status": "not_needed",
            "correction_skip_reason": (
                f"shoulder error {abs(initial_error):.2f} deg is within correction deadband "
                f"{correction_deadband:.2f} deg"
            ),
            "checked_at": time(),
        }
        state.clear_error()
        await broadcast_state()
        return {
            "ok": True,
            "verification": "already aligned",
            "mismatch": state.encoder_mismatch,
            "state": state.to_dict(),
        }
    if abs(initial_error) > total_limit:
        reason = (
            f"shoulder alignment error {abs(initial_error):.2f} deg exceeds Align Shoulder total cap "
            f"{total_limit:.2f} deg"
        )
        state.encoder_mismatch = {
            **state.encoder_mismatch,
            "status": "align_blocked",
            "source": motion_source,
            "commanded_deg": target_shoulder,
            "measured_deg": measured,
            "error_deg": initial_error,
            "correction_status": "skipped",
            "correction_skip_reason": reason,
            "correction_max_delta_deg": chunk_limit,
            "align_max_delta_deg": total_limit,
            "checked_at": time(),
        }
        state.set_error(reason)
        await broadcast_state()
        return {"ok": False, "error": reason, "mismatch": state.encoder_mismatch, "state": state.to_dict()}

    total_requested = 0.0
    chunk_records: list[dict[str, Any]] = []
    error = initial_error
    max_chunks = max(2, min(120, int(abs(initial_error) / chunk_limit) + 6))
    for chunk_index in range(1, max_chunks + 1):
        if abs(error) <= correction_deadband:
            break
        if chunk_index == 1 and not automatic:
            delta = max(-total_limit, min(total_limit, -error))
        else:
            delta = max(-chunk_limit, min(chunk_limit, -error))
        if abs(total_requested + delta) > total_limit + 1e-6:
            reason = "shoulder alignment would exceed the configured Align Shoulder total cap"
            state.encoder_mismatch = {
                **state.encoder_mismatch,
                "status": "align_blocked",
                "source": motion_source,
                "commanded_deg": target_shoulder,
                "measured_deg": measured,
                "error_deg": error,
                "correction_status": "skipped",
                "correction_skip_reason": reason,
                "correction_chunks": chunk_records,
                "align_total_requested_deg": total_requested,
                "align_max_delta_deg": total_limit,
            }
            state.set_error(reason)
            await broadcast_state()
            return {"ok": False, "error": reason, "mismatch": state.encoder_mismatch, "state": state.to_dict()}

        candidate_pulse_angle = float(measured) + delta
        if not (
            shoulder_joint.min_deg + limit_margin
            <= candidate_pulse_angle
            <= shoulder_joint.max_deg - limit_margin
        ):
            reason = "shoulder alignment would cross the configured joint-limit margin"
            state.encoder_mismatch = {
                **state.encoder_mismatch,
                "status": "align_blocked",
                "source": motion_source,
                "commanded_deg": target_shoulder,
                "measured_deg": measured,
                "error_deg": error,
                "correction_status": "skipped",
                "correction_skip_reason": reason,
                "correction_chunks": chunk_records,
                "align_total_requested_deg": total_requested,
            }
            state.set_error(reason)
            await broadcast_state()
            return {"ok": False, "error": reason, "mismatch": state.encoder_mismatch, "state": state.to_dict()}

        transaction_id = str(uuid4())
        command_speed = first_speed if chunk_index == 1 else correction_speed
        command_accel = first_accel if chunk_index == 1 else correction_accel
        command_kind = "ALIGNJ" if alignj_supported else "CORRECTJ"
        correction_timeout_s = correction_motion_timeout_s(delta, command_speed, command_accel)
        command = (
            format_alignj(2, delta, command_speed, command_accel, transaction_id)
            if alignj_supported
            else format_correctj(2, delta, command_speed, command_accel, transaction_id)
        )
        state.correction_state = {
            "state": "executing",
            "transaction_id": transaction_id,
            "attempt": chunk_index,
            "requested_delta_deg": delta,
            "speed_deg_s": command_speed,
            "accel_deg_s2": command_accel,
            "timeout_s": correction_timeout_s,
            "source": motion_source,
            "command": command_kind,
        }
        state.last_command = command
        state.encoder_mismatch = {
            **state.encoder_mismatch,
            "status": "aligning",
            "source": motion_source,
            "commanded_deg": target_shoulder,
            "measured_deg": measured,
            "error_deg": error,
            "correction_status": "executing",
            "correction_attempt": chunk_index,
            "correction_would_delta_deg": delta,
            "correction_max_delta_deg": chunk_limit,
            "align_max_delta_deg": total_limit,
            "align_total_requested_deg": total_requested + delta,
            "correction_chunks": chunk_records,
            "align_chunk_mode": "initial_full_error" if chunk_index == 1 else "residual_cleanup",
            "align_command": command_kind,
            "align_speed_deg_s": command_speed,
            "align_accel_deg_s2": command_accel,
        }
        await broadcast_state()
        try:
            if alignj_supported:
                send_alignj_and_read_response(command)
            else:
                send_correctj_and_read_response(command)
        except SerialClientError as exc:
            command_error = str(exc)
            if "delta_out_of_range" in command_error.lower() and abs(delta) > chunk_limit + 1e-6:
                fallback_reason = (
                    "firmware rejected the large Align delta; retrying this step with the "
                    f"{chunk_limit:.2f} deg automatic correction cap. Save/sync hardware so "
                    "future Align operations can use the larger Align cap."
                )
                delta = max(-chunk_limit, min(chunk_limit, delta))
                transaction_id = str(uuid4())
                correction_timeout_s = correction_motion_timeout_s(delta, command_speed, command_accel)
                command = (
                    format_alignj(2, delta, command_speed, command_accel, transaction_id)
                    if alignj_supported
                    else format_correctj(2, delta, command_speed, command_accel, transaction_id)
                )
                state.correction_state = {
                    "state": "executing",
                    "transaction_id": transaction_id,
                    "attempt": chunk_index,
                    "requested_delta_deg": delta,
                    "speed_deg_s": command_speed,
                    "accel_deg_s2": command_accel,
                    "timeout_s": correction_timeout_s,
                    "source": motion_source,
                    "command": command_kind,
                    "fallback_reason": fallback_reason,
                }
                state.last_command = command
                state.encoder_mismatch = {
                    **state.encoder_mismatch,
                    "correction_would_delta_deg": delta,
                    "correction_skip_reason": fallback_reason,
                    "align_chunk_mode": "firmware_delta_fallback",
                    "align_total_requested_deg": total_requested + delta,
                }
                await broadcast_state()
                try:
                    if alignj_supported:
                        send_alignj_and_read_response(command)
                    else:
                        send_correctj_and_read_response(command)
                except SerialClientError as fallback_exc:
                    message = f"shoulder alignment correction command failed after fallback: {fallback_exc}"
                    latch_encoder_mismatch_fault(message, error)
                    await broadcast_state()
                    return {"ok": False, "error": message, "mismatch": state.encoder_mismatch, "state": state.to_dict()}
            else:
                message = f"shoulder alignment correction command failed: {exc}"
                latch_encoder_mismatch_fault(message, error)
                await broadcast_state()
                return {"ok": False, "error": message, "mismatch": state.encoder_mismatch, "state": state.to_dict()}

        deadline = monotonic() + correction_timeout_s
        while monotonic() < deadline:
            await asyncio.sleep(0.08)
            try:
                refresh_serial_status()
            except SerialClientError as exc:
                message = f"shoulder alignment correction status failed: {exc}"
                latch_encoder_mismatch_fault(message, error)
                await broadcast_state()
                return {"ok": False, "error": message, "mismatch": state.encoder_mismatch, "state": state.to_dict()}
            if state.motion_state in {MotionState.IDLE, MotionState.STOPPED}:
                break
            if state.motion_state in {MotionState.FAULT, MotionState.ESTOP}:
                message = f"shoulder alignment interrupted: {state.motion_state.value}"
                latch_encoder_mismatch_fault(message, error)
                await broadcast_state()
                return {"ok": False, "error": message, "mismatch": state.encoder_mismatch, "state": state.to_dict()}
        else:
            message = (
                f"shoulder alignment correction timed out after {correction_timeout_s:.1f}s "
                f"for {delta:.2f} deg at {command_speed:.2f} deg/s"
            )
            latch_encoder_mismatch_fault(message, error)
            await broadcast_state()
            return {"ok": False, "error": message, "mismatch": state.encoder_mismatch, "state": state.to_dict()}

        total_requested += delta
        await asyncio.sleep(max(0.0, float(verification.get("settle_delay_ms", 300)) / 1000.0))
        measured, measurement_reason = await _stable_shoulder_measurement(required_samples)
        if measured is None:
            message = f"shoulder alignment lost encoder authority: {measurement_reason}"
            latch_encoder_mismatch_fault(message, None)
            await broadcast_state()
            return {"ok": False, "error": message, "mismatch": state.encoder_mismatch, "state": state.to_dict()}
        error = float(measured) - target_shoulder
        chunk_records.append(
            {
                "chunk": chunk_index,
                "requested_delta_deg": delta,
                "command": command_kind,
                "speed_deg_s": command_speed,
                "accel_deg_s2": command_accel,
                "measured_deg": measured,
                "error_deg": error,
            }
        )

    if abs(error) > correction_deadband:
        message = f"shoulder alignment did not converge; final error {error:.2f} deg"
        latch_encoder_mismatch_fault(message, error)
        await broadcast_state()
        return {"ok": False, "error": message, "mismatch": state.encoder_mismatch, "state": state.to_dict()}

    state.encoder_mismatch = {
        **state.encoder_mismatch,
        "status": "aligned",
        "source": motion_source,
        "commanded_deg": target_shoulder,
        "measured_deg": measured,
        "error_deg": error,
        "correction_status": "completed",
        "correction_skip_reason": "",
        "correction_chunks": chunk_records,
        "align_total_requested_deg": total_requested,
        "align_max_delta_deg": total_limit,
        "corrected_at": time(),
    }
    state.clear_error()
    log_event(
        "encoder",
        "shoulder alignment requested",
        motion_source=motion_source,
        automatic=automatic,
        target_shoulder_deg=target[1] if len(target) > 1 else None,
        final_error_deg=error,
        chunks=len(chunk_records),
        total_requested_deg=total_requested,
    )
    await broadcast_state()
    return {
        "ok": True,
        "verification": "aligned",
        "mismatch": state.encoder_mismatch,
        "state": state.to_dict(),
    }


async def ensure_shoulder_alignment_before_motion(
    motion_source: str,
    settings_payload: PathSettingsRequest | dict[str, Any] | None = None,
    *,
    target_shoulder_deg: float | None = None,
) -> tuple[bool, str]:
    reason = shoulder_alignment_motion_blocking_reason(target_shoulder_deg)
    if not reason:
        return True, ""

    settings = encoder_settings(config)
    correction = settings.get("correction", {}) if isinstance(settings.get("correction"), dict) else {}
    allowed_sources = {str(value) for value in correction.get("allowed_sources", [])}
    if not bool(correction.get("enabled")) or motion_source not in allowed_sources:
        return False, reason

    execution = dict(state.task_execution or {})
    run_id = str(execution.get("run_id") or "")
    previous_status = execution.get("status")
    previous_phase = execution.get("phase")
    previous_step = deepcopy(execution.get("current_step"))
    if run_id and task_active():
        update_task_execution(
            status="executing",
            phase="automatic_encoder_alignment",
            current_step={
                "label": "automatic shoulder alignment",
                "kind": "encoder_alignment",
                "motion_source": motion_source,
            },
        )
        await broadcast_state()

    request = EncoderShoulderAlignRequest(settings=settings_payload)
    result = await align_shoulder_to_planning_internal(
        request,
        allow_active_execution=True,
        motion_source=motion_source,
        automatic=True,
        target_shoulder_deg=target_shoulder_deg,
    )

    if (
        run_id
        and (state.task_execution or {}).get("run_id") == run_id
        and (state.task_execution or {}).get("status") in ACTIVE_TASK_STATUSES
    ):
        update_task_execution(
            status=previous_status,
            phase=previous_phase,
            current_step=previous_step,
        )
        await broadcast_state()

    if not result.get("ok"):
        message = str(result.get("error") or reason)
        return False, f"automatic shoulder alignment failed: {message}"

    remaining_reason = shoulder_alignment_motion_blocking_reason(target_shoulder_deg)
    if remaining_reason:
        return False, f"automatic shoulder alignment did not clear the motion gate: {remaining_reason}"
    log_event(
        "encoder",
        "automatic shoulder alignment completed",
        motion_source=motion_source,
        mismatch=deepcopy(state.encoder_mismatch),
    )
    return True, "automatic shoulder alignment completed"


@app.post("/api/encoder/shoulder/align")
async def align_shoulder_to_planning(request: EncoderShoulderAlignRequest | None = None) -> dict[str, Any]:
    """Explicit operator-requested shoulder alignment."""
    return await align_shoulder_to_planning_internal(request)


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


def _persist_calibration_updates(updates: dict[str, Any]) -> None:
    config_path = ensure_local_config()
    with tempfile.TemporaryDirectory() as tmp_dir:
        draft_path = Path(tmp_dir) / "robot.local.yaml"
        shutil.copyfile(config_path, draft_path)
        save_calibration_updates(draft_path, updates)
        load_config(draft_path)
    save_calibration_updates(config_path, updates)
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


def _joint_samples(
    minimum: float,
    maximum: float,
    count: int,
    *,
    lower_fraction: float,
    upper_fraction: float,
) -> list[float]:
    if count <= 1:
        return [(minimum + maximum) * 0.5]
    span = maximum - minimum
    return [
        minimum + span * (lower_fraction + (upper_fraction - lower_fraction) * index / (count - 1))
        for index in range(count)
    ]


def _automatic_calibration_targets(
    robot_config: RobotConfig,
    polygon: list[list[float]],
    *,
    count: int,
    validation_stride: int,
) -> dict[str, Any]:
    if not 6 <= count <= 24:
        raise ValueError("automatic calibration target count must be between 6 and 24")
    rows = robot_config.kinematics.dh_rows
    if len(rows) != 4:
        raise ValueError("automatic calibration target generation requires four DH rows")
    reference = kinematics_calibration_context(robot_config).get("measurement_reference", {})
    plane_z = float(reference.get("workspace_plane_z_mm", 0.0))
    modeled_reach = sum(abs(float(row.a_mm)) for row in rows) + abs(
        float(robot_config.links.tool_tcp_offset_mm.get("z", 0.0))
    )
    minimum_z = plane_z + 35.0
    maximum_z = plane_z + min(240.0, max(120.0, modeled_reach * 0.55))
    joints = robot_config.joints
    base_values = _joint_samples(
        joints[0].min_deg,
        joints[0].max_deg,
        9,
        lower_fraction=0.02,
        upper_fraction=0.98,
    )
    shoulder_values = _joint_samples(
        joints[1].min_deg,
        joints[1].max_deg,
        6,
        lower_fraction=0.03,
        upper_fraction=0.85,
    )
    elbow_values = _joint_samples(
        joints[2].min_deg,
        joints[2].max_deg,
        7,
        lower_fraction=0.05,
        upper_fraction=0.95,
    )
    desired_phi_values = [-90.0, -45.0, 0.0, 45.0, 90.0]
    candidates: list[dict[str, Any]] = []
    for base, shoulder, elbow, desired_phi in product(
        base_values,
        shoulder_values,
        elbow_values,
        desired_phi_values,
    ):
        pitch_without_wrist = sum(
            angle * row.direction_sign + row.zero_offset_deg
            for angle, row in [
                (shoulder, rows[1]),
                (elbow, rows[2]),
            ]
        )
        wrist = (
            desired_phi - pitch_without_wrist - rows[3].zero_offset_deg
        ) / rows[3].direction_sign
        if not joints[3].min_deg <= wrist <= joints[3].max_deg:
            continue
        angles = [base, shoulder, elbow, wrist]
        fk = forward_kinematics(angles, robot_config.links)
        if not _point_inside_polygon(float(fk["x_mm"]), float(fk["y_mm"]), polygon):
            continue
        if not minimum_z <= float(fk["z_mm"]) <= maximum_z:
            continue
        joint_clearance = min(
            min(
                (angle - joint.min_deg) / (joint.max_deg - joint.min_deg),
                (joint.max_deg - angle) / (joint.max_deg - joint.min_deg),
            )
            for angle, joint in zip(angles, joints, strict=True)
        )
        candidates.append(
            {
                "seed_angles_deg": [float(value) for value in angles],
                "intended_target": {
                    "x_mm": float(fk["x_mm"]),
                    "y_mm": float(fk["y_mm"]),
                    "z_mm": float(fk["z_mm"]),
                    "phi_deg": float(fk["tool_phi_deg"]),
                },
                "joint_clearance_fraction": float(joint_clearance),
            }
        )
    if len(candidates) < count:
        raise ValueError(
            f"only {len(candidates)} automatic calibration poses fit the active workspace/model; "
            "verify workspace bounds, tool TCP, and joint limits"
        )

    features = [
        [
            candidate["intended_target"]["x_mm"],
            candidate["intended_target"]["y_mm"],
            candidate["intended_target"]["z_mm"],
            candidate["intended_target"]["phi_deg"],
        ]
        for candidate in candidates
    ]
    minimums = [min(values) for values in zip(*features, strict=True)]
    maximums = [max(values) for values in zip(*features, strict=True)]
    spans = [max(1e-9, high - low) for low, high in zip(minimums, maximums, strict=True)]
    normalized = [
        [(value - low) / span for value, low, span in zip(values, minimums, spans, strict=True)]
        for values in features
    ]
    selected = [
        max(
            range(len(candidates)),
            key=lambda index: candidates[index]["joint_clearance_fraction"],
        )
    ]
    while len(selected) < count:
        next_index = max(
            (index for index in range(len(candidates)) if index not in selected),
            key=lambda index: min(
                dist(normalized[index], normalized[chosen])
                for chosen in selected
            )
            + 0.2 * candidates[index]["joint_clearance_fraction"],
        )
        selected.append(next_index)

    points: list[dict[str, Any]] = []
    for index, candidate_index in enumerate(selected, start=1):
        candidate = deepcopy(candidates[candidate_index])
        role = "validation" if index % validation_stride == 0 else "fit"
        candidate.update(
            {
                "index": index,
                "recommended_role": role,
                "command_target": deepcopy(candidate["intended_target"]),
                "calibration": {
                    "applied": False,
                    "reason": "automatic_fit_targets_are_uncorrected",
                },
                "reachable": True,
                "diagnostic_category": "reachable_from_generated_joint_pose",
                "ik_notes": ["target was generated by FK from a valid in-limit joint pose"],
            }
        )
        points.append(candidate)
    selected_features = [
        [
            point["intended_target"]["x_mm"],
            point["intended_target"]["y_mm"],
            point["intended_target"]["z_mm"],
            point["intended_target"]["phi_deg"],
        ]
        for point in points
    ]
    selected_ranges = [
        max(values) - min(values)
        for values in zip(*selected_features, strict=True)
    ]
    return {
        "points": points,
        "strategy": {
            "id": "model_aware_joint_pose_sampling",
            "message": (
                "Z and tool pitch were selected automatically from valid joint poses "
                "to provide spatial and orientation coverage."
            ),
            "candidate_count": len(candidates),
            "selected_count": len(points),
            "workspace_plane_z_mm": plane_z,
            "allowed_z_range_mm": [minimum_z, maximum_z],
            "coverage": {
                "x_span_mm": selected_ranges[0],
                "y_span_mm": selected_ranges[1],
                "z_span_mm": selected_ranges[2],
                "phi_span_deg": selected_ranges[3],
            },
        },
    }


def _calibration_capture_contract(
    request: KinematicsCalibrationSampleRequest,
) -> dict[str, Any]:
    preview_id = str(request.preview_id or "")
    if not preview_id:
        raise ValueError("save samples only after executing a calibration preview")
    preview = path_previews.get(preview_id)
    if not isinstance(preview, dict):
        raise ValueError("calibration preview expired; preview and execute the measurement move again")
    if preview.get("source") not in {
        "kinematics_calibration_fit",
        "kinematics_calibration_validation",
    }:
        raise ValueError("sample preview was not created by the calibration workflow")
    if not preview.get("execution_started_at"):
        raise ValueError("execute the calibration preview before saving its sample")
    if int(preview.get("execution_count", 0)) != 1:
        raise ValueError("calibration previews may be executed only once; create a fresh preview")
    expected_role = "validation" if preview.get("source") == "kinematics_calibration_validation" else "fit"
    if str(request.role).lower() != expected_role:
        raise ValueError(f"this preview is bound to a {expected_role} sample")
    trajectory = preview.get("trajectory") if isinstance(preview.get("trajectory"), dict) else {}
    waypoints = trajectory.get("waypoints") if isinstance(trajectory.get("waypoints"), list) else []
    if not waypoints:
        raise ValueError("calibration preview has no planned endpoint")
    final_angles = [float(value) for value in waypoints[-1]]
    deltas = [
        abs(float(actual) - expected)
        for actual, expected in zip(state.reported_angles_deg, final_angles, strict=True)
    ]
    max_delta = max(deltas, default=0.0)
    if max_delta > 0.5:
        raise ValueError(
            f"reported pose is {max_delta:.2f} deg from the executed calibration endpoint; "
            "wait for settling or preview the sample again"
        )
    for key, requested in [
        ("intended_target", request.intended_target),
        ("command_target", request.command_target or request.intended_target),
    ]:
        expected = preview.get("target" if key == "intended_target" else "command_target")
        if not isinstance(expected, dict):
            raise ValueError(f"calibration preview is missing {key}")
        if any(
            abs(float(requested[axis]) - float(expected[axis])) > 0.05
            for axis in ["x_mm", "y_mm", "z_mm"]
        ):
            raise ValueError(f"{key} changed after preview; preview and execute the measurement move again")
    return {
        "preview_id": preview_id,
        "preview_source": preview.get("source"),
        "preview_created_at": preview.get("created_at"),
        "execution_started_at": preview.get("execution_started_at"),
        "captured_pose_revision": int(state.pose_revision),
        "captured_pose_source": state.pose_source,
        "joint_authority": "simulation" if state.simulation else "estimated_open_loop",
        "per_joint_authority": list(state.joint_authority),
        "measurement_valid_mask": state.measurement_valid_mask,
        "measured_angles_deg": list(state.measured_angles_deg),
        "encoder_evidence": deepcopy(state.encoder_evidence),
        "reported_pose_is_estimated": True,
        "model_fingerprint": preview.get("model_fingerprint"),
        "config_id": preview.get("config_id"),
        "context": kinematics_calibration_context(config),
    }


@app.get("/api/kinematics-calibration")
async def get_kinematics_calibration() -> dict[str, Any]:
    return {
        "ok": True,
        "physical_model_parameter_groups": deepcopy(PHYSICAL_MODEL_PARAMETER_GROUPS),
        **kinematics_calibration_summary(config),
    }


@app.post("/api/kinematics-calibration/targets")
async def generate_kinematics_calibration_targets(
    request: KinematicsCalibrationTargetsRequest,
) -> dict[str, Any]:
    if not 2 <= int(request.validation_stride) <= 20:
        return {"ok": False, "error": "validation_stride must be between 2 and 20"}
    try:
        polygon = _workspace_polygon()
        generated = await asyncio.to_thread(
            _automatic_calibration_targets,
            config,
            polygon,
            count=int(request.count),
            validation_stride=int(request.validation_stride),
        )
        points = generated["points"]
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    summary = kinematics_calibration_summary(config)
    return {
        "ok": True,
        "points": points,
        "strategy": generated["strategy"],
        "workspace": kinematics_workspace_context(config),
        "fit_quality": summary.get("fit_quality"),
        "validation_quality": summary.get("validation_quality"),
        "reachability": {
            "reachable_count": len(points),
            "unreachable_count": 0,
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
        capture = _calibration_capture_contract(request)
        existing_settings = kinematics_calibration_settings(config)
        for existing_profile in existing_settings.get("profiles", {}).values():
            if not isinstance(existing_profile, dict):
                continue
            if any(
                str(sample.get("capture", {}).get("preview_id") or "") == str(request.preview_id or "")
                for sample in existing_profile.get("samples", [])
                if isinstance(sample, dict)
            ):
                raise ValueError("this calibration preview already has a saved sample")
        current_fk = forward_kinematics(state.reported_angles_deg, config.links)
        payload = dict(request.__dict__)
        payload["capture"] = capture
        payload["joint_source"] = capture["joint_authority"]
        sample = create_kinematics_calibration_sample(
            payload,
            config,
            state.reported_angles_deg,
            current_fk,
        )
        settings = existing_settings
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
                "model_type": str(profile.get("model_type") or settings.get("default_model") or DEFAULT_CALIBRATION_MODEL),
                "workspace": kinematics_workspace_context(config),
                "context": kinematics_calibration_context(config, profile_key),
                "samples": samples,
            }
        )
        profiles[profile_key] = profile
        settings["active_profile"] = profile_key
        if sample["role"] == "fit":
            profile.pop("result", None)
            profile.pop("activation", None)
            profile.pop("physical_model_result", None)
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
    profile.pop("activation", None)
    profile.pop("physical_model_result", None)
    profile["enabled"] = False
    settings["enabled"] = False
    _persist_kinematics_calibration(settings)
    log_event("calibration", "TCP sample deleted", sample_id=sample_id)
    return {"ok": True, "config": public_config(), **kinematics_calibration_summary(config)}


@app.post("/api/kinematics-calibration/manual-offsets")
async def save_kinematics_calibration_manual_offsets(
    request: KinematicsCalibrationManualOffsetsRequest,
) -> dict[str, Any]:
    if state.motion_state == MotionState.MOVING:
        return {"ok": False, "error": "stop motion before saving manual calibration offsets"}
    try:
        settings, result = save_manual_radial_offsets(
            kinematics_calibration_settings(config),
            config,
            profile_key=request.profile_key,
            reach_offset_mm=request.reach_offset_mm,
            z_offset_mm=request.z_offset_mm,
            enabled=request.enabled,
        )
        _persist_kinematics_calibration(settings)
    except Exception as exc:
        return {"ok": False, "error": str(exc), "diagnostic_category": "manual_offsets"}
    log_event(
        "calibration",
        "manual reach/Z calibration offsets saved",
        reach_offset_mm=request.reach_offset_mm,
        z_offset_mm=request.z_offset_mm,
        enabled=request.enabled,
    )
    return {
        "ok": True,
        "result": result,
        "config": public_config(),
        **kinematics_calibration_summary(config),
    }


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
        activation = result.get("activation") if isinstance(result.get("activation"), dict) else {}
        enable = bool(request.enable_after_fit and activation.get("eligible"))
        profile["enabled"] = enable
        settings["enabled"] = enable
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
    summary = kinematics_calibration_summary(config)
    if request.enabled and not summary.get("freshness", {}).get("fresh"):
        return {
            "ok": False,
            "error": "calibration profile is stale: " + "; ".join(summary.get("freshness", {}).get("messages", [])),
        }
    if request.enabled and not summary.get("activation", {}).get("eligible"):
        return {
            "ok": False,
            "error": "calibration correction is not eligible: "
            + "; ".join(summary.get("activation", {}).get("reasons", [])),
        }
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


@app.post("/api/kinematics-calibration/physical-model/fit")
async def fit_kinematics_physical_model(
    request: PhysicalModelFitRequest,
) -> dict[str, Any]:
    if state.motion_state == MotionState.MOVING:
        return {"ok": False, "error": "stop motion before fitting the physical model"}
    settings = kinematics_calibration_settings(config)
    profile_key = str(
        request.profile_key
        or kinematics_calibration_summary(config).get("active_profile_key")
        or ""
    )
    profiles = settings.get("profiles")
    profile = profiles.get(profile_key) if isinstance(profiles, dict) else None
    if not isinstance(profile, dict):
        return {"ok": False, "error": "active calibration profile not found"}
    try:
        result = fit_physical_model(
            profile,
            config,
            parameter_group=request.parameter_group,
            thresholds=settings.get("thresholds"),
        )
        profile["physical_model_result"] = result
        profile["enabled"] = False
        settings["enabled"] = False
        _persist_kinematics_calibration(settings)
    except Exception as exc:
        return {"ok": False, "error": str(exc), "diagnostic_category": "physical_model_fit"}
    log_event(
        "calibration",
        "physical model candidate fitted",
        result_id=result["id"],
        parameter_group=request.parameter_group,
        safe_to_apply=result.get("safe_to_apply"),
    )
    return {
        "ok": True,
        "result": result,
        "config": public_config(),
        **kinematics_calibration_summary(config),
    }


@app.post("/api/kinematics-calibration/physical-model/apply")
async def apply_kinematics_physical_model(
    request: PhysicalModelApplyRequest,
) -> dict[str, Any]:
    if not request.confirm:
        return {"ok": False, "error": "explicit confirmation is required before applying a physical-model update"}
    if state.motion_state == MotionState.MOVING or any(
        task is not None and not task.done() for task in [path_task, live_task, task_task]
    ):
        return {"ok": False, "error": "stop all motion and tasks before applying a physical-model update"}
    if state.hardware_armed:
        return {"ok": False, "error": "disarm hardware before applying a physical-model update"}
    settings = kinematics_calibration_settings(config)
    profile_key = str(
        request.profile_key
        or kinematics_calibration_summary(config).get("active_profile_key")
        or ""
    )
    profiles = settings.get("profiles")
    profile = profiles.get(profile_key) if isinstance(profiles, dict) else None
    result = profile.get("physical_model_result") if isinstance(profile, dict) else None
    if not isinstance(result, dict) or str(result.get("id")) != request.result_id:
        return {"ok": False, "error": "physical-model result not found or no longer active"}
    if result.get("context", {}).get("model", {}).get("signature") != kinematics_calibration_context(config).get("model", {}).get("signature"):
        return {"ok": False, "error": "robot model changed after fitting; refit before applying"}
    try:
        updates = physical_model_updates(config, result)
        applied_result = deepcopy(result)
        applied_result["applied_at"] = datetime.now(timezone.utc).isoformat()
        profile["last_applied_physical_model_result"] = applied_result
        existing_samples = deepcopy(profile.get("samples") or [])
        archives = profile.get("sample_archives")
        archives = list(archives) if isinstance(archives, list) else []
        if existing_samples:
            archives.append(
                {
                    "archived_at": applied_result["applied_at"],
                    "reason": "physical model changed; samples no longer match the active model",
                    "samples": existing_samples,
                }
            )
        profile["sample_archives"] = archives[-3:]
        profile["samples"] = []
        profile.pop("context", None)
        profile.pop("physical_model_result", None)
        profile.pop("result", None)
        profile.pop("activation", None)
        profile["enabled"] = False
        settings["enabled"] = False
        updates["kinematics_calibration"] = settings
        _persist_calibration_updates(updates)
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    log_event(
        "calibration",
        "physical model update applied",
        result_id=request.result_id,
        parameter_group=result.get("parameter_group"),
    )
    return {
        "ok": True,
        "message": "physical model updated; correction disabled and all previews invalidated",
        "config": public_config(),
        "state": state.to_dict(),
    }


@app.post("/api/tools/validation")
async def set_active_tool_dimensions_validation(
    request: ToolDimensionsValidationRequest,
) -> dict[str, Any]:
    tools = tools_settings(config)
    active = str(tools.get("active", "gripper"))
    presets = tools.get("presets") if isinstance(tools.get("presets"), dict) else {}
    preset = presets.get(active) if isinstance(presets.get(active), dict) else None
    if preset is None:
        state.set_error(f"active tool {active} is missing from presets")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    preset["dimensions_validated"] = bool(request.validated)
    if request.validated:
        preset["dimensions_validated_at"] = datetime.now(timezone.utc).isoformat()
    else:
        preset.pop("dimensions_validated_at", None)
    calibration = calibration_settings(config)
    calibration["tool_dimensions_validated"] = bool(request.validated)
    calibration["last_validation"] = preset.get("dimensions_validated_at", "")
    try:
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
    except Exception as exc:
        state.set_error(f"could not update tool-dimension validation: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}

    state.last_command = "VALIDATE_TOOL_DIMENSIONS" if request.validated else "INVALIDATE_TOOL_DIMENSIONS"
    state.clear_error()
    log_event(
        "calibration",
        "active tool dimensions validation changed",
        tool=active,
        validated=bool(request.validated),
    )
    await broadcast_state()
    return {
        "ok": True,
        "validated": active_tool_dimensions_validated(config),
        "config": public_config(),
        "config_change": state.config_change,
        "state": state.to_dict(),
    }


@app.post("/api/ik/solve")
async def solve_ik(request: IkSolveRequest) -> dict[str, Any]:
    links = links_from_override(request.links_mm)
    requested_target = request.target.__dict__
    calibration_compatible = links == config.links
    command_target, correction = correct_cartesian_target(
        requested_target,
        config,
        apply_enabled=bool(request.apply_calibration and calibration_compatible),
    )
    if request.apply_calibration and not calibration_compatible:
        correction["reason"] = "kinematics_override"
        correction["warnings"] = [
            *correction.get("warnings", []),
            "Cartesian calibration is not applied while solving with overridden link geometry",
        ]
    result = inverse_kinematics(
        command_target,
        links,
        config.joints,
        state.reported_angles_deg,
        request.branch,
        selection_policy=ik_selection_policy({}),
    )
    return {
        "ok": result["ok"],
        "ik": result,
        "requested_target": requested_target,
        "command_target": command_target,
        "calibration": correction,
    }


@app.post("/api/path/preview")
async def preview_path(request: PathPreviewRequest) -> dict[str, Any]:
    refresh_error = refresh_idle_planning_pose_from_hardware()
    if refresh_error:
        state.set_error(refresh_error)
        await broadcast_state()
        return {"ok": False, "error": refresh_error, "state": state.to_dict()}
    links = links_from_override(request.links_mm)
    purpose = str(request.purpose or "path")
    calibration_move = purpose in {
        "kinematics_calibration_fit",
        "kinematics_calibration_validation",
    }
    result = build_preview(
        mode=request.mode,
        target=request.target.__dict__ if request.target else None,
        waypoint_program=request.waypoints,
        links=links,
        settings=calibration_path_settings(request.settings) if calibration_move else request_settings(request.settings),
        branch=request.branch,
        source=purpose if calibration_move else "path",
        apply_calibration=request.apply_calibration,
        calibration_trial=purpose == "kinematics_calibration_validation",
        program_revision=request.program_revision,
    )
    if (
        result.get("ok")
        and request.mode.lower() == "program"
        and request.program_id
        and request.waypoints
    ):
        try:
            result["plan_cache"] = {
                "saved": True,
                **cache_program_preview(
                    request.program_id,
                    request.waypoints,
                    result["preview"],
                ),
            }
            result["preview"]["program_id"] = request.program_id
        except ProgramLibraryError as exc:
            result["plan_cache"] = {"saved": False, "error": str(exc)}
    return result


@app.post("/api/programs/preview-step")
async def preview_program_step(request: ProgramStepPreviewRequest) -> dict[str, Any]:
    refresh_error = refresh_idle_planning_pose_from_hardware()
    if refresh_error:
        state.set_error(refresh_error)
        await broadcast_state()
        return {"ok": False, "error": refresh_error, "state": state.to_dict()}
    if request.step_index < 0 or request.step_index >= len(request.waypoints):
        return {"ok": False, "error": "program step index is outside the sequence"}
    selected = request.waypoints[request.step_index]
    if selected.get("enabled", True) is False:
        return {"ok": False, "error": "disabled program steps cannot be previewed"}
    result = build_preview(
        mode="program",
        target=None,
        waypoint_program=request.waypoints[: request.step_index + 1],
        links=config.links,
        settings=request_settings(request.settings),
        branch=request.branch,
        source="program_step_preview",
        program_revision=request.program_revision,
    )
    result["step_index"] = request.step_index
    return result


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
        aligned, reason = await ensure_shoulder_alignment_before_motion(
            "program" if preview.get("mode") == "program" else str(preview.get("source", "path")),
            preview.get("settings", {}),
        )
        if not aligned:
            state.set_error(reason)
            await broadcast_state()
            return {"ok": False, "error": reason, "state": state.to_dict()}
        reason = hardware_trajectory_start_blocking_reason()
        if reason:
            state.set_error(reason)
            await broadcast_state()
            return {"ok": False, "error": reason, "state": state.to_dict()}
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
    rebase_preview_start_to_current_if_encoder_tracked(preview)
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
    if state.motion_state == MotionState.STOPPED:
        state.motion_state = MotionState.IDLE
    state.last_command = f"PATH_EXECUTE {request.preview_id}"
    preview["execution_started_at"] = time()
    preview["execution_start_pose_revision"] = int(state.pose_revision)
    preview["execution_count"] = int(preview.get("execution_count", 0)) + 1
    path_task_source = "path_execute"
    trajectory = preview.get("trajectory", {})
    trajectory_mode = str(trajectory.get("mode", preview.get("mode", ""))).lower()
    waypoints = trajectory.get("waypoints") or []
    final_target = (
        [float(value) for value in waypoints[-1]]
        if waypoints
        else state.reported_angles_deg.copy()
    )
    execution_steps = list(trajectory.get("execution_steps") or [])
    start_motion_diagnostics(
        source="program" if preview.get("mode") == "program" else str(preview.get("source", "path")),
        mode=trajectory_mode or str(preview.get("mode", "path")),
        target_deg=final_target,
        expected_duration_s=float(trajectory.get("duration_s", 0.0)),
        waypoint_count=int(trajectory.get("waypoint_count", len(waypoints)) or 0),
        step_total=len(execution_steps),
    )
    if preview.get("mode") == "program" and trajectory.get("execution_steps"):
        path_task_source = "program_execute"
        path_task = asyncio.create_task(execute_program_sequence(preview))
    elif trajectory_mode == "joint":
        path_task = asyncio.create_task(execute_joint_endpoint_move(preview))
    else:
        path_task = asyncio.create_task(execute_waypoint_path(preview))
    await broadcast_state()
    return {"ok": True, "state": state.to_dict()}


@app.post("/api/path/go")
async def go_path(request: PathGoRequest) -> dict[str, Any]:
    if not request.waypoints:
        state.set_error("Go To requires at least one waypoint")
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    preview_result = build_preview(
        mode="program",
        target=None,
        waypoint_program=request.waypoints,
        links=config.links,
        settings=request_settings(request.settings),
        branch=request.branch,
        source="position_library_go",
    )
    if not preview_result.get("ok"):
        return preview_result
    execution = await execute_path(PathExecuteRequest(preview_id=preview_result["preview_id"]))
    execution["preview"] = preview_result["preview"]
    return execution


@app.post("/api/task/preview")
async def preview_task(request: TaskPreviewRequest) -> dict[str, Any]:
    detection_snapshot_id: str | None = None
    detection_captured_at: float | None = None
    detection_fingerprint: str | None = None
    detection_task_fingerprint: str | None = None
    detection_snapshot_server_bound = False
    task_settings_raw = request.task_settings or {}
    runtime_config = config
    profiles: dict[str, dict[str, Any]] = {}
    profile_overrides = None
    if isinstance(task_settings_raw, dict):
        profile_overrides = (
            task_settings_raw.get("color_profile_overrides")
            or task_settings_raw.get("draft_color_profiles")
            or task_settings_raw.get("color_profiles")
        )
    try:
        runtime_config = task_runtime_config(config, task_settings_raw)
        profiles = deepcopy(color_profiles(runtime_config))
        if isinstance(profile_overrides, dict):
            for name, profile in profile_overrides.items():
                normalized_name = str(name).strip().lower()
                if normalized_name and isinstance(profile, dict):
                    merged = deepcopy(profiles.get(normalized_name, {}))
                    merged.update(deepcopy(profile))
                    profiles[normalized_name] = merged
        path_settings = validated_task_path_settings(request.settings)
        if request.task in {"sorting", "color_sorting"}:
            detections = request.detections or ([request.detection] if request.detection else [])
            if not detections:
                raise TaskSettingsError("refresh detections before previewing a color-sorting task")
            detection_fingerprint = stable_payload_fingerprint(detections)
            detection_task_fingerprint = task_detection_fingerprint(detections)
            if request.detection_snapshot_id:
                detection_snapshot_id = str(request.detection_snapshot_id)
                if latest_vision_snapshot:
                    if detection_snapshot_id != str(latest_vision_snapshot.get("id") or ""):
                        raise TaskSettingsError(
                            "detection snapshot is stale; refresh detections before previewing the task"
                        )
                    latest_task_fingerprint = latest_vision_snapshot.get("task_fingerprint")
                    if latest_task_fingerprint:
                        contents_match = detection_task_fingerprint == latest_task_fingerprint
                    else:
                        contents_match = detection_fingerprint == latest_vision_snapshot.get("fingerprint")
                    if not contents_match:
                        raise TaskSettingsError(
                            "detection snapshot contents do not match the latest capture"
                        )
                    detection_snapshot_server_bound = True
                    detection_captured_at = float(latest_vision_snapshot.get("captured_at") or time())
                else:
                    detection_captured_at = float(request.detection_captured_at or time())
            else:
                detection_snapshot_id = f"client-{detection_fingerprint}"
                detection_captured_at = float(request.detection_captured_at or time())
            sequence = build_color_sorting_plan(
                runtime_config,
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
                runtime_config,
                {"execution_strategy": "batch_once", **(request.task_settings or {})},
            )
            sequence = build_pick_and_place_sequence(runtime_config, target, request.drop_zone, task_settings=task_settings)
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
    except (TaskSettingsError, TaskDestinationError, ValueError) as exc:
        state.set_error(str(exc))
        await broadcast_state()
        return {
            "ok": False,
            "error": str(exc),
            "sequence": {"ok": False, "steps": [], "waypoints": []},
            "task_preview": {"warnings": [], "ignored_detections": [], "selected_objects": []},
            "state": state.to_dict(),
        }
    task_preview = dict(sequence.get("task_preview", {}))
    normalized_task_settings = task_preview.get("normalized_settings") or request.task_settings or {}
    if (
        request.task in {"sorting", "color_sorting"}
        and normalized_task_settings.get("execution_strategy") == "closed_loop"
    ):
        camera_clear_position = str(
            normalized_task_settings.get("camera_clear_position")
            or normalized_task_settings.get("safe_position")
            or "home"
        )
        camera_clear_check = check_task_named_position_motion(
            camera_clear_position,
            path_settings,
            request.branch,
            "camera clear",
        )
        task_preview["camera_clear_position"] = camera_clear_position
        task_preview["camera_clear_check"] = camera_clear_check
        sequence["task_preview"] = task_preview
        if not camera_clear_check.get("ok"):
            reason = str(camera_clear_check.get("error") or "camera clear move failed")
            if not reason.lower().startswith("camera clear"):
                reason = f"camera clear is not available: {reason}"
            state.set_error(reason)
            await broadcast_state()
            return {
                "ok": False,
                "error": reason,
                "sequence": sequence,
                "task_preview": task_preview,
                "state": state.to_dict(),
            }
    settings_revision = task_contract_fingerprint(
        task_settings=normalized_task_settings,
        path_settings=path_settings,
        branch=request.branch,
        selected_detection_ids=request.selected_detection_ids,
    )
    destination_revision = task_mapping_fingerprint(normalized_task_settings)
    bindings = {
        "detection_snapshot_id": detection_snapshot_id,
        "detection_captured_at": detection_captured_at,
        "detection_fingerprint": detection_fingerprint,
        "detection_task_fingerprint": detection_task_fingerprint,
        "pose_revision": int(state.pose_revision),
        "config_id": RUNNING_CONFIG_ID,
        "model_fingerprint": robot_model_fingerprint(),
        "task_settings_revision": settings_revision,
        "destination_revision": destination_revision,
    }
    task_preview["bindings"] = bindings
    task_preview["active_tool"] = str(tools_settings(config).get("active", ""))
    task_preview["tool_type"] = str(tool_settings(config).get("type", ""))
    sequence["task_preview"] = task_preview

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

    skipped_preview_records: list[dict[str, Any]] = []
    skipped_preview_ids: set[str] = set()
    while True:
        preview_result, sequence, skipped_objects = build_task_motion_preview_skipping_failed_objects(
            sequence,
            links=runtime_config.links,
            settings=path_settings,
            branch=request.branch,
        )
        for skipped in skipped_objects:
            skipped_preview_records.append(skipped)
            if skipped.get("detection_id") is not None:
                skipped_preview_ids.add(str(skipped["detection_id"]))
        if preview_result.get("ok"):
            break
        if (
            request.task in {"sorting", "color_sorting"}
            and normalized_task_settings.get("execution_strategy") == "closed_loop"
            and not request.selected_detection_ids
            and str(preview_result.get("error") or "").startswith(
                "no task objects have a safe continuous IK path"
            )
        ):
            metadata = sequence.get("task_preview", {})
            candidate_ids = [
                str(item.get("detection_id"))
                for item in metadata.get("candidate_objects", [])
                if isinstance(item, dict)
                and item.get("detection_id") is not None
                and str(item.get("detection_id")) not in skipped_preview_ids
            ]
            if candidate_ids:
                sequence = build_color_sorting_plan(
                    runtime_config,
                    detections,
                    profiles,
                    task_settings=request.task_settings,
                    selected_detection_ids=[candidate_ids[0]],
                )
                continue
        break
    task_preview = dict(sequence.get("task_preview", task_preview))
    if skipped_preview_records:
        warnings = list(task_preview.get("warnings") or [])
        ignored = list(task_preview.get("ignored_detections") or [])
        existing_ignored = {str(item.get("detection_id")) for item in ignored if isinstance(item, dict)}
        for skipped in skipped_preview_records:
            detection_id = str(skipped.get("detection_id") or "")
            reason = str(skipped.get("reason") or "object has no valid IK path")
            message = f"skipped {detection_id or skipped.get('index', 'object')}: {reason}"
            if message not in warnings:
                warnings.append(message)
            if detection_id:
                replacement = {
                    "detection_id": detection_id,
                    "color": skipped.get("color"),
                    "ok": False,
                    "reason_code": "ik_unreachable",
                    "reason": reason,
                    "message": message,
                }
                if detection_id in existing_ignored:
                    for index, item in enumerate(ignored):
                        if isinstance(item, dict) and str(item.get("detection_id")) == detection_id:
                            ignored[index] = {**item, **replacement}
                else:
                    ignored.append(replacement)
                    existing_ignored.add(detection_id)
        task_preview["warnings"] = warnings
        task_preview["ignored_detections"] = ignored
    task_preview["bindings"] = bindings
    sequence["task_preview"] = task_preview
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
    task_preview["estimated_duration_s"] = preview_result["preview"].get("trajectory", {}).get("duration_s", 0.0)
    sequence["task_preview"] = task_preview
    preview_result["preview"]["task_bindings"] = bindings
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
        "selected_detection_ids": [str(value) for value in request.selected_detection_ids or []],
        "detection_snapshot_id": detection_snapshot_id,
        "detection_captured_at": detection_captured_at,
        "detection_fingerprint": detection_fingerprint,
        "detection_task_fingerprint": detection_task_fingerprint,
        "detection_snapshot_server_bound": detection_snapshot_server_bound,
        "task_settings_revision": settings_revision,
        "destination_revision": destination_revision,
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
    stale_reason = task_preview_stale_reason(preview)
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
    aligned, alignment_reason = await ensure_shoulder_alignment_before_motion(
        "task",
        preview.get("settings", {}),
    )
    if not aligned:
        state.set_error(alignment_reason)
        await broadcast_state()
        return {"ok": False, "error": alignment_reason, "state": state.to_dict()}
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
    if strategy == "closed_loop" and state.simulation and not simulation_vision_queue:
        state.set_error("closed-loop simulation requires queued synthetic vision frames")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if strategy == "closed_loop" and not state.simulation and not camera_settings(config).get("enabled"):
        state.set_error("closed-loop task execution requires an enabled camera")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    state.clear_error()
    if state.motion_state == MotionState.STOPPED:
        state.motion_state = MotionState.IDLE
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
                    "cycle_confirmation": preview.get("task_settings", {}).get("cycle_confirmation", "automatic"),
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


@app.post("/api/task/continue")
async def continue_task(request: TaskContinueRequest) -> dict[str, Any]:
    execution = state.task_execution or {}
    if execution.get("run_id") != request.run_id:
        state.set_error("task continuation run ID does not match the active task")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if execution.get("status") != "waiting_for_confirmation":
        state.set_error("task is not waiting for operator confirmation")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    event = task_confirmation_events.get(request.run_id)
    if event is None:
        state.set_error("task confirmation waiter is not available")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    update_task_execution(
        status="running",
        phase="confirmation_received",
        current_step={"label": "continuing", "kind": "operator"},
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
        finish_task_execution("stopped", "task stop requested")
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
            reason = hardware_trajectory_start_blocking_reason()
            if reason:
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
            "motion_contract": trajectory.get("motion_contract", {}),
            "limit_summary": trajectory.get("limit_summary", {}),
            "completion_feedback": "timed + STATUS estimate for hardware",
        }
        attach_controller_command_to_motion_contract(
            preview,
            anticipated_controller_command(str(trajectory.get("mode", "joint"))),
            settings,
        )
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
    verified = True
    verification_message = ""
    if not state.simulation and serial_client.is_connected and not state.encoder_fault:
        verified, verification_message = await verify_shoulder_after_motion(
            "cartesian_jog_stop",
            state.estimated_angles_deg.copy(),
            allow_correction=False,
        )
    await broadcast_state()
    return {
        "ok": verified and state.motion_state != MotionState.FAULT,
        "verification": verification_message,
        "state": state.to_dict(),
    }


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
        motion_contract = cartesian_jog_motion_contract(
            settings,
            tcp_speed_mm_s=tcp_limit,
            phi_speed_deg_s=phi_limit,
        )
        run_id = start_motion_diagnostics(
            source="cartesian_jog",
            mode="cartesian_jog",
            target_deg=state.reported_angles_deg,
            expected_duration_s=0.0,
            waypoint_count=0,
            motion_contract=motion_contract,
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


def _coerce_task_destination_updates(updates: dict[str, Any]) -> bool:
    raw_task_destinations = updates.get("task_destinations")
    raw_drop_zones = updates.get("drop_zones")
    destinations: dict[str, dict[str, Any]] | None = None
    if isinstance(raw_task_destinations, dict):
        destinations = _task_destinations_from_request(raw_task_destinations)
    elif isinstance(raw_drop_zones, dict):
        destinations = _task_destinations_from_request(raw_drop_zones)

    if destinations is None:
        return False
    updates["task_destinations"] = task_destination_payload(destinations)
    updates["drop_zones"] = legacy_drop_zones_from_task_destinations(destinations)
    return True


@app.post("/api/config/calibration")
async def save_calibration(request: CalibrationRequest) -> dict[str, Any]:
    config_path = ensure_local_config()
    updates = request.__dict__
    if isinstance(updates.get("encoders"), dict):
        updates["encoders"] = normalize_encoder_settings(updates["encoders"])
    if isinstance(updates.get("color_profiles"), dict):
        updates["color_profiles"] = _persisted_color_profiles(updates["color_profiles"])
    try:
        task_destinations_changed = _coerce_task_destination_updates(updates)
    except ValueError as exc:
        state.set_error(str(exc))
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    if isinstance(updates.get("tools"), dict):
        tool_errors = validate_tools_payload(updates["tools"])
        if tool_errors:
            state.set_error("; ".join(tool_errors))
            await broadcast_state()
            return {"ok": False, "errors": tool_errors, "error": state.last_error, "state": state.to_dict()}
        _invalidate_changed_tool_validations(tools_settings(config), updates["tools"])
        calibration = calibration_settings(config)
        requested_calibration = updates.get("calibration")
        if isinstance(requested_calibration, dict):
            calibration.update(deepcopy(requested_calibration))
        calibration["tool_dimensions_validated"] = _active_tool_validation_from_payload(
            updates["tools"],
            bool(calibration.get("tool_dimensions_validated", False)),
        )
        updates["calibration"] = calibration
    try:
        with tempfile.TemporaryDirectory() as tmp_dir:
            draft_path = Path(tmp_dir) / "robot.local.yaml"
            shutil.copyfile(config_path, draft_path)
            save_calibration_updates(draft_path, updates)
            draft_config = load_config(draft_path)
            encoder_errors = validate_encoder_settings(
                draft_config,
                encoder_settings(draft_config),
            )
            if encoder_errors:
                state.set_error("; ".join(encoder_errors))
                await broadcast_state()
                return {
                    "ok": False,
                    "errors": encoder_errors,
                    "error": state.last_error,
                    "state": state.to_dict(),
                }
            if task_destinations_changed:
                destination_errors = task_destination_errors(draft_config, named_positions(draft_config))
                if destination_errors:
                    state.set_error("one or more task destinations are invalid")
                    await broadcast_state()
                    return {
                        "ok": False,
                        "errors": destination_errors,
                        "error": state.last_error,
                        "state": state.to_dict(),
                    }
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
        state.controller_capabilities = {"simulation": True}
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
        state.controller_capabilities = {}
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
    state.controller_capabilities = parse_hello_capabilities(hello)
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
    state.controller_capabilities = {}
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
async def home(request: HomeRequest | None = None) -> dict[str, Any]:
    response = await start_joint_target_trajectory(
        config.home_pose,
        "home",
        home_path_settings(request.settings if request else None),
    )
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
        finish_task_execution("stopped", "STOP")
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
    verified = True
    verification_message = ""
    if not state.simulation and serial_client.is_connected and not state.encoder_fault:
        verified, verification_message = await verify_shoulder_after_motion(
            "stop",
            state.estimated_angles_deg.copy(),
            allow_correction=False,
        )
    await broadcast_state()
    return {
        "ok": verified,
        "verification": verification_message,
        "state": state.to_dict(),
    }


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
        finish_task_execution("stopped", "ESTOP")
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
