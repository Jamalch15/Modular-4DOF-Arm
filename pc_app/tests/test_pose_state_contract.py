import asyncio
from copy import deepcopy

import pytest
from fastapi.testclient import TestClient

from app import main
from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.motion import RateLimitedMotion
from app.robot_state import MotionState, RobotState
from app.simulator import apply_simulation_step


class FakeSerial:
    def __init__(self, responses=None, repeated_status=None):
        self.responses = list(responses or [])
        self.repeated_status = repeated_status
        self.sent = []
        self.connection = object()

    @property
    def is_connected(self):
        return True

    def clear_input(self):
        pass

    def send_line(self, line):
        self.sent.append(line)

    def read_line(self):
        if self.responses:
            return self.responses.pop(0)
        return self.repeated_status or ""

    def read_until_prefix(self, prefix, timeout_s=2.0):
        prefixes = prefix if isinstance(prefix, tuple) else (prefix,)
        for _ in range(20):
            line = self.read_line()
            if any(line.startswith(item) for item in prefixes):
                return line
        raise RuntimeError(f"timed out waiting for {prefix}")


@pytest.fixture(autouse=True)
def restore_runtime_state():
    main.cancel_motion_tasks()
    state_snapshot = deepcopy(main.state.__dict__)
    path_previews = deepcopy(main.path_previews)
    task_previews = deepcopy(main.task_previews)
    path_task = main.path_task
    path_task_source = main.path_task_source
    active_motion_run_id = main.active_motion_run_id
    yield
    main.cancel_motion_tasks()
    main.state.__dict__.clear()
    main.state.__dict__.update(state_snapshot)
    main.path_previews.clear()
    main.path_previews.update(path_previews)
    main.task_previews.clear()
    main.task_previews.update(task_previews)
    main.path_task = path_task
    main.path_task_source = path_task_source
    main.active_motion_run_id = active_motion_run_id


def configure_example_simulation(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "example-test-config")
    limiter = RateLimitedMotion(config, config.home_pose.copy(), config.home_pose.copy())
    monkeypatch.setattr(main, "limiter", limiter)
    main.state.connected = True
    main.state.simulation = True
    main.state.hardware_armed = False
    main.state.motion_state = MotionState.IDLE
    main.state.homed = False
    main.state.update_reported_pose(
        config.home_pose,
        source="simulation",
        known_pose=True,
        force_revision=True,
    )
    main.state.target_angles_deg = config.home_pose.copy()
    limiter.reset(config.home_pose)
    main.path_previews.clear()
    main.task_previews.clear()
    return config


def moved_pose(config, amount=2.0):
    pose = config.home_pose.copy()
    joint = config.joints[0]
    pose[0] = min(joint.max_deg - 0.5, max(joint.min_deg + 0.5, pose[0] + amount))
    if abs(pose[0] - config.home_pose[0]) < 0.2:
        pose[0] = config.home_pose[0] - amount
    return pose


def status_line_for(angles):
    return (
        "STATUS state=idle homed=0 known=1 pose_source=open_loop_estimate armed=1 "
        "hw=mixed enabled=1100 enc=0000 "
        f"j1={angles[0]} j2={angles[1]} j3={angles[2]} j4={angles[3]} fault=OK"
    )


def trajectory_responses_for(trajectory, target, start):
    points = main.trajectory_upload_points(trajectory)
    return (
        [status_line_for(start), status_line_for(start), f"OK command=TRAJ_BEGIN count={len(points)}"]
        + [f"OK command=TRAJ_POINT index={index}" for index in range(len(points))]
        + [f"OK command=TRAJ_START count={len(points)} duration={points[-1][0]:.3f}", status_line_for(target)]
    )


def test_encoder_telemetry_is_separate_from_planning_pose(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {"enabled": False}
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "cs_pin": 12,
            "reference_raw_deg": 0.0,
            "reference_joint_deg": 0.0,
            "direction_sign": 1,
            "sensor_turns_per_joint_turn": 1.0,
            "mounting_location": "joint_output",
            "calibration_validated": True,
            "calibration_id": "fixture",
        }
    )
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    main.state.update_reported_pose(
        [0.0, 20.0, 20.0, 0.0],
        source="setpose",
        known_pose=True,
        force_revision=True,
    )
    pose_revision = main.state.pose_revision
    telemetry_revision = main.state.encoder_telemetry_revision

    main.apply_controller_status(
        "STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=setpose "
        "armed=1 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        "er2=1365 ea2=30.0 em2=30.0 eage2=20 enoise2=0.05 ef2=OK "
        "j1=0 j2=20 j3=20 j4=0 closed_loop=diagnostic correction=idle fault=OK"
    )

    assert main.state.reported_angles_deg == pytest.approx([0.0, 20.0, 20.0, 0.0])
    assert main.state.estimated_angles_deg == pytest.approx([0.0, 20.0, 20.0, 0.0])
    assert main.state.measured_angles_deg[1] == pytest.approx(30.0)
    assert main.state.measurement_valid_mask == "0100"
    assert main.state.pose_revision == pose_revision
    assert main.state.encoder_telemetry_revision > telemetry_revision


def test_encoder_pose_tracking_can_update_idle_shoulder_planning_pose(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {
        "enabled": True,
        "mode": "idle",
        "min_update_delta_deg": 0.05,
        "max_jump_deg": 180.0,
        "set_shoulder_known": True,
    }
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "cs_pin": 12,
            "reference_raw_deg": 0.0,
            "reference_joint_deg": 0.0,
            "direction_sign": 1,
            "sensor_turns_per_joint_turn": 1.0,
            "mounting_location": "joint_output",
            "calibration_validated": True,
            "calibration_id": "fixture",
        }
    )
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    main.state.connected = True
    main.state.simulation = False
    main.state.motion_state = MotionState.IDLE
    main.state.motion_diagnostics = {}
    main.state.pending_motion = {}
    main.state.update_reported_pose(
        [0.0, 20.0, 20.0, 0.0],
        source="unknown",
        known_mask="0000",
        force_revision=True,
    )
    main.state.target_angles_deg = [0.0, 20.0, 20.0, 0.0]
    pose_revision = main.state.pose_revision

    main.apply_controller_status(
        "STATUS state=idle homed=0 known=0 known_mask=0000 pose_source=unknown "
        "armed=0 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        "er2=1365 ea2=30.0 em2=30.0 eage2=20 enoise2=0.05 ef2=OK "
        "j1=0 j2=20 j3=20 j4=0 closed_loop=diagnostic correction=idle fault=OK"
    )

    assert main.state.reported_angles_deg == pytest.approx([0.0, 30.0, 20.0, 0.0])
    assert main.state.estimated_angles_deg == pytest.approx([0.0, 30.0, 20.0, 0.0])
    assert main.state.target_angles_deg[1] == pytest.approx(30.0)
    assert main.state.pose_known_mask == "0100"
    assert main.state.known_pose is False
    assert main.state.joint_authority[1] == "measured"
    assert main.state.pose_revision > pose_revision
    assert main.state.encoder_mismatch["pose_tracking_status"] == "applied"


def test_encoder_pose_tracking_does_not_update_while_controller_reports_moving(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {"enabled": True, "mode": "idle"}
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "cs_pin": 12,
            "reference_raw_deg": 0.0,
            "reference_joint_deg": 0.0,
            "direction_sign": 1,
            "sensor_turns_per_joint_turn": 1.0,
            "mounting_location": "joint_output",
            "calibration_validated": True,
            "calibration_id": "fixture",
        }
    )
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    main.state.connected = True
    main.state.simulation = False
    main.state.update_reported_pose(
        [0.0, 20.0, 20.0, 0.0],
        source="setpose",
        known_pose=True,
        force_revision=True,
    )
    main.state.target_angles_deg = [0.0, 20.0, 20.0, 0.0]
    main.state.motion_diagnostics = {"result": "executing"}
    pose_revision = main.state.pose_revision

    main.apply_controller_status(
        "STATUS state=moving homed=0 known=1 known_mask=1111 pose_source=open_loop_estimate "
        "armed=1 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        "er2=1365 ea2=30.0 em2=30.0 eage2=20 enoise2=0.05 ef2=OK "
        "j1=0 j2=20 j3=20 j4=0 closed_loop=diagnostic correction=idle fault=OK"
    )

    assert main.state.reported_angles_deg[1] == pytest.approx(20.0)
    assert main.state.measured_angles_deg[1] == pytest.approx(30.0)


def test_encoder_pose_tracking_updates_idle_status_even_with_pending_motion_diagnostics(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {"enabled": True, "mode": "idle"}
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "cs_pin": 12,
            "reference_raw_deg": 0.0,
            "reference_joint_deg": 0.0,
            "direction_sign": 1,
            "sensor_turns_per_joint_turn": 1.0,
            "mounting_location": "joint_output",
            "calibration_validated": True,
            "calibration_id": "fixture",
        }
    )
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    main.state.connected = True
    main.state.simulation = False
    main.state.update_reported_pose(
        [0.0, 20.0, 20.0, 0.0],
        source="setpose",
        known_pose=True,
        force_revision=True,
    )
    main.state.target_angles_deg = [0.0, 20.0, 20.0, 0.0]
    main.state.motion_diagnostics = {"result": "executing"}
    main.state.pending_motion = {"status": "queued"}

    main.apply_controller_status(
        "STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=open_loop_estimate "
        "armed=1 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        "er2=1365 ea2=30.0 em2=30.0 eage2=20 enoise2=0.05 ef2=OK "
        "j1=0 j2=20 j3=20 j4=0 closed_loop=diagnostic correction=idle fault=OK"
    )

    assert main.state.reported_angles_deg[1] == pytest.approx(30.0)
    assert main.state.target_angles_deg[1] == pytest.approx(30.0)
    assert main.state.pose_source == "encoder_shoulder_tracking"


def test_hardware_joint_move_blocks_when_encoder_tracked_pose_not_rebased_to_controller(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {
        "enabled": True,
        "mode": "idle",
        "min_update_delta_deg": 0.05,
        "max_jump_deg": 180.0,
        "set_shoulder_known": True,
    }
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "cs_pin": 15,
            "reference_raw_deg": 0.0,
            "reference_joint_deg": 0.0,
            "direction_sign": 1,
            "sensor_turns_per_joint_turn": 1.0,
            "mounting_location": "joint_output",
            "calibration_validated": True,
            "calibration_id": "fixture",
        }
    )
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "encoder-planning-refresh-test")
    status = (
        "STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=open_loop_estimate "
        "armed=1 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        "er2=4789 ea2=105.264 em2=105.264 eage2=20 enoise2=0.05 evalidn2=4 ef2=OK "
        "j1=0 j2=93.78 j3=20 j4=0 closed_loop=diagnostic correction=idle fault=OK"
    )
    fake = FakeSerial(repeated_status=status)
    monkeypatch.setattr(main, "serial_client", fake)
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = True
    main.state.config_sync_status = "synced"
    main.state.motion_state = MotionState.IDLE
    main.state.motion_diagnostics = {}
    main.state.pending_motion = {}
    main.state.update_reported_pose(
        [0.0, 93.78, 20.0, 0.0],
        source="open_loop_estimate",
        known_pose=True,
        force_revision=True,
    )
    main.state.target_angles_deg = [0.0, 93.78, 20.0, 0.0]
    main.path_previews.clear()
    main.path_task = None
    main.path_task_source = None

    async def scenario():
        response = await main.start_joint_target_trajectory(
            [0.0, 100.0, 20.0, 0.0],
            "set_all_joint_targets",
            main.PathSettingsRequest(global_speed_deg_s=20.0, global_accel_deg_s2=40.0),
        )
        if response.get("ok") and main.path_task is not None:
            await asyncio.wait_for(main.path_task, timeout=1.0)
        return response

    response = asyncio.run(scenario())

    assert response["ok"] is False
    assert "controller step position is not synced" in response["error"]
    assert main.path_task is None or main.path_task.done()
    assert not any(line.startswith(("TRAJ", "MOVEJ")) for line in fake.sent)
    assert main.state.reported_angles_deg[1] == pytest.approx(105.264)
    assert main.state.pose_source == "encoder_shoulder_tracking"
    assert main.state.encoder_mismatch["controller_pose_rebase_required"] is True


def test_small_encoder_tracked_delta_inside_correction_deadband_does_not_force_rebase(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {
        "enabled": True,
        "mode": "idle",
        "min_update_delta_deg": 0.05,
        "max_jump_deg": 180.0,
        "set_shoulder_known": True,
    }
    raw["encoders"]["correction"].update(
        {
            "enabled": True,
            "validation_id": "validated",
            "deadband_deg": 0.75,
            "max_delta_deg": 8.0,
        }
    )
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "cs_pin": 15,
            "reference_raw_deg": 0.0,
            "reference_joint_deg": 0.0,
            "direction_sign": 1,
            "sensor_turns_per_joint_turn": 1.0,
            "mounting_location": "joint_output",
            "calibration_validated": True,
            "calibration_id": "fixture",
        }
    )
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "encoder-small-rebase-deadband-test")
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = True
    main.state.config_sync_status = "synced"
    main.state.motion_state = MotionState.IDLE
    main.state.update_reported_pose(
        [0.0, 90.0, 20.0, 0.0],
        source="open_loop_estimate",
        known_pose=True,
        force_revision=True,
    )
    main.state.target_angles_deg = [0.0, 90.0, 20.0, 0.0]

    main.apply_controller_status(
        "STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=open_loop_estimate "
        "armed=1 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        "em2=90.410 eage2=20 enoise2=0.05 evalidn2=4 ef2=OK "
        "j1=0 j2=90.000 j3=20 j4=0 closed_loop=diagnostic correction=idle fault=OK"
    )

    assert main.state.reported_angles_deg[1] == pytest.approx(90.41)
    assert main.state.pose_source == "encoder_shoulder_tracking"
    assert main.state.encoder_mismatch["pose_tracking_status"] == "applied"
    assert main.state.encoder_mismatch["controller_pose_rebase_required"] is False
    assert main.controller_pose_rebase_blocking_reason() is None


def test_completed_correction_bias_does_not_rewrite_planning_target_or_force_rebase(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {
        "enabled": True,
        "mode": "idle",
        "min_update_delta_deg": 0.05,
        "max_jump_deg": 180.0,
        "set_shoulder_known": True,
    }
    raw["encoders"]["correction"].update(
        {
            "enabled": True,
            "validation_id": "validated",
            "deadband_deg": 0.75,
            "max_delta_deg": 8.0,
        }
    )
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "cs_pin": 15,
            "reference_raw_deg": 0.0,
            "reference_joint_deg": 0.0,
            "direction_sign": 1,
            "sensor_turns_per_joint_turn": 1.0,
            "mounting_location": "joint_output",
            "calibration_validated": True,
            "calibration_id": "fixture",
        }
    )
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "encoder-post-correction-rebase-test")
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = True
    main.state.config_sync_status = "synced"
    main.state.motion_state = MotionState.IDLE
    main.state.correction_state = {"state": "executing", "bias_deg": [None] * len(config.joints)}
    main.update_controller_rebase_state(
        required=True,
        controller_deg=90.0,
        tracked_deg=91.7,
        delta_deg=1.7,
        reason="pre-correction tracking drift",
    )
    main.state.update_reported_pose(
        [0.0, 90.0, 20.0, 0.0],
        source="open_loop_estimate",
        known_pose=True,
        force_revision=True,
    )
    main.state.target_angles_deg = [0.0, 90.0, 20.0, 0.0]

    main.apply_controller_status(
        "STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=open_loop_estimate "
        "armed=1 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        "em2=92.000 eage2=20 enoise2=0.05 evalidn2=4 ef2=OK "
        "j1=0 j2=90.000 j3=20 j4=0 closed_loop=diagnostic "
        "correction=completed correction_id=tx-1 correction_delta=2.000000 "
        "correction_steps=100 correction_attempts=1 cb1=0 cb2=2.0000 cb3=0 cb4=0 fault=OK"
    )

    assert main.state.reported_angles_deg[1] == pytest.approx(90.0)
    assert main.state.target_angles_deg[1] == pytest.approx(90.0)
    assert main.state.pose_source == "open_loop_estimate"
    assert main.state.correction_state["state"] == "completed"
    assert main.state.correction_state["bias_deg"][1] == pytest.approx(2.0)
    assert main.state.encoder_mismatch["pose_tracking_status"] == "held_by_correction_bias"
    assert main.state.encoder_mismatch["controller_pose_rebase_required"] is False


def test_hardware_arm_rebases_encoder_tracked_shoulder_before_enabling(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {
        "enabled": True,
        "mode": "idle",
        "min_update_delta_deg": 0.05,
        "max_jump_deg": 180.0,
        "set_shoulder_known": True,
    }
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "cs_pin": 15,
            "reference_raw_deg": 0.0,
            "reference_joint_deg": 0.0,
            "direction_sign": 1,
            "sensor_turns_per_joint_turn": 1.0,
            "mounting_location": "joint_output",
            "calibration_validated": True,
            "calibration_id": "fixture",
        }
    )
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "encoder-arm-rebase-test")
    stale_status = (
        "STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=open_loop_estimate "
        "armed=0 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        "er2=4789 ea2=105.264 em2=105.264 eage2=20 enoise2=0.05 evalidn2=4 ef2=OK "
        "j1=0 j2=93.78 j3=20 j4=0 closed_loop=diagnostic correction=idle fault=OK"
    )
    rebased_status = (
        "STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=setpose "
        "armed=0 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        "er2=4789 ea2=105.264 em2=105.264 eage2=20 enoise2=0.05 evalidn2=4 ef2=OK "
        "j1=0 j2=105.264 j3=20 j4=0 closed_loop=diagnostic correction=idle fault=OK"
    )
    armed_status = rebased_status.replace("armed=0", "armed=1")
    fake = FakeSerial(
        responses=[
            stale_status,
            "OK command=SETPOSE",
            rebased_status,
            "OK command=ARM armed=1",
            armed_status,
        ]
    )
    monkeypatch.setattr(main, "serial_client", fake)
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = False
    main.state.config_sync_status = "synced"
    main.state.motion_state = MotionState.IDLE
    main.state.motion_diagnostics = {}
    main.state.pending_motion = {}
    main.state.update_reported_pose(
        [0.0, 93.78, 20.0, 0.0],
        source="open_loop_estimate",
        known_pose=True,
        force_revision=True,
    )
    main.state.target_angles_deg = [0.0, 93.78, 20.0, 0.0]

    response = asyncio.run(main.set_hardware_arm(main.ArmRequest(armed=True)))

    assert response["ok"] is True
    assert any(line.startswith("SETPOSE 0.000 105.264 20.000 0.000") for line in fake.sent)
    assert fake.sent.index(next(line for line in fake.sent if line.startswith("SETPOSE"))) < fake.sent.index("ARM 1")
    assert main.state.hardware_armed is True
    assert main.state.reported_angles_deg[1] == pytest.approx(105.264)
    assert main.state.encoder_mismatch["controller_pose_rebase_required"] is False


def test_encoder_pose_tracking_drift_does_not_make_preview_stale(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {
        "enabled": True,
        "mode": "idle",
        "preview_stale_tolerance_deg": 2.0,
    }
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "encoder-preview-test")
    start = [0.0, 136.209, 20.0, 0.0]
    current = [0.0, 135.264, 20.0, 0.0]
    target = [0.0, 120.0, 20.0, 0.0]
    main.state.connected = True
    main.state.simulation = False
    main.state.update_reported_pose(
        start,
        source="encoder_shoulder_tracking",
        known_mask="0100",
        force_revision=True,
    )
    main.state.encoder_mismatch = {"pose_tracking_status": "applied"}
    preview = {
        "id": "encoder-drift-preview",
        "trajectory": {"waypoints": [start.copy(), target.copy()]},
        **main.pose_snapshot_fields(),
    }

    main.state.update_reported_pose(
        current,
        source="encoder_shoulder_tracking",
        known_mask="0100",
        force_revision=True,
    )
    main.state.encoder_mismatch = {"pose_tracking_status": "applied"}

    assert main.preview_stale_reason(preview) is None
    assert main.rebase_preview_start_to_current_if_encoder_tracked(preview) is True
    assert preview["start_reported_angles_deg"] == pytest.approx(current)
    assert preview["trajectory"]["waypoints"][0] == pytest.approx(current)
    assert preview["trajectory"]["waypoints"][-1] == pytest.approx(target)


def test_encoder_tracked_shoulder_preview_tolerance_does_not_depend_on_last_mismatch_status(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {
        "enabled": True,
        "mode": "idle",
        "preview_stale_tolerance_deg": 2.0,
    }
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "encoder-preview-test")
    start = [0.0, 136.209, 20.0, 0.0]
    current = [0.0, 135.264, 20.0, 0.0]
    main.state.connected = True
    main.state.simulation = False
    main.state.update_reported_pose(
        start,
        source="encoder_shoulder_tracking",
        known_mask="0100",
        force_revision=True,
    )
    preview = {
        "id": "encoder-drift-preview",
        "trajectory": {"waypoints": [start.copy(), [0.0, 120.0, 20.0, 0.0]]},
        **main.pose_snapshot_fields(),
    }
    main.state.update_reported_pose(
        current,
        source="encoder_shoulder_tracking",
        known_mask="0100",
        force_revision=True,
    )
    main.state.joint_authority[1] = "measured"
    main.state.measured_angles_deg[1] = current[1]
    main.state.encoder_mismatch = {"status": "diagnostic"}

    assert main.preview_stale_reason(preview) is None


def test_encoder_pose_tracking_large_drift_still_makes_preview_stale(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["pose_tracking"] = {
        "enabled": True,
        "mode": "idle",
        "preview_stale_tolerance_deg": 2.0,
    }
    patched_config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched_config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "encoder-preview-test")
    main.state.connected = True
    main.state.simulation = False
    main.state.update_reported_pose(
        [0.0, 136.209, 20.0, 0.0],
        source="encoder_shoulder_tracking",
        known_mask="0100",
        force_revision=True,
    )
    main.state.encoder_mismatch = {"pose_tracking_status": "applied"}
    preview = {
        "id": "encoder-large-drift-preview",
        "trajectory": {"waypoints": [[0.0, 136.209, 20.0, 0.0], [0.0, 120.0, 20.0, 0.0]]},
        **main.pose_snapshot_fields(),
    }

    main.state.update_reported_pose(
        [0.0, 133.5, 20.0, 0.0],
        source="encoder_shoulder_tracking",
        known_mask="0100",
        force_revision=True,
    )
    main.state.encoder_mismatch = {"pose_tracking_status": "applied"}

    reason = main.preview_stale_reason(preview)
    assert reason is not None
    assert "preview start pose is stale" in reason


def test_legacy_encoder_status_cannot_establish_whole_pose_authority(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.update_reported_pose(
        config.home_pose,
        source="unknown",
        known_pose=False,
        force_revision=True,
    )

    main.apply_controller_status(
        "STATUS state=idle homed=0 known=1 pose_source=mixed armed=0 "
        "hw=mixed enabled=1100 enc=0100 e2=30.0 "
        "j1=0 j2=20 j3=20 j4=0 closed_loop=readback fault=OK"
    )

    assert main.state.known_pose is False
    assert main.state.pose_known_mask == "0000"
    assert main.state.pose_source == "unknown"
    assert main.state.reported_angles_deg[1] == pytest.approx(20.0)


def test_robot_state_pose_revision_changes_only_for_authoritative_transitions():
    state = RobotState(
        joint_names=["j1", "j2", "j3", "j4"],
        target_angles_deg=[0.0] * 4,
        reported_angles_deg=[0.0] * 4,
    )

    assert state.update_reported_pose([0.0] * 4, source="simulation", known_pose=True) is False
    assert state.pose_revision == 0

    assert state.update_reported_pose([1.0, 0.0, 0.0, 0.0], source="simulation", known_pose=True) is True
    assert state.pose_revision == 1
    assert state.pose_known_mask == "1111"

    assert state.update_reported_pose(
        [1.0, 0.0, 0.0, 0.0],
        source="setpose",
        known_pose=True,
        force_revision=True,
    ) is True
    assert state.pose_revision == 2
    assert state.to_dict()["commanded_target_deg"] == [0.0] * 4


@pytest.mark.parametrize("mode", ["joint", "linear", "program"])
def test_path_execute_rejects_preview_after_pose_moves(monkeypatch, mode):
    config = configure_example_simulation(monkeypatch)
    preview_id = f"stale-{mode}"
    preview = {
        "id": preview_id,
        "mode": mode,
        "program_revision": 3 if mode == "program" else None,
        "trajectory": {"mode": mode, "waypoints": [config.home_pose]},
        **main.pose_snapshot_fields(),
    }
    main.path_previews[preview_id] = preview
    main.state.update_reported_pose(moved_pose(config), source="simulation", known_pose=True)

    payload = TestClient(main.app).post(
        "/api/path/execute",
        json={"preview_id": preview_id, "program_revision": 3 if mode == "program" else None},
    ).json()

    assert payload["ok"] is False
    assert "preview start pose is stale" in payload["error"]
    assert "preview again" in payload["error"]
    assert str(preview["start_pose_revision"]) in payload["error"]
    assert str(main.state.pose_revision) in payload["error"]


def test_task_execute_rejects_preview_after_pose_moves(monkeypatch):
    config = configure_example_simulation(monkeypatch)
    preview_id = "stale-task"
    main.task_previews[preview_id] = {
        "id": preview_id,
        "created_at": main.time(),
        "config_id": main.RUNNING_CONFIG_ID,
        "model_fingerprint": main.robot_model_fingerprint(),
        "consumed": False,
        **main.pose_snapshot_fields(),
    }
    main.state.update_reported_pose(moved_pose(config), source="simulation", known_pose=True)

    payload = TestClient(main.app).post(
        "/api/task/execute",
        json={"preview_id": preview_id},
    ).json()

    assert payload["ok"] is False
    assert "preview start pose is stale" in payload["error"]
    assert "preview the task again" in payload["error"]


def test_go_home_simulation_moves_on_first_request(monkeypatch):
    config = configure_example_simulation(monkeypatch)
    away = moved_pose(config, amount=8.0)
    main.state.update_reported_pose(away, source="simulation", known_pose=True, force_revision=True)
    main.state.target_angles_deg = away.copy()
    main.limiter.reset(away)
    revision_before = main.state.pose_revision
    requested_settings = main.PathSettingsRequest(
        global_speed_deg_s=11.0,
        global_accel_deg_s2=22.0,
        per_joint_speed_deg_s=[11.0] * 4,
        per_joint_accel_deg_s2=[22.0] * 4,
    )

    async def scenario():
        response = await main.home(main.HomeRequest(settings=requested_settings))
        assert response["ok"]
        await asyncio.wait_for(main.path_task, timeout=4.0)
        return response

    response = asyncio.run(scenario())

    assert response["command"] == "home"
    assert response["preview"]["settings"]["motion_purpose"] == "configured_home_pose_move"
    assert response["preview"]["settings"]["global_speed_deg_s"] == 11.0
    assert response["preview"]["settings"]["global_accel_deg_s2"] == 22.0
    assert response["preview"]["motion_contract"]["controller_command"]["command"] == "SIM_TRAJ"
    assert main.state.reported_angles_deg == pytest.approx(config.home_pose, abs=0.08)
    assert main.state.pose_revision > revision_before
    assert main.state.homed is False
    assert main.state.pose_source == "simulation"


def test_go_home_hardware_sends_timed_trajectory_and_no_home_command(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "example-test-config")
    away = moved_pose(config, amount=5.0)
    target_status = status_line_for(config.home_pose)
    settings = main.home_path_settings(None)
    trajectory = main.build_joint_trajectory(away, config.home_pose, config.joints, settings)
    fake = FakeSerial(
        responses=trajectory_responses_for(trajectory, config.home_pose, away),
        repeated_status=target_status,
    )
    monkeypatch.setattr(main, "serial_client", fake)
    monkeypatch.setattr(main, "hardware_ready_for_motion", lambda: (True, ""))
    monkeypatch.setattr(
        main,
        "limiter",
        RateLimitedMotion(config, away.copy(), away.copy()),
    )
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = True
    main.state.config_sync_status = "synced"
    main.state.motion_state = MotionState.IDLE
    main.state.homed = False
    main.state.update_reported_pose(
        away,
        source="open_loop_estimate",
        known_pose=True,
        force_revision=True,
    )
    main.state.target_angles_deg = away.copy()

    async def scenario():
        response = await main.home()
        assert response["ok"]
        await asyncio.wait_for(main.path_task, timeout=1.0)
        return response

    response = asyncio.run(scenario())
    movement_commands = [
        line for line in fake.sent
        if line.startswith(("MOVEJ", "HOME", "TRAJ START", "SERVOJ", "JOG"))
    ]

    assert response["command"] == "home"
    assert response["preview"]["motion_contract"]["controller_command"]["command"] == "TRAJ"
    assert response["preview"]["motion_contract"]["controller_command"]["uses_planned_timestamps"] is True
    assert len([line for line in movement_commands if line.startswith("TRAJ START")]) == 1
    assert not any(line.startswith("MOVEJ") for line in movement_commands)
    assert not any(line.startswith("HOME") for line in movement_commands)
    assert main.state.reported_angles_deg == pytest.approx(config.home_pose)
    assert main.state.homed is False
    assert main.state.pose_source == "open_loop_estimate"


def test_setpose_revalidates_pose_without_claiming_physical_home(monkeypatch):
    config = configure_example_simulation(monkeypatch)
    main.state.config_change = {
        "pose_invalidated": True,
        "pose_revalidation_required": True,
    }
    angles = moved_pose(config)

    payload = TestClient(main.app).post(
        "/api/hardware/setpose",
        json={"angles_deg": angles},
    ).json()

    assert payload["ok"] is True
    assert payload["state"]["known_pose"] is True
    assert payload["state"]["pose_source"] == "setpose"
    assert payload["state"]["homed"] is False
    assert payload["state"]["config_change"]["pose_revalidation_required"] is False
