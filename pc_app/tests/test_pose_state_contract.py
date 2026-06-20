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

    async def scenario():
        response = await main.home()
        assert response["ok"]
        for _ in range(500):
            apply_simulation_step(main.state, main.limiter, 0.05)
            await asyncio.sleep(0)
            if main.state.motion_state == MotionState.IDLE:
                break
        await asyncio.wait_for(main.path_task, timeout=1.0)
        return response

    response = asyncio.run(scenario())

    assert response["command"] == "home"
    assert main.state.reported_angles_deg == pytest.approx(config.home_pose, abs=0.08)
    assert main.state.pose_revision > revision_before
    assert main.state.homed is False
    assert main.state.pose_source == "simulation"


def test_go_home_hardware_sends_one_move_command_and_no_home_command(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "example-test-config")
    away = moved_pose(config, amount=5.0)
    target_status = status_line_for(config.home_pose)
    fake = FakeSerial(
        responses=["OK command=MOVEJ hw=mixed", target_status],
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
    assert len([line for line in movement_commands if line.startswith("MOVEJ")]) == 1
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
