import asyncio

import app.main as main
from app.kinematics import forward_kinematics
from app.robot_state import MotionState


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
        if self.repeated_status:
            return self.repeated_status
        return ""

    def read_until_prefix(self, prefix, timeout_s=2.0):
        prefixes = prefix if isinstance(prefix, tuple) else (prefix,)
        for _ in range(20):
            line = self.read_line()
            if any(line.startswith(item) for item in prefixes):
                return line
        raise RuntimeError(f"timed out waiting for {prefix}")


def reset_hardware_state() -> None:
    main.cancel_motion_tasks()
    main.active_motion_run_id = None
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = True
    main.state.live_motion_enabled = False
    main.state.known_pose = True
    main.state.pose_source = "setpose"
    main.state.config_sync_status = "synced"
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = main.config.home_pose.copy()
    main.state.target_angles_deg = main.config.home_pose.copy()
    main.state.last_status_line = ""
    main.state.last_controller_response = ""
    main.state.motion_diagnostics = {}
    main.state.motion_execution_state = "idle"
    main.state.clear_error()


def status_line_for(angles, state="idle"):
    return (
        f"STATUS state={state} homed=1 known=1 pose_source=setpose armed=1 "
        f"hw=mixed enabled=1100 enc=0000 "
        f"j1={angles[0]} j2={angles[1]} j3={angles[2]} j4={angles[3]} fault=OK"
    )


def test_set_targets_reads_movej_ack_before_status(monkeypatch):
    reset_hardware_state()
    target = main.config.home_pose.copy()
    target[0] += 1.0
    fake = FakeSerial(["OK command=MOVEJ hw=mixed", status_line_for(target)])
    monkeypatch.setattr(main, "serial_client", fake)
    monkeypatch.setattr(main, "hardware_ready_for_motion", lambda: (True, ""))
    main.start_motion_diagnostics(
        source="test",
        mode="joint_endpoint",
        target_deg=target,
        expected_duration_s=0.1,
        waypoint_count=1,
    )

    result = main.set_targets(target, "test_move", speed_deg_s=10.0, accel_deg_s2=10.0)

    assert result["ok"]
    assert fake.sent[0].startswith("MOVEJ")
    assert fake.sent[1] == "STATUS"
    assert main.state.last_controller_response == "OK command=MOVEJ hw=mixed"
    assert main.state.last_status_line.startswith("STATUS state=idle")


def test_set_targets_surfaces_movej_error_without_status_poll(monkeypatch):
    reset_hardware_state()
    target = main.config.home_pose.copy()
    target[0] += 1.0
    fake = FakeSerial(["ERR code=LIMIT message=j1_out_of_range"])
    monkeypatch.setattr(main, "serial_client", fake)
    monkeypatch.setattr(main, "hardware_ready_for_motion", lambda: (True, ""))
    main.start_motion_diagnostics(
        source="test",
        mode="joint_endpoint",
        target_deg=target,
        expected_duration_s=0.1,
        waypoint_count=1,
    )

    result = main.set_targets(target, "test_move", speed_deg_s=10.0, accel_deg_s2=10.0)

    assert not result["ok"]
    assert "j1_out_of_range" in result["error"]
    assert fake.sent[0].startswith("MOVEJ")
    assert "STATUS" not in fake.sent
    assert main.state.motion_diagnostics["result"] == "failed"


def test_waypoint_path_uses_queued_trajectory_protocol(monkeypatch):
    reset_hardware_state()
    start = main.config.home_pose.copy()
    target = start.copy()
    target[0] += 5.0
    fake = FakeSerial(
        [
            "OK command=TRAJ_BEGIN count=2",
            "OK command=TRAJ_POINT index=0",
            "OK command=TRAJ_POINT index=1",
            "OK command=TRAJ_START count=2 duration=1.000",
            status_line_for(target, state="idle"),
        ]
    )
    monkeypatch.setattr(main, "serial_client", fake)
    monkeypatch.setattr(main, "hardware_ready_for_motion", lambda: (True, ""))
    preview = {
        "id": "test-preview",
        "source": "test",
        "mode": "linear",
        "settings": {"global_speed_deg_s": 10.0, "global_accel_deg_s2": 20.0},
        "trajectory": {
            "ok": True,
            "mode": "linear",
            "duration_s": 1.0,
            "waypoint_count": 2,
            "waypoints": [start, target],
            "segment_durations_s": [0.0, 1.0],
            "time_from_start_s": [0.0, 1.0],
            "errors": [],
        },
    }

    asyncio.run(main.execute_waypoint_path(preview))

    assert fake.sent[0].startswith("TRAJ BEGIN")
    assert fake.sent[1].startswith("TRAJ POINT index=0")
    assert fake.sent[2].startswith("TRAJ POINT index=1")
    assert fake.sent[3] == "TRAJ START"
    assert not any(line.startswith("MOVEJ") for line in fake.sent)
    assert "STATUS" in fake.sent
    assert main.state.motion_diagnostics["result"] == "reached"


def test_wait_for_hardware_target_times_out_when_status_never_reaches_idle(monkeypatch):
    reset_hardware_state()
    target = main.config.home_pose.copy()
    target[0] += 1.0
    fake = FakeSerial(repeated_status=status_line_for(main.config.home_pose, state="moving"))
    monkeypatch.setattr(main, "serial_client", fake)

    ok, message = asyncio.run(
        main.wait_for_hardware_target(target, timeout_s=0.01, poll_interval_s=0.001)
    )

    assert not ok
    assert "timeout" in message
    assert "STATUS" in fake.sent


def test_motion_diagnostics_records_progress_and_actual_tcp_samples():
    reset_hardware_state()
    target = main.config.home_pose.copy()
    target[0] += 10.0

    run_id = main.start_motion_diagnostics(
        source="test",
        mode="joint_endpoint",
        target_deg=target,
        expected_duration_s=0.5,
        waypoint_count=1,
    )
    main.update_motion_diagnostics(
        run_id,
        result="executing",
        execution_state="executing",
        current_waypoint_index=1,
        current_waypoint_total=1,
    )
    main.state.reported_angles_deg = target.copy()
    main.state.fk = forward_kinematics(main.state.reported_angles_deg, main.config.links)
    main.record_motion_sample(run_id)

    diagnostics = main.state.motion_diagnostics
    assert diagnostics["run_id"] == run_id
    assert diagnostics["current_waypoint_index"] == 1
    assert diagnostics["progress_ratio"] > 0.95
    assert len(diagnostics["actual_tcp_path"]) >= 2

    main.finish_motion_diagnostics("reached", run_id=run_id)
    assert main.state.motion_diagnostics["result"] == "reached"
    assert main.state.motion_diagnostics["progress_ratio"] == 1.0


def test_stale_motion_run_cannot_overwrite_active_diagnostics():
    reset_hardware_state()
    first_target = main.config.home_pose.copy()
    first_target[0] += 5.0
    second_target = main.config.home_pose.copy()
    second_target[1] += 5.0

    first_run_id = main.start_motion_diagnostics(
        source="first",
        mode="joint_endpoint",
        target_deg=first_target,
        expected_duration_s=0.5,
        waypoint_count=1,
    )
    second_run_id = main.start_motion_diagnostics(
        source="second",
        mode="joint_endpoint",
        target_deg=second_target,
        expected_duration_s=0.5,
        waypoint_count=1,
    )

    main.finish_motion_diagnostics("stopped", "old cancellation", first_run_id)

    assert main.state.motion_diagnostics["run_id"] == second_run_id
    assert main.state.motion_diagnostics["source"] == "second"
    assert main.state.motion_diagnostics["result"] == "queued"

    main.finish_motion_diagnostics("stopped", "new cancellation", second_run_id)
    assert main.state.motion_diagnostics["result"] == "stopped"
