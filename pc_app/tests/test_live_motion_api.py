from dataclasses import replace

from pytest import approx
from fastapi.testclient import TestClient

import app.main as main
from app.cartesian_jog_debug import cartesian_path_metrics
from app.kinematics import forward_kinematics
from app.robot_state import MotionState


def reset_runtime_state() -> None:
    main.cancel_motion_tasks()
    main.reset_cartesian_jog_runtime()
    main.state.connected = True
    main.state.simulation = True
    main.state.hardware_armed = False
    main.state.live_motion_enabled = False
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = main.config.home_pose.copy()
    main.state.target_angles_deg = main.config.home_pose.copy()
    main.state.fk = forward_kinematics(main.state.reported_angles_deg, main.config.links)
    main.limiter.reset(main.state.reported_angles_deg)
    main.state.clear_error()


class FakeSerial:
    def __init__(self, responses=None):
        self.responses = list(responses or [])
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
        return self.responses.pop(0) if self.responses else ""

    def read_until_prefix(self, prefix, timeout_s=2.0):
        prefixes = prefix if isinstance(prefix, tuple) else (prefix,)
        while self.responses:
            line = self.read_line()
            if any(line.startswith(item) for item in prefixes):
                return line
        raise RuntimeError(f"timed out waiting for {prefix}")


def test_live_motion_hardware_requires_armed_toggle():
    reset_runtime_state()
    main.state.simulation = False
    main.state.connected = True
    main.state.hardware_armed = False
    client = TestClient(main.app)

    response = client.post("/api/live-motion", json={"enabled": True})

    assert response.status_code == 200
    payload = response.json()
    assert not payload["ok"]
    assert "Armed" in payload["error"]
    assert not payload["state"]["live_motion_enabled"]


def test_live_target_requires_live_motion_enabled():
    reset_runtime_state()
    client = TestClient(main.app)

    response = client.post("/api/live-target", json={"angles_deg": main.config.home_pose})

    assert response.status_code == 200
    payload = response.json()
    assert not payload["ok"]
    assert "disabled" in payload["error"]


def test_live_target_accepts_simulation_joint_target_when_enabled():
    reset_runtime_state()
    client = TestClient(main.app)
    enabled = client.post("/api/live-motion", json={"enabled": True}).json()
    assert enabled["ok"]
    target = main.config.home_pose.copy()
    target[0] += 2.0

    response = client.post(
        "/api/live-target",
        json={
            "angles_deg": target,
            "settings": {"global_speed_deg_s": 20.0, "global_accel_deg_s2": 100.0},
        },
    )

    payload = response.json()
    assert payload["ok"]
    assert payload["preview"]["mode"] == "jog"
    assert payload["preview"]["trajectory"]["waypoints"][-1] == target
    reset_runtime_state()


def test_cartesian_jog_accepts_simulation_velocity_step():
    reset_runtime_state()
    client = TestClient(main.app)
    start_x = main.state.fk.get("x_mm", 0.0)

    response = client.post(
        "/api/cartesian-jog",
        json={
            "vx_mm_s": 0.0,
            "vy_mm_s": 20.0,
            "vz_mm_s": 0.0,
            "vphi_deg_s": 0.0,
            "dt_s": 0.05,
            "tcp_speed_mm_s": 60.0,
        },
    )

    payload = response.json()
    assert payload["ok"], payload
    assert payload["jog"]["target_angles_deg"] != main.config.home_pose
    assert payload["jog"]["requested_delta"]["y_mm"] > 0
    assert main.state.target_angles_deg == payload["jog"]["target_angles_deg"]
    assert main.state.motion_execution_state == "cartesian_jog"
    assert main.state.fk.get("x_mm", 0.0) == approx(start_x, abs=1e-3)
    reset_runtime_state()


def test_cartesian_jog_simulation_endpoint_tracks_straight_z_path():
    reset_runtime_state()
    start = [0.0, 25.0, 80.0, -50.0]
    main.state.reported_angles_deg = start.copy()
    main.state.target_angles_deg = start.copy()
    main.state.fk = forward_kinematics(start, main.config.links)
    main.limiter.reset(start)
    client = TestClient(main.app)

    for _ in range(20):
        response = client.post(
            "/api/cartesian-jog",
            json={
                "vx_mm_s": 0.0,
                "vy_mm_s": 0.0,
                "vz_mm_s": 40.0,
                "vphi_deg_s": 0.0,
                "dt_s": 1.0 / 12.0,
                "tcp_speed_mm_s": 60.0,
            },
        )
        payload = response.json()
        assert payload["ok"], payload

    path = main.state.motion_diagnostics.get("actual_tcp_path", [])
    points = [[sample["x_mm"], sample["y_mm"], sample["z_mm"]] for sample in path]
    metrics = cartesian_path_metrics(points, [0.0, 0.0, 1.0])

    assert metrics["progress_mm"] > 30.0
    assert metrics["alignment"] > 0.99
    assert metrics["max_lateral_mm"] < 4.0
    client.post("/api/cartesian-jog/stop")
    reset_runtime_state()


def test_cartesian_jog_rejects_stale_blocked_goal_and_allows_immediate_reverse():
    reset_runtime_state()
    start = [0.0, 45.0, 25.0, -20.0]
    main.state.reported_angles_deg = start.copy()
    main.state.target_angles_deg = start.copy()
    main.state.fk = forward_kinematics(start, main.config.links)
    main.limiter.reset(start)
    client = TestClient(main.app)

    blocked_payload = None
    for _ in range(40):
        payload = client.post(
            "/api/cartesian-jog",
            json={"vz_mm_s": 40.0, "dt_s": 0.08},
        ).json()
        assert payload["ok"], payload
        if payload["jog"]["blocked"]:
            blocked_payload = payload
            break

    assert blocked_payload is not None
    assert blocked_payload["jog"]["failure_code"] in {"local_step_unreachable", "excessive_lateral_drift"}
    assert blocked_payload["jog"]["achieved_delta"] == {
        "x_mm": 0.0,
        "y_mm": 0.0,
        "z_mm": 0.0,
        "phi_deg": 0.0,
    }
    assert blocked_payload["state"]["motion_state"] == "idle"
    blocked_pose = main.state.reported_angles_deg.copy()

    reverse = client.post(
        "/api/cartesian-jog",
        json={"vz_mm_s": -20.0, "dt_s": 0.08},
    ).json()

    assert reverse["ok"], reverse
    assert not reverse["jog"]["blocked"], reverse["jog"]
    assert reverse["jog"]["requested_delta"]["z_mm"] < 0.0
    assert reverse["jog"]["achieved_delta"]["z_mm"] < 0.0
    assert reverse["jog"]["target_angles_deg"] != blocked_pose
    client.post("/api/cartesian-jog/stop")
    reset_runtime_state()


def test_cartesian_jog_hardware_uses_jogv_protocol(monkeypatch):
    reset_runtime_state()
    main.state.simulation = False
    main.state.hardware_armed = True
    main.state.live_motion_enabled = True
    main.state.config_sync_status = "synced"
    fake = FakeSerial(["OK command=JOGV hw=mixed"])
    monkeypatch.setattr(main, "serial_client", fake)
    monkeypatch.setattr(main, "hardware_ready_for_motion", lambda: (True, ""))
    monkeypatch.setattr(main, "refresh_serial_status", lambda: None)
    client = TestClient(main.app)

    response = client.post(
        "/api/cartesian-jog",
        json={"vx_mm_s": 0.0, "vy_mm_s": 20.0, "vz_mm_s": 0.0, "vphi_deg_s": 0.0, "dt_s": 0.05},
    )

    payload = response.json()
    assert payload["ok"], payload
    assert fake.sent[0].startswith("JOGV")
    assert not fake.sent[0].startswith("JOGJ")
    assert not any(line.startswith("MOVEJ") for line in fake.sent)
    reset_runtime_state()


def test_direct_joint_apply_uses_requested_motion_settings():
    reset_runtime_state()
    client = TestClient(main.app)
    target = main.config.home_pose.copy()
    target[2] += 8.0

    response = client.post(
        "/api/joints",
        json={
            "angles_deg": target,
            "settings": {
                "global_speed_deg_s": 12.0,
                "global_accel_deg_s2": 6.0,
                "waypoint_rate_hz": 20.0,
                "per_joint_speed_deg_s": [12.0, 12.0, 8.0, 12.0],
                "per_joint_accel_deg_s2": [6.0, 6.0, 4.0, 6.0],
            },
        },
    )

    payload = response.json()
    assert payload["ok"]
    trajectory = payload["preview"]["trajectory"]
    assert trajectory["waypoints"][-1] == target
    assert trajectory["speed_limits_deg_s"][2] == 8.0
    assert trajectory["accel_limits_deg_s2"][2] == 4.0
    reset_runtime_state()


def test_default_path_settings_apply_saved_overrides_and_joint_fallbacks():
    reset_runtime_state()

    settings = main.request_settings(None)
    stored = main.config.raw.get("path_defaults", {})

    assert settings["global_speed_deg_s"] == stored.get(
        "global_speed_deg_s",
        min(joint.max_speed_deg_s for joint in main.config.joints),
    )
    assert settings["global_accel_deg_s2"] == stored.get(
        "global_accel_deg_s2",
        main.config.motion.acceleration_deg_s2,
    )
    assert settings["waypoint_rate_hz"] == stored.get(
        "waypoint_rate_hz",
        main.config.motion.command_rate_limit_hz,
    )
    assert settings["per_joint_speed_deg_s"] == stored.get(
        "per_joint_speed_deg_s",
        [joint.max_speed_deg_s for joint in main.config.joints],
    )
    assert settings["per_joint_accel_deg_s2"] == stored.get(
        "per_joint_accel_deg_s2",
        [joint.max_accel_deg_s2 for joint in main.config.joints],
    )


def test_tool_command_rejects_action_for_wrong_active_tool(monkeypatch):
    original_config = main.config
    raw = {
        **original_config.raw,
        "tools": {
            **original_config.raw.get("tools", {}),
            "active": "magnet",
        },
    }
    try:
        monkeypatch.setattr(main, "config", replace(original_config, raw=raw))
        reset_runtime_state()
        client = TestClient(main.app)

        response = client.post("/api/tool", json={"action": "open", "tool": "magnet"})

        payload = response.json()
        assert not payload["ok"]
        assert "does not support" in payload["error"]
    finally:
        monkeypatch.setattr(main, "config", original_config)
