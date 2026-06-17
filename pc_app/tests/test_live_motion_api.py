from dataclasses import replace

from fastapi.testclient import TestClient

import app.main as main
from app.robot_state import MotionState


def reset_runtime_state() -> None:
    main.cancel_motion_tasks()
    main.state.connected = True
    main.state.simulation = True
    main.state.hardware_armed = False
    main.state.live_motion_enabled = False
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = main.config.home_pose.copy()
    main.state.target_angles_deg = main.config.home_pose.copy()
    main.state.clear_error()


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


def test_default_path_settings_follow_saved_joint_limits():
    reset_runtime_state()

    settings = main.request_settings(None)

    assert settings["global_speed_deg_s"] == min(joint.max_speed_deg_s for joint in main.config.joints)
    assert settings["global_accel_deg_s2"] == main.config.motion.acceleration_deg_s2
    assert settings["waypoint_rate_hz"] == main.config.motion.command_rate_limit_hz
    assert settings["per_joint_speed_deg_s"] == [joint.max_speed_deg_s for joint in main.config.joints]
    assert settings["per_joint_accel_deg_s2"] == [joint.max_accel_deg_s2 for joint in main.config.joints]


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
