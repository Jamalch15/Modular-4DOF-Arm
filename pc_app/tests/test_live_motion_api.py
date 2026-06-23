import asyncio
from dataclasses import replace
from time import sleep

from pytest import approx, fixture
from fastapi.testclient import TestClient

import app.main as main
from app.cartesian_jog_debug import cartesian_path_metrics
from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.kinematics import forward_kinematics
from app.motion import RateLimitedMotion
from app.robot_state import MotionState


@fixture(autouse=True)
def use_committed_example_config(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "live-motion-example-config")
    monkeypatch.setattr(
        main,
        "limiter",
        RateLimitedMotion(config, config.home_pose.copy(), config.home_pose.copy()),
    )
    yield
    main.cancel_motion_tasks()


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
    start_x = main.state.fk.get("x_mm", 0.0)

    with TestClient(main.app) as client:
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
        client.post("/api/cartesian-jog/stop")
    reset_runtime_state()


def test_cartesian_jog_simulation_endpoint_tracks_positive_z_with_bounded_lateral_error():
    reset_runtime_state()
    start = [0.0, 25.0, 80.0, -50.0]
    main.state.reported_angles_deg = start.copy()
    main.state.target_angles_deg = start.copy()
    main.state.fk = forward_kinematics(start, main.config.links)
    main.limiter.reset(start)
    with TestClient(main.app) as client:
        for _ in range(12):
            response = client.post(
                "/api/cartesian-jog",
                json={
                    "vx_mm_s": 0.0,
                    "vy_mm_s": 0.0,
                    "vz_mm_s": 40.0,
                    "vphi_deg_s": 0.0,
                    "tcp_speed_mm_s": 60.0,
                },
            )
            payload = response.json()
            assert payload["ok"], payload
            sleep(0.08)

        path = main.state.motion_diagnostics.get("actual_tcp_path", [])
        points = [[sample["x_mm"], sample["y_mm"], sample["z_mm"]] for sample in path]
        metrics = cartesian_path_metrics(points, [0.0, 0.0, 1.0])

        assert metrics["progress_mm"] > 13.0
        assert metrics["alignment"] > 0.7
        assert metrics["max_lateral_mm"] < 15.0
        assert metrics["backward_steps"] == 0
        client.post("/api/cartesian-jog/stop")
    reset_runtime_state()


def test_cartesian_jog_replaces_stale_blocked_goal_with_reverse_command():
    reset_runtime_state()
    start = [0.0, 45.0, 25.0, -20.0]
    main.state.reported_angles_deg = start.copy()
    main.state.target_angles_deg = start.copy()
    main.state.fk = forward_kinematics(start, main.config.links)
    main.limiter.reset(start)
    with TestClient(main.app) as client:
        blocked_payload = None
        for _ in range(30):
            payload = client.post(
                "/api/cartesian-jog",
                json={"vz_mm_s": 40.0},
            ).json()
            assert payload["ok"], payload
            sleep(0.08)
            latest = main.cartesian_jog_runtime.get("last_result")
            if latest and latest.get("blocked"):
                blocked_payload = latest
                break

        assert blocked_payload is not None
        assert blocked_payload["failure_code"] == "direction_unavailable"
        assert blocked_payload["achieved_delta"] == {
            "x_mm": 0.0,
            "y_mm": 0.0,
            "z_mm": 0.0,
            "phi_deg": 0.0,
        }
        reverse = client.post(
            "/api/cartesian-jog",
            json={"vz_mm_s": -20.0},
        ).json()
        assert reverse["ok"], reverse
        reverse_result = None
        for _ in range(20):
            refresh = client.post(
                "/api/cartesian-jog",
                json={"vz_mm_s": -20.0},
            ).json()
            assert refresh["ok"], refresh
            sleep(0.08)
            latest = main.cartesian_jog_runtime.get("last_result")
            if (
                latest
                and latest["target_task_velocity"][2] < 0.0
            ):
                reverse_result = latest
                break
        assert reverse_result, reverse_result
        assert main.cartesian_jog_runtime["command_velocity"][2] == -20.0
        assert main.cartesian_servo.target_task_velocity[2] == -20.0
        assert reverse_result["target_task_velocity"][2] == -20.0
        client.post("/api/cartesian-jog/stop")
    reset_runtime_state()


def test_cartesian_jog_hardware_uses_servoj_protocol(monkeypatch):
    reset_runtime_state()
    main.state.simulation = False
    main.state.hardware_armed = True
    main.state.live_motion_enabled = True
    main.state.config_sync_status = "synced"
    fake = FakeSerial(
        ["OK command=SERVOJ hw=mixed"] * 20
        + ["OK command=JOG_STOP"]
    )
    monkeypatch.setattr(main, "serial_client", fake)
    monkeypatch.setattr(main, "hardware_ready_for_motion", lambda: (True, ""))
    monkeypatch.setattr(main, "refresh_serial_status", lambda: None)
    with TestClient(main.app) as client:
        response = client.post(
            "/api/cartesian-jog",
            json={"vx_mm_s": 0.0, "vy_mm_s": 20.0, "vz_mm_s": 0.0, "vphi_deg_s": 0.0},
        )

        payload = response.json()
        assert payload["ok"], payload
        assert fake.sent[0].startswith("SERVOJ")
        assert not any(line.startswith("JOGV") for line in fake.sent)
        assert not any(line.startswith("MOVEJ") for line in fake.sent)
        client.post("/api/cartesian-jog/stop")
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
    assert trajectory["motion_contract"]["limits"]["effective_joint_speed_deg_s"][2] == 8.0
    assert payload["preview"]["motion_contract"]["controller_command"]["command"] == "SIM_TRAJ"
    reset_runtime_state()


def test_direct_joint_apply_simulation_executes_timed_trajectory():
    reset_runtime_state()
    target = main.config.home_pose.copy()
    target[0] += 0.5
    settings = {
        "global_speed_deg_s": 120.0,
        "global_accel_deg_s2": 600.0,
        "waypoint_rate_hz": 30.0,
        "per_joint_speed_deg_s": [2.0, 120.0, 120.0, 120.0],
        "per_joint_accel_deg_s2": [600.0, 600.0, 600.0, 600.0],
    }

    async def scenario():
        response = await main.start_joint_target_trajectory(target, "test_joint", settings)
        assert response["ok"], response
        await main.path_task
        return response

    response = asyncio.run(scenario())
    diagnostics = main.state.motion_diagnostics

    assert response["preview"]["trajectory"]["duration_s"] > 0.0
    assert diagnostics["result"] == "reached"
    assert diagnostics["expected_duration_s"] == response["preview"]["trajectory"]["duration_s"]
    assert diagnostics["motion_contract"]["controller_command"]["command"] == "SIM_TRAJ"
    assert diagnostics["motion_contract"]["controller_command"]["uses_planned_timestamps"] is True
    assert main.state.reported_angles_deg == approx(target, abs=0.08)
    reset_runtime_state()


def test_direct_joint_apply_and_program_preview_report_consistent_limits():
    reset_runtime_state()
    client = TestClient(main.app)
    target = main.config.home_pose.copy()
    target[0] += 6.0
    settings = {
        "global_speed_deg_s": 18.0,
        "global_accel_deg_s2": 90.0,
        "waypoint_rate_hz": 12.0,
        "per_joint_speed_deg_s": [12.0, 18.0, 18.0, 18.0],
        "per_joint_accel_deg_s2": [30.0, 90.0, 90.0, 90.0],
    }

    direct = client.post("/api/joints", json={"angles_deg": target, "settings": settings}).json()
    assert direct["ok"], direct
    direct_limits = direct["preview"]["motion_contract"]["limits"]
    reset_runtime_state()

    program = client.post(
        "/api/path/preview",
        json={
            "mode": "program",
            "settings": settings,
            "waypoints": [{"type": "joint", "mode": "joint", "angles_deg": target}],
        },
    ).json()

    assert program["ok"], program
    program_limits = program["preview"]["motion_contract"]["limits"]["segment_limits"][0]
    assert program["preview"]["motion_contract"]["path_mode"] == "program"
    assert program_limits["effective_joint_speed_deg_s"] == direct_limits["effective_joint_speed_deg_s"]
    assert program_limits["effective_joint_accel_deg_s2"] == direct_limits["effective_joint_accel_deg_s2"]
    assert program_limits["limiting_constraint"] == direct_limits["limiting_constraint"]
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
    assert settings["tcp_speed_mm_s"] == stored.get("tcp_speed_mm_s", 60.0)
    assert settings["phi_speed_deg_s"] == stored.get("phi_speed_deg_s", 45.0)
    assert settings["tcp_accel_mm_s2"] == stored.get("tcp_accel_mm_s2", 360.0)
    assert settings["phi_accel_deg_s2"] == stored.get("phi_accel_deg_s2", 240.0)
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


def test_cartesian_servo_limits_use_requested_joint_speed_and_acceleration():
    settings = main.request_settings(
        {
            "global_speed_deg_s": 20.0,
            "global_accel_deg_s2": 30.0,
            "per_joint_speed_deg_s": [18.0, 17.0, 16.0, 15.0],
            "per_joint_accel_deg_s2": [14.0, 13.0, 12.0, 11.0],
        }
    )

    limits = main._cartesian_servo_limits(settings)

    assert limits.joint_speed_deg_s == [18.0, 17.0, 16.0, 15.0]
    assert limits.joint_accel_deg_s2 == [12.0, 12.0, 12.0, 11.0]


def test_cartesian_jog_motion_contract_reports_velocity_limits():
    settings = main.request_settings(
        {
            "tcp_speed_mm_s": 44.0,
            "phi_speed_deg_s": 33.0,
            "tcp_accel_mm_s2": 222.0,
            "phi_accel_deg_s2": 111.0,
            "global_speed_deg_s": 20.0,
            "global_accel_deg_s2": 30.0,
        }
    )

    contract = main.cartesian_jog_motion_contract(settings)
    limits = contract["limits"]

    assert contract["path_mode"] == "cartesian_jog"
    assert limits["tcp_speed_mm_s"] == 44.0
    assert limits["phi_speed_deg_s"] == 33.0
    assert limits["tcp_accel_mm_s2"] == 222.0
    assert limits["phi_accel_deg_s2"] == 111.0
    assert limits["effective_joint_speed_deg_s"] == [20.0, 20.0, 20.0, 20.0]


def test_browser_live_cartesian_jog_uses_continuous_servo_loop():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")
    payload_body = app_js.split("function cartesianJogPayload()", 1)[1].split(
        "function scheduleCartesianJog",
        1,
    )[0]

    assert "dt_s:" not in payload_body


def test_motion_settings_labels_are_honest_about_supported_controls():
    index_html = (main.STATIC_DIR / "index.html").read_text(encoding="utf-8")
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")

    assert "Waypoint blend" not in index_html
    assert ">Smoothing<" not in index_html
    assert "Trapezoid ramp" in index_html
    assert "Cart jog TCP acceleration" in index_html
    assert "function motionContractHtml" in app_js
    assert "function syncPlannerControls" in app_js
    assert "elements.blendPercentInput.disabled = !trapezoidSelected" in app_js
    assert "motionContractHtml(preview, trajectory)" in app_js
    assert "motionContractHtml(state.programPreview, trajectory)" in app_js
    assert "motionContractHtml(diagnostics" in app_js
    assert "Controller" in app_js
    assert "controller_command" in app_js


def test_armed_toggle_allows_disarming_when_controller_sync_is_blocked():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")
    disabled_state = app_js.split("function updateDisabledState()", 1)[1].split(
        "elements.executeIkBtn.disabled",
        1,
    )[0]

    assert "!state.robotState?.hardware_armed &&" in disabled_state
    assert 'state.robotState?.config_sync_status !== "synced"' in disabled_state
    assert 'state.robotState?.hardware_armed\n      ? "Disarm hardware"' in disabled_state


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


def test_tool_hardware_ack_does_not_require_redundant_status_round_trip(monkeypatch):
    reset_runtime_state()
    try:
        main.state.simulation = False
        main.state.hardware_armed = True
        main.state.tool_state = "unknown"
        fake = FakeSerial(["OK command=TOOL state=open value=0.000"])
        monkeypatch.setattr(main, "serial_client", fake)
        client = TestClient(main.app)

        payload = client.post("/api/tool", json={"action": "open", "tool": "gripper"}).json()

        assert payload["ok"] is True
        assert fake.sent == ["TOOL OPEN"]
        assert payload["state"]["tool_state"] == "open"
        assert payload["state"]["tool_value"] == 0.0
        assert payload["state"]["motion_state"] == "idle"
        assert payload["state"]["last_controller_response"].startswith("OK command=TOOL")
    finally:
        main.state.simulation = True
        main.state.hardware_armed = False
