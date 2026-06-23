from copy import deepcopy
import asyncio

import pytest
from fastapi.testclient import TestClient

import app.main as main
from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.motion import RateLimitedMotion
from app.robot_state import MotionState


class FakeSerial:
    def __init__(self, responses):
        self.responses = list(responses)
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


def encoder_status(raw_deg: float, raw_count: int = 1000) -> str:
    return (
        "STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=setpose "
        "armed=0 hw=mixed enabled=1100 enc=0100 enc_valid=0100 "
        f"er2={raw_count} ea2={raw_deg} eage2=10 enoise2=0.02 ef2=OK "
        "j1=0 j2=20 j3=20 j4=0 closed_loop=diagnostic correction=idle fault=OK"
    )


@pytest.fixture(autouse=True)
def example_encoder_runtime(monkeypatch):
    state_snapshot = deepcopy(main.state.__dict__)
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "cs_pin": 15,
            "calibration_validated": False,
            "calibration_id": "",
        }
    )
    config = type(config)(**{**config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", config)
    monkeypatch.setattr(
        main,
        "limiter",
        RateLimitedMotion(config, config.home_pose.copy(), config.home_pose.copy()),
    )
    main.encoder_calibration_sessions.clear()
    main.encoder_calibration_sweep_task = None
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = False
    main.state.motion_state = MotionState.IDLE
    main.state.update_reported_pose(
        config.home_pose,
        source="setpose",
        known_pose=True,
        force_revision=True,
    )
    main.state.encoder_fault = False
    main.state.clear_error()
    yield
    main.encoder_calibration_sessions.clear()
    main.encoder_calibration_sweep_task = None
    main.cancel_motion_tasks()
    main.state.__dict__.clear()
    main.state.__dict__.update(state_snapshot)


def test_guided_calibration_captures_validates_and_commits(monkeypatch):
    fake = FakeSerial(
        [
            encoder_status(10.0, 455),
            encoder_status(20.0, 910),
            encoder_status(40.0, 1820),
        ]
    )
    monkeypatch.setattr(main, "serial_client", fake)
    persisted = {}

    def capture_persist(settings):
        persisted.update(deepcopy(settings))
        return {"categories": ["encoder_calibration"], "sync_required": True}

    monkeypatch.setattr(main, "persist_encoder_calibration", capture_persist)
    monkeypatch.setattr(main, "public_config", lambda: {"encoders": persisted})
    client = TestClient(main.app)

    start = client.post(
        "/api/encoder/calibration/start",
        json={"mounting_location": "joint_output", "reference_description": "fixture A"},
    ).json()
    session_id = start["session"]["id"]
    first = client.post(
        "/api/encoder/calibration/sample",
        json={"session_id": session_id, "joint_angle_deg": 10.0, "label": "low"},
    ).json()
    second = client.post(
        "/api/encoder/calibration/sample",
        json={"session_id": session_id, "joint_angle_deg": 30.0, "label": "high"},
    ).json()
    committed = client.post(
        "/api/encoder/calibration/commit",
        json={"session_id": session_id, "confirm": True},
    ).json()

    assert start["ok"]
    assert first["ok"]
    assert second["validation"]["ok"]
    assert second["validation"]["direction_sign"] == 1
    assert second["validation"]["sensor_turns_per_joint_turn"] == pytest.approx(1.0)
    assert committed["ok"]
    shoulder = persisted["axes"][0]
    assert shoulder["reference_raw_deg"] == pytest.approx(20.0)
    assert shoulder["reference_joint_deg"] == pytest.approx(10.0)
    assert shoulder["calibration_validated"] is True
    assert persisted["correction"]["enabled"] is False


def test_guided_calibration_start_can_capture_initial_reference(monkeypatch):
    fake = FakeSerial(
        [
            encoder_status(120.0, 5461),
            encoder_status(140.0, 6372),
        ]
    )
    monkeypatch.setattr(main, "serial_client", fake)
    client = TestClient(main.app)

    start = client.post(
        "/api/encoder/calibration/start",
        json={
            "mounting_location": "joint_output",
            "reference_description": "90 degree fixture mark",
            "joint_angle_deg": 90.0,
            "capture_initial": True,
        },
    ).json()
    session_id = start["session"]["id"]
    second = client.post(
        "/api/encoder/calibration/sample",
        json={"session_id": session_id, "joint_angle_deg": 110.0, "label": "high"},
    ).json()

    assert start["ok"]
    assert start["sample"]["raw_angle_deg"] == pytest.approx(120.0)
    assert start["sample"]["joint_angle_deg"] == pytest.approx(90.0)
    assert start["validation"]["sample_count"] == 1
    assert second["validation"]["ok"]
    assert second["validation"]["raw_span_deg"] == pytest.approx(20.0)
    assert second["validation"]["joint_span_deg"] == pytest.approx(20.0)


def test_quick_calibration_commits_single_point_offset(monkeypatch):
    fake = FakeSerial([encoder_status(120.0, 5461)])
    monkeypatch.setattr(main, "serial_client", fake)
    persisted = {}

    def capture_persist(settings):
        persisted.update(deepcopy(settings))
        return {"categories": ["encoder_calibration"], "sync_required": True}

    monkeypatch.setattr(main, "persist_encoder_calibration", capture_persist)
    monkeypatch.setattr(main, "public_config", lambda: {"encoders": persisted})
    client = TestClient(main.app)

    response = client.post(
        "/api/encoder/calibration/quick",
        json={
            "joint_angle_deg": 90.0,
            "direction_sign": -1,
            "reference_description": "90 degree fixture mark",
            "confirm_one_to_one_output_mount": True,
        },
    ).json()

    assert response["ok"]
    assert response["validation"]["fit_model"] == "single_point"
    shoulder = persisted["axes"][0]
    assert shoulder["reference_raw_deg"] == pytest.approx(120.0)
    assert shoulder["reference_joint_deg"] == pytest.approx(90.0)
    assert shoulder["direction_sign"] == -1
    assert shoulder["sensor_turns_per_joint_turn"] == pytest.approx(1.0)
    assert shoulder["calibration_model"] == "single_point"
    assert shoulder["calibration_validated"] is True
    assert persisted["correction"]["enabled"] is False


def test_assisted_sweep_allows_stationary_stopped_state(monkeypatch):
    monkeypatch.setattr(main, "serial_client", FakeSerial([]))
    main.state.controller_capabilities = {"protocol": 4, "encoder_config": True}
    main.state.config_sync_status = "synced"
    main.state.hardware_armed = True
    main.state.motion_state = MotionState.STOPPED
    main.state.live_motion_enabled = False
    main.state.task_execution = {}

    ready, reason = main.encoder_calibration_sweep_ready()

    assert ready is True
    assert reason == ""


def test_assisted_sweep_rejects_active_motion_with_specific_message(monkeypatch):
    monkeypatch.setattr(main, "serial_client", FakeSerial([]))
    main.state.controller_capabilities = {"protocol": 4, "encoder_config": True}
    main.state.config_sync_status = "synced"
    main.state.hardware_armed = True
    main.state.motion_state = MotionState.MOVING

    ready, reason = main.encoder_calibration_sweep_ready()

    assert ready is False
    assert reason == "wait for the current motion to finish before assisted encoder sweep"


def test_calibration_fit_handles_raw_wraparound():
    session = {
        "mounting_location": "joint_output",
        "samples": [
            {"raw_angle_deg": 350.0, "joint_angle_deg": 0.0},
            {"raw_angle_deg": 10.0, "joint_angle_deg": 20.0},
        ],
    }

    validation = main.validate_encoder_calibration_session(session)

    assert validation["ok"]
    assert validation["direction_sign"] == 1
    assert validation["sensor_turns_per_joint_turn"] == pytest.approx(1.0)


def test_calibration_fit_handles_directional_backlash():
    session = {
        "mounting_location": "joint_output",
        "samples": [
            {"raw_angle_deg": 93.0, "joint_angle_deg": 90.0, "approach_direction": -1},
            {"raw_angle_deg": 63.0, "joint_angle_deg": 60.0, "approach_direction": -1},
            {"raw_angle_deg": 33.0, "joint_angle_deg": 30.0, "approach_direction": -1},
            {"raw_angle_deg": 57.0, "joint_angle_deg": 60.0, "approach_direction": 1},
            {"raw_angle_deg": 87.0, "joint_angle_deg": 90.0, "approach_direction": 1},
        ],
    }

    validation = main.validate_encoder_calibration_session(session)

    assert validation["ok"]
    assert validation["fit_model"] == "linear_with_backlash"
    assert validation["direction_sign"] == 1
    assert validation["sensor_turns_per_joint_turn"] == pytest.approx(1.0)
    assert validation["backlash_estimate_deg"] == pytest.approx(6.0)
    assert validation["approach_bias_deg"] == pytest.approx(3.0)
    assert validation["reference_raw_deg"] == pytest.approx(93.0)
    assert validation["reference_joint_deg"] == pytest.approx(93.0)
    assert validation["max_residual_deg"] == pytest.approx(0.0)
    assert validation["linear_max_residual_deg"] > validation["residual_limit_deg"]


def test_assisted_sweep_captures_settled_samples(monkeypatch):
    raw_samples = iter(
        [
            {"raw_count": 2731, "raw_angle_deg": 60.0, "age_ms": 10, "noise_deg": 0.02, "captured_at": 1.0},
            {"raw_count": 1365, "raw_angle_deg": 30.0, "age_ms": 10, "noise_deg": 0.02, "captured_at": 2.0},
            {"raw_count": 2731, "raw_angle_deg": 60.0, "age_ms": 10, "noise_deg": 0.02, "captured_at": 3.0},
            {"raw_count": 4096, "raw_angle_deg": 90.0, "age_ms": 10, "noise_deg": 0.02, "captured_at": 4.0},
        ]
    )

    def current_sample():
        return deepcopy(next(raw_samples)), ""

    async def fake_start_joint_target_trajectory(targets, command_label, settings):
        assert command_label == "encoder_calibration_sweep"
        assert settings["per_joint_speed_deg_s"][1] <= 8.0
        main.state.update_reported_pose(
            [float(value) for value in targets],
            source="open_loop_estimate",
            known_pose=True,
            force_revision=True,
        )
        main.state.motion_state = MotionState.IDLE
        main.state.motion_diagnostics = {"result": "reached"}
        return {"ok": True, "state": main.state.to_dict()}

    async def no_sleep(_seconds):
        return None

    monkeypatch.setattr(main, "current_raw_shoulder_sample", current_sample)
    monkeypatch.setattr(main, "start_joint_target_trajectory", fake_start_joint_target_trajectory)
    monkeypatch.setattr(main.asyncio, "sleep", no_sleep)
    initial = main._encoder_calibration_capture(
        current_sample()[0],
        60.0,
        "start check",
        use_for_fit=False,
    )
    session = {
        "id": "sweep-session",
        "joint": 2,
        "created_at": 1.0,
        "mounting_location": "joint_output",
        "reference_description": "fixture",
        "samples": [initial],
        "sweep": {
            "targets_deg": main._encoder_unidirectional_sweep_targets(
                sweep_min_deg=30.0,
                sweep_max_deg=90.0,
                step_deg=30.0,
                final_approach_direction=1,
            ),
            "final_approach_direction": 1,
            "preload_deg": 5.0,
            "preload_target_deg": main._encoder_sweep_preload_target(
                30.0,
                final_approach_direction=1,
                preload_deg=5.0,
            ),
            "path_settings": main._encoder_sweep_path_settings(6.0, 24.0),
            "settle_ms": 100,
        },
    }
    main.encoder_calibration_sessions["sweep-session"] = session

    asyncio.run(main.run_encoder_calibration_sweep("sweep-session"))

    assert session["sweep"]["status"] == "completed"
    assert session["sweep"]["completed"] == 3
    assert len(session["samples"]) == 4
    assert [sample["joint_angle_deg"] for sample in session["samples"]] == [60.0, 30.0, 60.0, 90.0]
    assert session["samples"][0]["use_for_fit"] is False
    assert [sample.get("approach_direction") for sample in session["samples"][1:]] == [1, 1, 1]
    assert session["validation"]["sample_count"] == 4
    assert session["validation"]["fit_sample_count"] == 3
    assert session["validation"]["ok"]


def test_backlash_check_measures_same_target_from_both_directions(monkeypatch):
    raw = deepcopy(main.config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "calibration_validated": True,
            "calibration_id": "quick-zero",
            "mounting_location": "joint_output",
        }
    )
    config = type(main.config)(**{**main.config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", config)
    monkeypatch.setattr(main, "serial_client", FakeSerial([]))
    main.state.controller_capabilities = {"protocol": 4, "encoder_config": True}
    main.state.config_sync_status = "synced"
    main.state.hardware_armed = True
    main.state.motion_state = MotionState.IDLE
    main.state.update_reported_pose(
        [0.0, 60.0, 20.0, 0.0],
        source="setpose",
        known_pose=True,
        force_revision=True,
    )
    measurements = iter([58.0, 63.5])
    moves = []

    async def fake_start_joint_target_trajectory(targets, command_label, settings):
        assert command_label == "encoder_backlash_check"
        moves.append(float(targets[1]))
        main.state.update_reported_pose(
            [float(value) for value in targets],
            source="open_loop_estimate",
            known_pose=True,
            force_revision=True,
        )
        main.state.motion_state = MotionState.IDLE
        main.state.motion_diagnostics = {"result": "reached"}
        return {"ok": True, "state": main.state.to_dict()}

    async def stable(_required_samples):
        return next(measurements), "stable"

    def evidence():
        return {"raw_angle_deg": 120.0, "raw_count": 5461, "age_ms": 10, "noise_deg": 0.02}

    async def no_sleep(_seconds):
        return None

    monkeypatch.setattr(main, "start_joint_target_trajectory", fake_start_joint_target_trajectory)
    monkeypatch.setattr(main, "_stable_shoulder_measurement", stable)
    monkeypatch.setattr(main, "_shoulder_evidence", evidence)
    monkeypatch.setattr(main.asyncio, "sleep", no_sleep)

    result = asyncio.run(
        main.run_encoder_backlash_check(
            main.EncoderBacklashCheckRequest(
                center_joint_angle_deg=60.0,
                travel_deg=10.0,
                repeats=1,
                speed_deg_s=6.0,
                settle_ms=100,
            )
        )
    )

    assert result["ok"]
    assert moves == [50.0, 60.0, 70.0, 60.0]
    assert result["backlash"]["backlash_estimate_deg"] == pytest.approx(5.5)
    assert result["backlash"]["midpoint_error_deg"] == pytest.approx(0.75)
    assert main.state.encoder_mismatch["status"] == "backlash_measured"


def test_unidirectional_sweep_rejects_preload_outside_limits():
    shoulder_min = main.config.joints[1].min_deg
    with pytest.raises(ValueError, match="too close to the shoulder limit"):
        main._encoder_sweep_preload_target(
            shoulder_min,
            final_approach_direction=1,
            preload_deg=8.0,
        )


def test_validation_reports_localized_backlash_when_directional_fit_fails():
    session = {
        "mounting_location": "joint_output",
        "samples": [
            {"raw_angle_deg": 35.002, "joint_angle_deg": 180.0, "approach_direction": -1},
            {"raw_angle_deg": 49.768, "joint_angle_deg": 165.0, "approach_direction": -1},
            {"raw_angle_deg": 64.731, "joint_angle_deg": 150.0, "approach_direction": -1},
            {"raw_angle_deg": 80.002, "joint_angle_deg": 135.0, "approach_direction": -1},
            {"raw_angle_deg": 95.537, "joint_angle_deg": 120.0, "approach_direction": -1},
            {"raw_angle_deg": 110.830, "joint_angle_deg": 105.0, "approach_direction": -1},
            {"raw_angle_deg": 126.035, "joint_angle_deg": 90.0, "approach_direction": -1},
            {"raw_angle_deg": 144.250, "joint_angle_deg": 75.0, "approach_direction": -1},
            {"raw_angle_deg": 161.082, "joint_angle_deg": 60.0, "approach_direction": -1},
            {"raw_angle_deg": 176.462, "joint_angle_deg": 45.0, "approach_direction": -1},
            {"raw_angle_deg": 191.931, "joint_angle_deg": 30.0, "approach_direction": -1},
            {"raw_angle_deg": 207.664, "joint_angle_deg": 15.0, "approach_direction": -1},
            {"raw_angle_deg": 223.000, "joint_angle_deg": 0.0, "approach_direction": -1},
            {"raw_angle_deg": 207.686, "joint_angle_deg": 15.0, "approach_direction": 1},
            {"raw_angle_deg": 191.975, "joint_angle_deg": 30.0, "approach_direction": 1},
            {"raw_angle_deg": 176.616, "joint_angle_deg": 45.0, "approach_direction": 1},
            {"raw_angle_deg": 161.609, "joint_angle_deg": 60.0, "approach_direction": 1},
            {"raw_angle_deg": 146.843, "joint_angle_deg": 75.0, "approach_direction": 1},
            {"raw_angle_deg": 131.836, "joint_angle_deg": 90.0, "approach_direction": 1},
            {"raw_angle_deg": 113.225, "joint_angle_deg": 105.0, "approach_direction": 1},
            {"raw_angle_deg": 95.669, "joint_angle_deg": 120.0, "approach_direction": 1},
            {"raw_angle_deg": 80.310, "joint_angle_deg": 135.0, "approach_direction": 1},
            {"raw_angle_deg": 65.039, "joint_angle_deg": 150.0, "approach_direction": 1},
            {"raw_angle_deg": 49.966, "joint_angle_deg": 165.0, "approach_direction": 1},
            {"raw_angle_deg": 34.937, "joint_angle_deg": 180.0, "approach_direction": 1},
        ],
    }

    validation = main.validate_encoder_calibration_session(session)

    assert not validation["ok"]
    assert validation["localized_backlash_pair_count"] >= 1
    assert validation["localized_backlash_at_joint_deg"] == pytest.approx(90.0)
    assert validation["localized_backlash_estimate_raw_deg"] == pytest.approx(5.801)
    assert "localized bidirectional backlash" in "; ".join(validation["errors"])


def test_same_direction_nonlinear_samples_accept_piecewise_calibration():
    samples = []
    for joint_angle in [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0]:
        raw_angle = 220.0 - joint_angle - 0.004 * joint_angle * joint_angle
        samples.append(
            {
                "raw_angle_deg": raw_angle,
                "joint_angle_deg": joint_angle,
                "approach_direction": 1,
            }
        )
    session = {
        "mounting_location": "joint_output",
        "samples": samples,
    }

    validation = main.validate_encoder_calibration_session(session)

    assert validation["ok"]
    assert validation["fit_model"] == "piecewise_linear"
    assert validation["calibration_map_point_count"] == 8
    assert validation["reference_joint_deg"] == pytest.approx(45.0)
    assert validation["max_residual_deg"] == pytest.approx(0.0)
    assert validation["linear_max_residual_deg"] > validation["residual_limit_deg"]
    assert any("piecewise calibration" in warning for warning in validation["warnings"])


def test_correction_policy_enable_creates_validation_record(monkeypatch):
    raw = deepcopy(main.config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "calibration_validated": True,
            "calibration_id": "fixture-map",
            "mounting_location": "joint_output",
        }
    )
    raw["encoders"]["verification"]["policy"] = "diagnostic"
    config = type(main.config)(**{**main.config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", config)
    monkeypatch.setattr(main, "serial_client", FakeSerial([]))
    main.state.config_sync_status = "synced"
    main.state.controller_capabilities = {"protocol": 4, "encoder_config": True}
    main.state.hardware_armed = False
    main.state.motion_state = MotionState.IDLE
    main.state.update_reported_pose(
        [0.0, 20.0, 20.0, 0.0],
        source="setpose",
        known_pose=True,
        force_revision=True,
    )
    persisted = {}

    async def stable(_required_samples):
        return 20.05, "stable"

    def capture_persist(settings):
        persisted.update(deepcopy(settings))
        return {"categories": ["encoder_correction"], "sync_required": True}

    monkeypatch.setattr(main, "_stable_shoulder_measurement", stable)
    monkeypatch.setattr(main, "persist_encoder_calibration", capture_persist)
    monkeypatch.setattr(main, "public_config", lambda: {"encoders": persisted})
    client = TestClient(main.app)

    response = client.post(
        "/api/encoder/correction/policy",
        json={"enabled": True, "confirm": True},
    ).json()

    assert response["ok"]
    assert persisted["mode"] == "bounded_correction"
    assert persisted["verification"]["policy"] == "warning"
    assert persisted["correction"]["enabled"] is True
    assert persisted["correction"]["validation_id"].startswith("shoulder-correction-")


def test_correction_policy_enable_allows_correctable_backlash_mismatch(monkeypatch):
    raw = deepcopy(main.config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "calibration_validated": True,
            "calibration_id": "fixture-map",
            "mounting_location": "joint_output",
        }
    )
    raw["encoders"]["verification"]["policy"] = "diagnostic"
    raw["encoders"]["verification"]["warning_tolerance_deg"] = 2.0
    raw["encoders"]["correction"]["max_delta_deg"] = 8.0
    config = type(main.config)(**{**main.config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", config)
    monkeypatch.setattr(main, "serial_client", FakeSerial([]))
    main.state.config_sync_status = "synced"
    main.state.controller_capabilities = {"protocol": 4, "encoder_config": True}
    main.state.hardware_armed = False
    main.state.motion_state = MotionState.IDLE
    main.state.update_reported_pose(
        [0.0, 20.0, 20.0, 0.0],
        source="setpose",
        known_pose=True,
        force_revision=True,
    )
    persisted = {}

    async def stable(_required_samples):
        return 25.5, "stable"

    def capture_persist(settings):
        persisted.update(deepcopy(settings))
        return {"categories": ["encoder_correction"], "sync_required": True}

    monkeypatch.setattr(main, "_stable_shoulder_measurement", stable)
    monkeypatch.setattr(main, "persist_encoder_calibration", capture_persist)
    monkeypatch.setattr(main, "public_config", lambda: {"encoders": persisted})
    client = TestClient(main.app)

    response = client.post(
        "/api/encoder/correction/policy",
        json={"enabled": True, "confirm": True},
    ).json()

    assert response["ok"]
    assert response["details"]["initial_mismatch_correctable"] is True
    assert response["details"]["error_deg"] == pytest.approx(5.5)
    assert persisted["correction"]["enabled"] is True


def test_encoder_fault_clear_does_not_restore_pose_knowledge(monkeypatch):
    monkeypatch.setattr(main, "serial_client", FakeSerial([]))
    main.state.encoder_fault = True
    main.state.motion_state = MotionState.FAULT
    main.state.update_reported_pose(
        main.state.reported_angles_deg,
        source="open_loop_estimate",
        known_mask="1011",
        force_revision=True,
    )
    client = TestClient(main.app)

    rejected = client.post(
        "/api/encoder/fault/clear",
        json={"acknowledge_pose_unknown": False},
    ).json()
    cleared = client.post(
        "/api/encoder/fault/clear",
        json={"acknowledge_pose_unknown": True},
    ).json()

    assert rejected["ok"] is False
    assert cleared["ok"]
    assert cleared["requires_setpose"] is True
    assert cleared["state"]["known_pose"] is False
    assert cleared["state"]["pose_known_mask"] == "1011"
    assert cleared["state"]["motion_state"] == "stopped"


def test_post_move_fault_latches_without_rebasing_the_planning_pose(monkeypatch):
    raw = deepcopy(main.config.raw)
    raw["encoders"]["axes"][0].update(
        {
            "calibration_validated": True,
            "calibration_id": "fixture",
            "mounting_location": "joint_output",
        }
    )
    raw["encoders"]["verification"].update(
        {
            "policy": "fault",
            "warning_tolerance_deg": 2.0,
            "fault_tolerance_deg": 5.0,
        }
    )
    config = type(main.config)(**{**main.config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", config)
    fake = FakeSerial([])
    monkeypatch.setattr(main, "serial_client", fake)

    async def stable(_required_samples):
        return 30.0, "stable"

    async def no_sleep(_seconds):
        return None

    monkeypatch.setattr(main, "_stable_shoulder_measurement", stable)
    monkeypatch.setattr(main.asyncio, "sleep", no_sleep)
    main.state.hardware_armed = True
    main.state.motion_state = MotionState.IDLE
    main.state.update_reported_pose(
        [0.0, 20.0, 20.0, 0.0],
        source="open_loop_estimate",
        known_pose=True,
        force_revision=True,
    )

    ok, message = asyncio.run(
        main.verify_shoulder_after_motion(
            "set_all_joint_targets",
            [0.0, 20.0, 20.0, 0.0],
        )
    )

    assert ok is False
    assert "mismatch" in message
    assert main.state.reported_angles_deg[1] == pytest.approx(20.0)
    assert main.state.measured_angles_deg[1] is None
    assert main.state.encoder_fault is True
    assert main.state.pose_known_mask == "1011"
    assert fake.sent[-1] == "STOP"


def test_post_move_verification_uses_commanded_target_not_open_loop_estimate(monkeypatch):
    raw = deepcopy(main.config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "calibration_validated": True,
            "calibration_id": "fixture",
            "mounting_location": "joint_output",
        }
    )
    raw["encoders"]["verification"].update(
        {
            "policy": "fault",
            "warning_tolerance_deg": 2.0,
            "fault_tolerance_deg": 5.0,
        }
    )
    config = type(main.config)(**{**main.config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", config)

    async def stable(_required_samples):
        return 62.5, "stable"

    async def no_sleep(_seconds):
        return None

    monkeypatch.setattr(main, "_stable_shoulder_measurement", stable)
    monkeypatch.setattr(main.asyncio, "sleep", no_sleep)
    main.state.hardware_armed = True
    main.state.motion_state = MotionState.IDLE
    main.state.update_reported_pose(
        [0.0, 50.0, 20.0, 0.0],
        source="open_loop_estimate",
        known_pose=True,
        force_revision=True,
    )

    ok, message = asyncio.run(
        main.verify_shoulder_after_motion(
            "set_all_joint_targets",
            [0.0, 60.0, 20.0, 0.0],
            allow_correction=False,
        )
    )

    assert ok is True
    assert message == "warning"
    assert main.state.encoder_fault is False
    assert main.state.encoder_mismatch["status"] == "warning"
    assert main.state.encoder_mismatch["error_deg"] == pytest.approx(2.5)
    assert main.state.encoder_mismatch["commanded_error_deg"] == pytest.approx(2.5)
    assert main.state.encoder_mismatch["estimated_error_deg"] == pytest.approx(12.5)


def test_post_move_correction_reports_when_error_exceeds_max_delta(monkeypatch):
    raw = deepcopy(main.config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "calibration_validated": True,
            "calibration_id": "fixture",
            "mounting_location": "joint_output",
        }
    )
    raw["encoders"]["verification"].update(
        {
            "policy": "warning",
            "warning_tolerance_deg": 1.0,
            "fault_tolerance_deg": 5.0,
        }
    )
    raw["encoders"]["correction"].update(
        {
            "enabled": True,
            "validation_id": "validated",
            "deadband_deg": 0.75,
            "max_delta_deg": 1.0,
        }
    )
    config = type(main.config)(**{**main.config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", config)

    async def stable(_required_samples):
        return 91.7, "stable"

    async def no_sleep(_seconds):
        return None

    monkeypatch.setattr(main, "_stable_shoulder_measurement", stable)
    monkeypatch.setattr(main.asyncio, "sleep", no_sleep)
    main.state.hardware_armed = True
    main.state.motion_state = MotionState.IDLE
    main.state.hardware_axis_states = ["hardware"] * len(config.joints)
    main.state.update_reported_pose(
        [0.0, 90.0, 20.0, 0.0],
        source="open_loop_estimate",
        known_pose=True,
        force_revision=True,
    )

    ok, message = asyncio.run(
        main.verify_shoulder_after_motion(
            "set_all_joint_targets",
            [0.0, 90.0, 20.0, 0.0],
        )
    )

    assert ok is True
    assert message == "warning"
    assert main.state.encoder_mismatch["error_deg"] == pytest.approx(1.7)
    assert main.state.encoder_mismatch["correction_status"] == "skipped"
    assert "exceeds correction max delta 1.00 deg" in main.state.encoder_mismatch["correction_skip_reason"]


def test_post_move_correction_uses_deadband_not_warning_threshold(monkeypatch):
    raw = deepcopy(main.config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "calibration_validated": True,
            "calibration_id": "fixture",
            "mounting_location": "joint_output",
        }
    )
    raw["encoders"]["verification"].update(
        {
            "policy": "warning",
            "warning_tolerance_deg": 2.0,
            "fault_tolerance_deg": 5.0,
        }
    )
    raw["encoders"]["correction"].update(
        {
            "enabled": True,
            "validation_id": "validated",
            "deadband_deg": 0.75,
            "max_delta_deg": 8.0,
        }
    )
    config = type(main.config)(**{**main.config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", config)
    measurements = iter([91.7, 90.2])
    sent: list[str] = []

    async def stable(_required_samples):
        return next(measurements), "stable"

    async def no_sleep(_seconds):
        return None

    def send_correctj(command):
        sent.append(command)
        return "OK command=CORRECTJ joint=2 delta=-1.700000 steps=-100 attempt=1 id=test"

    monkeypatch.setattr(main, "_stable_shoulder_measurement", stable)
    monkeypatch.setattr(main.asyncio, "sleep", no_sleep)
    monkeypatch.setattr(main, "send_correctj_and_read_response", send_correctj)
    monkeypatch.setattr(main, "refresh_serial_status", lambda: None)
    main.state.hardware_armed = True
    main.state.motion_state = MotionState.IDLE
    main.state.hardware_axis_states = ["hardware"] * len(config.joints)
    main.state.update_reported_pose(
        [0.0, 90.0, 20.0, 0.0],
        source="open_loop_estimate",
        known_pose=True,
        force_revision=True,
    )

    ok, message = asyncio.run(
        main.verify_shoulder_after_motion(
            "set_all_joint_targets",
            [0.0, 90.0, 20.0, 0.0],
        )
    )

    assert ok is True
    assert message == "corrected"
    assert sent
    assert "CORRECTJ joint=2 delta=-1.700000" in sent[0]
    assert main.state.encoder_mismatch["status"] == "corrected"
    assert main.state.encoder_mismatch["error_deg"] == pytest.approx(0.2)


def test_motion_diagnostics_wait_for_encoder_verification(monkeypatch):
    raw = deepcopy(main.config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["axes"][0]["enabled"] = True
    config = type(main.config)(**{**main.config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = False
    main.state.motion_state = MotionState.IDLE
    main.state.update_reported_pose(
        config.home_pose,
        source="setpose",
        known_pose=True,
        force_revision=True,
    )
    main.state.target_angles_deg = config.home_pose.copy()
    run_id = main.start_motion_diagnostics(
        source="set_all_joint_targets",
        mode="joint_endpoint",
        target_deg=config.home_pose,
        expected_duration_s=0.1,
        waypoint_count=1,
    )

    main.maybe_finish_reached_motion()

    assert main.state.motion_diagnostics["run_id"] == run_id
    assert main.state.motion_diagnostics["result"] == "executing"
    assert main.state.motion_diagnostics["execution_state"] == "settling_verification"
