from dataclasses import replace
from copy import deepcopy

from fastapi.testclient import TestClient
from pytest import fixture

import app.main as main
from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.motion import RateLimitedMotion
from app.robot_state import MotionState


@fixture(autouse=True)
def use_committed_example_config(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "hardware-sync-example-config")
    monkeypatch.setattr(
        main,
        "limiter",
        RateLimitedMotion(config, config.home_pose.copy(), config.home_pose.copy()),
    )
    yield
    main.cancel_motion_tasks()


class FakeSerial:
    def __init__(self, responses):
        self.responses = list(responses)
        self.sent = []
        self.connection = object()

    @property
    def is_connected(self):
        return True

    def send_line(self, line):
        self.sent.append(line)

    def read_line(self):
        return self.responses.pop(0) if self.responses else ""

    def clear_input(self):
        pass

    def read_until_prefix(self, prefix, timeout_s=2.0):
        while self.responses:
            line = self.read_line()
            if line.startswith(prefix):
                return line
        raise RuntimeError(f"timed out waiting for {prefix}")


def reset_runtime_state() -> None:
    main.cancel_motion_tasks()
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = False
    main.state.encoder_fault = False
    main.state.encoder_mismatch = {}
    main.state.live_motion_enabled = False
    main.state.motion_state = MotionState.IDLE
    main.state.update_reported_pose(
        main.config.home_pose,
        source="setpose",
        known_pose=True,
        force_revision=True,
    )
    main.state.target_angles_deg = main.config.home_pose.copy()
    main.state.config_sync_status = "stale"
    main.state.clear_error()


def disable_joint_hardware(joint):
    stepper = replace(joint.hardware.stepper, enabled=False) if joint.hardware.stepper else None
    servo = replace(joint.hardware.servo, enabled=False) if joint.hardware.servo else None
    return replace(joint, hardware=replace(joint.hardware, stepper=stepper, servo=servo))


def config_with_only_first_axis_enabled(config):
    joints = [disable_joint_hardware(joint) for joint in config.joints]
    first_joint = replace(
        joints[0],
        hardware=replace(
            joints[0].hardware,
            stepper=replace(
                joints[0].hardware.stepper,
                enabled=True,
                step_pin=17,
                dir_pin=16,
            ),
        ),
    )
    return replace(config, joints=[first_joint, *joints[1:]])


def config_with_first_joint_mapping_change(config):
    first_joint = replace(
        config.joints[0],
        zero_offset_deg=config.joints[0].zero_offset_deg + 1.0,
    )
    return replace(config, joints=[first_joint, *config.joints[1:]])


def config_with_first_joint_io_change(config):
    first_joint = replace(
        config.joints[0],
        hardware=replace(
            config.joints[0].hardware,
            stepper=replace(
                config.joints[0].hardware.stepper,
                step_pin=config.joints[0].hardware.stepper.step_pin + 1,
            ),
        ),
    )
    return replace(config, joints=[first_joint, *config.joints[1:]])


def config_with_enabled_encoder(config):
    raw = deepcopy(config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["mode"] = "bounded_correction"
    raw["encoders"]["axes"][0].update(
        {
            "enabled": True,
            "cs_pin": 15,
            "reference_joint_deg": config.joints[1].home_deg,
            "calibration_validated": True,
            "calibration_id": "test-calibration-id",
            "mounting_location": "joint_output",
        }
    )
    raw["encoders"]["verification"].update({"policy": "warning"})
    raw["encoders"]["correction"].update(
        {
            "enabled": True,
            "validation_id": "test-validation-id",
            "allowed_sources": ["set_joint_target", "encoder_shoulder_align"],
        }
    )
    return type(config)(**{**config.__dict__, "raw": raw})


def encoder_status_line(*, closed_loop: str = "bounded_correction", enc: str = "0100") -> str:
    return (
        "STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=setpose "
        f"armed=0 hw=hardware enabled=1111 enc={enc} enc_valid={enc} "
        "er2=8192 ea2=180.0 em2=90.0 eage2=5 enoise2=0.01 evalidn2=3 ef2=OK "
        f"j1=0.0 j2=90.0 j3=0.0 j4=0.0 closed_loop={closed_loop} correction=idle "
        "correction_id=none correction_delta=0.000000 correction_steps=0 correction_attempts=0 "
        "cb1=0.0000 cb2=0.0000 cb3=0.0000 cb4=0.0000 fault=OK"
    )


def test_hardware_sync_reports_synced(monkeypatch):
    reset_runtime_state()
    fake = FakeSerial(["OK command=CONFIG axes=4 hw=simulated enabled=0000"])
    monkeypatch.setattr(main, "serial_client", fake)
    client = TestClient(main.app)

    response = client.post("/api/hardware/sync")

    payload = response.json()
    assert payload["ok"]
    assert payload["status"] == "synced"
    assert fake.sent[0] == "CONFIG BEGIN axes=4"
    assert fake.sent[-1] == "CONFIG END"


def test_enabled_encoder_sync_verifies_controller_runtime(monkeypatch):
    original_config = main.config
    try:
        monkeypatch.setattr(main, "config", config_with_enabled_encoder(original_config))
        reset_runtime_state()
        main.state.controller_capabilities = {"protocol": 4, "encoder_config": True}
        fake = FakeSerial(
            [
                "OK command=CONFIG axes=4 hw=hardware enabled=1111",
                encoder_status_line(closed_loop="bounded_correction"),
            ]
        )
        monkeypatch.setattr(main, "serial_client", fake)
        client = TestClient(main.app)

        payload = client.post("/api/hardware/sync").json()

        assert payload["ok"] is True
        assert payload["status"] == "synced"
        assert "STATUS" in fake.sent
        assert payload["state"]["closed_loop_mode"] == "bounded_correction"
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_enabled_encoder_sync_fails_if_controller_runtime_is_off(monkeypatch):
    original_config = main.config
    try:
        monkeypatch.setattr(main, "config", config_with_enabled_encoder(original_config))
        reset_runtime_state()
        main.state.controller_capabilities = {"protocol": 4, "encoder_config": True}
        fake = FakeSerial(
            [
                "OK command=CONFIG axes=4 hw=hardware enabled=1111",
                encoder_status_line(closed_loop="off", enc="0000"),
            ]
        )
        monkeypatch.setattr(main, "serial_client", fake)
        client = TestClient(main.app)

        payload = client.post("/api/hardware/sync").json()

        assert payload["ok"] is False
        assert payload["status"] == "failed"
        assert "encoder runtime off" in payload["message"]
        assert payload["state"]["encoder_mismatch"]["status"] == "encoder_config_inactive"
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_synced_encoder_config_is_marked_stale_if_status_reports_runtime_off(monkeypatch):
    original_config = main.config
    try:
        monkeypatch.setattr(main, "config", config_with_enabled_encoder(original_config))
        reset_runtime_state()
        main.state.controller_capabilities = {"protocol": 4, "encoder_config": True}
        main.state.config_sync_status = "synced"

        main.apply_controller_status(encoder_status_line(closed_loop="off", enc="0000"))

        assert main.state.config_sync_status == "stale"
        assert "encoder runtime off" in main.state.config_sync_message
        assert main.state.encoder_mismatch["status"] == "encoder_config_inactive"
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_hardware_sync_skips_disabled_encoder_lines_for_legacy_controller(monkeypatch):
    reset_runtime_state()
    main.state.controller_capabilities = {
        "protocol": 3,
        "config": True,
        "encoder": False,
        "encoder_config": False,
        "raw": "HELLO name=esp32s3-arm firmware=arm_controller protocol=3 config=1",
    }
    fake = FakeSerial(["OK command=CONFIG axes=4 hw=hardware enabled=1111"])
    monkeypatch.setattr(main, "serial_client", fake)
    client = TestClient(main.app)

    payload = client.post("/api/hardware/sync").json()

    assert payload["ok"] is True
    assert payload["status"] == "synced"
    assert not any(line.startswith("CONFIG ENCODER") for line in fake.sent)
    assert "skipped" in payload["message"]


def test_enabled_encoder_bus_requires_protocol_v4_encoder_firmware(monkeypatch):
    original_config = main.config
    raw = deepcopy(original_config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["bus"].update({"sck_pin": 12, "miso_pin": 13, "mosi_pin": 14})
    raw["encoders"]["axes"][0].update({"enabled": True, "cs_pin": 15})
    patched = type(original_config)(**{**original_config.__dict__, "raw": raw})
    try:
        monkeypatch.setattr(main, "config", patched)
        reset_runtime_state()
        main.state.controller_capabilities = {
            "protocol": 3,
            "config": True,
            "encoder": False,
            "encoder_config": False,
            "raw": "HELLO name=esp32s3-arm firmware=arm_controller protocol=3 config=1",
        }
        fake = FakeSerial([])
        monkeypatch.setattr(main, "serial_client", fake)
        client = TestClient(main.app)

        payload = client.post("/api/hardware/sync").json()

        assert payload["ok"] is False
        assert payload["status"] == "unsupported"
        assert "protocol-v4 encoder config support" in payload["message"]
        assert fake.sent == []
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_hardware_sync_requires_disarmed_hardware(monkeypatch):
    reset_runtime_state()
    main.state.hardware_armed = True
    fake = FakeSerial([])
    monkeypatch.setattr(main, "serial_client", fake)
    client = TestClient(main.app)

    payload = client.post("/api/hardware/sync").json()

    assert payload["ok"] is False
    assert payload["status"] == "blocked"
    assert "disarm" in payload["message"]
    assert fake.sent == []


def test_hardware_sync_reports_unsupported(monkeypatch):
    reset_runtime_state()
    fake = FakeSerial(["ERR code=UNKNOWN message=CONFIG"])
    monkeypatch.setattr(main, "serial_client", fake)
    client = TestClient(main.app)

    response = client.post("/api/hardware/sync")

    payload = response.json()
    assert not payload["ok"]
    assert payload["status"] == "unsupported"


def test_partial_hardware_sync_allows_arm_as_mixed(monkeypatch):
    original_config = main.config
    try:
        monkeypatch.setattr(main, "config", config_with_only_first_axis_enabled(original_config))
        reset_runtime_state()
        fake = FakeSerial(
            [
                "OK command=CONFIG axes=4 hw=mixed enabled=1000",
                "OK command=ARM armed=1",
                "STATUS state=idle homed=0 known=1 pose_source=setpose armed=1 hw=mixed enabled=1000 "
                "j1=0.0 j2=20.0 j3=20.0 j4=0.0 fault=OK",
            ]
        )
        monkeypatch.setattr(main, "serial_client", fake)
        client = TestClient(main.app)

        sync_response = client.post("/api/hardware/sync")
        arm_response = client.post("/api/hardware-arm", json={"armed": True})

        sync_payload = sync_response.json()
        arm_payload = arm_response.json()
        assert sync_payload["ok"]
        assert sync_payload["evaluation"]["mode"] == "mixed"
        assert arm_payload["ok"]
        assert arm_payload["state"]["hardware_armed"] is True
        assert arm_payload["state"]["hardware_mode"] == "mixed"
        assert "ARM 1" in fake.sent
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_unsupported_config_blocks_hardware_arm(monkeypatch):
    original_config = main.config
    try:
        monkeypatch.setattr(main, "config", config_with_only_first_axis_enabled(original_config))
        reset_runtime_state()
        main.state.config_sync_status = "unsupported"
        fake = FakeSerial([])
        monkeypatch.setattr(main, "serial_client", fake)
        client = TestClient(main.app)

        response = client.post("/api/hardware-arm", json={"armed": True})

        payload = response.json()
        assert not payload["ok"]
        assert "not synced" in payload["error"]
        assert fake.sent == []
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_hardware_motion_blocks_unsynced_config(monkeypatch):
    original_config = main.config
    try:
        monkeypatch.setattr(main, "config", config_with_only_first_axis_enabled(original_config))
        reset_runtime_state()
        main.state.hardware_armed = True
        client = TestClient(main.app)

        response = client.post("/api/joints", json={"angles_deg": main.config.home_pose})

        payload = response.json()
        assert not payload["ok"]
        assert "not synced" in payload["error"]
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_enabled_axis_with_missing_pins_is_invalid(monkeypatch):
    original_config = main.config
    try:
        first_joint = replace(
            original_config.joints[0],
            hardware=replace(
                original_config.joints[0].hardware,
                stepper=replace(original_config.joints[0].hardware.stepper, enabled=True, step_pin=-1, dir_pin=-1),
            ),
        )
        patched = replace(original_config, joints=[first_joint, *original_config.joints[1:]])
        monkeypatch.setattr(main, "config", patched)
        reset_runtime_state()
        client = TestClient(main.app)

        response = client.post("/api/hardware/sync")

        payload = response.json()
        assert not payload["ok"]
        assert payload["status"] == "invalid"
        assert payload["evaluation"]["mode"] == "invalid"
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_shoulder_encoder_chip_select_conflict_blocks_sync(monkeypatch):
    original_config = main.config
    raw = deepcopy(original_config.raw)
    raw["encoders"]["enabled"] = True
    raw["encoders"]["axes"] = [
        {
            **raw["encoders"]["axes"][0],
            "joint": 2,
            "enabled": True,
            "cs_pin": original_config.joints[1].hardware.stepper.dir_pin,
        }
    ]
    patched = type(original_config)(**{**original_config.__dict__, "raw": raw})
    monkeypatch.setattr(main, "config", patched)
    reset_runtime_state()

    evaluation = main.evaluate_hardware_config()

    assert evaluation["mode"] == "invalid"
    assert any(
        "shoulder encoder CS" in error and "shoulder DIR" in error
        for error in evaluation["errors"]
    )


def test_config_change_classification_separates_model_mapping_and_io():
    original = main.config
    model_change = replace(
        original,
        links=replace(original.links, base_height_mm=original.links.base_height_mm + 1.0),
    )
    mapping_change = config_with_first_joint_mapping_change(original)
    io_change = config_with_first_joint_io_change(original)

    model = main.classify_config_change(original, model_change)
    mapping = main.classify_config_change(original, mapping_change)
    io = main.classify_config_change(original, io_change)

    assert model["categories"] == ["model"]
    assert model["previews_invalidated"] is True
    assert model["sync_required"] is False
    assert model["pose_invalidated"] is False

    assert "actuator_mapping" in mapping["categories"]
    assert mapping["sync_required"] is True
    assert mapping["disarm_required"] is True
    assert mapping["pose_invalidated"] is True

    assert "io" in io["categories"]
    assert io["sync_required"] is True
    assert io["pose_invalidated"] is False


def test_pose_invalidating_config_reload_requires_sync_and_setpose(monkeypatch):
    original_config = main.config
    original_state = deepcopy(main.state.__dict__)
    changed = config_with_first_joint_mapping_change(original_config)
    change = main.classify_config_change(original_config, changed)
    try:
        main.state.simulation = False
        main.state.connected = True
        main.state.hardware_armed = False
        main.state.known_pose = True
        main.state.pose_source = "setpose"
        main.state.config_sync_status = "synced"
        main.reload_runtime_config(changed, change)

        assert main.state.known_pose is False
        assert main.state.homed is False
        assert main.state.pose_source == "unknown"
        assert main.state.config_sync_status == "stale"
        assert main.state.config_change["pose_revalidation_required"] is True
        assert "Set Pose" in main.state.config_sync_message
    finally:
        main.config = original_config
        main.limiter.config = original_config
        main.serial_client.config = original_config.serial
        main.cartesian_servo.reconfigure(original_config.links, original_config.joints)
        main.state.__dict__.clear()
        main.state.__dict__.update(original_state)


def test_model_only_reload_preserves_controller_sync_and_known_pose():
    original_config = main.config
    original_state = deepcopy(main.state.__dict__)
    changed = replace(
        original_config,
        links=replace(original_config.links, base_height_mm=original_config.links.base_height_mm + 1.0),
    )
    change = main.classify_config_change(original_config, changed)
    try:
        main.state.simulation = False
        main.state.connected = True
        main.state.hardware_armed = True
        main.state.known_pose = True
        main.state.pose_source = "open_loop_estimate"
        main.state.config_sync_status = "synced"
        main.reload_runtime_config(changed, change)

        assert main.state.known_pose is True
        assert main.state.pose_source == "open_loop_estimate"
        assert main.state.config_sync_status == "synced"
        assert main.state.config_change["previews_invalidated"] is True
        assert main.state.config_change["sync_required"] is False
    finally:
        main.config = original_config
        main.limiter.config = original_config
        main.serial_client.config = original_config.serial
        main.cartesian_servo.reconfigure(original_config.links, original_config.joints)
        main.state.__dict__.clear()
        main.state.__dict__.update(original_state)


def test_synced_mapping_change_does_not_restore_pose_knowledge(monkeypatch):
    reset_runtime_state()
    main.state.known_pose = False
    main.state.pose_source = "unknown"
    main.state.config_change = {
        "pose_invalidated": True,
        "pose_revalidation_required": True,
    }
    fake = FakeSerial(["OK command=CONFIG axes=4 hw=simulated enabled=0000 pose_invalidated=1"])
    monkeypatch.setattr(main, "serial_client", fake)
    client = TestClient(main.app)

    payload = client.post("/api/hardware/sync").json()

    assert payload["ok"] is True
    assert payload["pose_revalidation_required"] is True
    assert payload["state"]["known_pose"] is False
    assert "Set Pose" in payload["message"]
