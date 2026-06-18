from fastapi.testclient import TestClient
from pytest import approx

import app.main as main
from app.kinematics import forward_kinematics
from app.robot_state import MotionState


def test_hardware_motion_requires_known_pose():
    main.cancel_motion_tasks()
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = True
    main.state.known_pose = False
    main.state.config_sync_status = "synced"
    main.state.motion_state = MotionState.IDLE
    main.state.clear_error()
    client = TestClient(main.app)

    response = client.post("/api/joints", json={"angles_deg": main.config.home_pose})

    payload = response.json()
    assert not payload["ok"]
    assert "pose is unknown" in payload["error"]
    main.state.simulation = True
    main.state.known_pose = True


def test_simulation_setpose_returns_matching_fk():
    main.cancel_motion_tasks()
    main.state.connected = True
    main.state.simulation = True
    main.state.motion_state = MotionState.IDLE
    main.state.clear_error()
    client = TestClient(main.app)
    angles = [20.0, 60.0, -30.0, -20.0]

    response = client.post("/api/hardware/setpose", json={"angles_deg": angles})

    payload = response.json()
    expected = forward_kinematics(angles, main.config.links)
    assert payload["ok"]
    assert payload["state"]["reported_angles_deg"] == approx(angles)
    assert payload["state"]["fk"]["x_mm"] == approx(expected["x_mm"])
    assert payload["state"]["fk"]["y_mm"] == approx(expected["y_mm"])
    assert payload["state"]["fk"]["z_mm"] == approx(expected["z_mm"])
