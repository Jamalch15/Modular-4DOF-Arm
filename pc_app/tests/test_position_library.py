from copy import deepcopy
from dataclasses import replace
from shutil import copyfile

import pytest
from fastapi.testclient import TestClient

import app.main as main
from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.demo_settings import drop_zones, named_positions
from app.position_library import (
    POSITION_LIBRARY_SCHEMA_VERSION,
    normalize_position_record,
    position_library_records,
    validate_position_record,
)
from app.robot_state import MotionState
from app.task_destinations import TASK_DESTINATIONS_SCHEMA_VERSION, task_destination_errors
from app.tasks import build_pick_and_place_sequence


@pytest.fixture(autouse=True)
def restore_main_config():
    original_config = main.config
    original_config_id = main.RUNNING_CONFIG_ID
    try:
        yield
    finally:
        main.cancel_motion_tasks()
        main.config = original_config
        main.RUNNING_CONFIG_ID = original_config_id


def test_position_library_migrates_named_positions_as_stable_records():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["named_positions"] = deepcopy(named_positions(config))
    raw.pop("position_library", None)
    legacy_config = replace(config, raw=raw)

    library = position_library_records(legacy_config, named_positions(legacy_config))

    assert {"home", "safe", "pickup_test", "dropoff_a", "dropoff_b"} <= set(library)
    assert library["dropoff_a"]["id"] == "dropoff_a"
    assert library["dropoff_a"]["display_name"] == "Dropoff A"
    assert library["dropoff_a"]["type"] == "cartesian"
    assert library["dropoff_a"]["target"]["x_mm"] == -160.0
    assert "draft" not in library["dropoff_a"]


def test_custom_position_library_record_keeps_display_name_and_legacy_bridge():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["position_library"] = {
        "schema_version": POSITION_LIBRARY_SCHEMA_VERSION,
        "positions": {
            "inspection_slot": {
                "type": "cartesian",
                "display_name": "Inspection Slot",
                "description": "Working assumption for a camera-inspection pose.",
                "target": {"x_mm": 120.0, "y_mm": 180.0, "z_mm": 45.0, "phi_deg": 0.0},
            }
        },
    }
    patched = replace(config, raw=raw)

    library = position_library_records(patched, named_positions(patched))
    legacy = named_positions(patched)

    assert library["inspection_slot"]["display_name"] == "Inspection Slot"
    assert library["inspection_slot"]["description"].startswith("Working assumption")
    assert legacy["inspection_slot"]["label"] == "Inspection Slot"
    assert legacy["inspection_slot"]["target"]["x_mm"] == 120.0


def test_position_validation_is_not_camera_workspace_validation():
    config = load_config(EXAMPLE_CONFIG_PATH)
    record = normalize_position_record(
        config,
        "outside_camera_but_reachable",
        {
            "type": "cartesian",
            "display_name": "Outside Camera But Reachable",
            "target": {"x_mm": 250.0, "y_mm": 0.0, "z_mm": 120.0, "phi_auto": True},
        },
    )

    assert validate_position_record(config, "outside_camera_but_reachable", record) == []


def test_task_destination_can_reference_position_library_record():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["position_library"] = {
        "schema_version": POSITION_LIBRARY_SCHEMA_VERSION,
        "positions": {
            "blue_bin_anchor": {
                "type": "cartesian",
                "display_name": "Blue Bin Anchor",
                "target": {"x_mm": 120.0, "y_mm": 180.0, "z_mm": 45.0, "phi_deg": 0.0},
            }
        },
    }
    raw["task_destinations"] = {
        "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
        "destinations": {
            "blue_bin": {
                "position_id": "blue_bin_anchor",
                "grid": {"rows": 1, "columns": 2, "x_spacing_mm": 20.0, "y_spacing_mm": 0.0},
            }
        },
    }
    patched = replace(config, raw=raw)

    zones = drop_zones(patched)
    sequence = build_pick_and_place_sequence(
        patched,
        {"x_mm": 0.0, "y_mm": 180.0, "z_mm": 30.0, "phi_auto": True},
        "blue_bin",
    )

    assert zones["blue_bin"]["position_id"] == "blue_bin_anchor"
    assert zones["blue_bin"]["position_display_name"] == "Blue Bin Anchor"
    assert zones["blue_bin"]["x_mm"] == 120.0
    assert sequence["ok"], sequence
    assert sequence["drop_target"]["position_id"] == "blue_bin_anchor"


def test_task_destinations_are_independent_from_named_position_values():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw.pop("drop_zones", None)
    raw["named_positions"] = deepcopy(named_positions(config))
    raw.pop("position_library", None)
    raw["named_positions"]["dropoff_a"]["target"]["x_mm"] = 999.0
    raw["task_destinations"] = {
        "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
        "destinations": {
            "inspection_bin": {
                "label": "Inspection Bin",
                "x_mm": 80.0,
                "y_mm": 200.0,
                "z_mm": 45.0,
                "phi_deg": 0.0,
            }
        },
    }
    patched = replace(config, raw=raw)

    destinations = drop_zones(patched)

    assert set(destinations) == {"inspection_bin"}
    assert destinations["inspection_bin"]["x_mm"] == 80.0


def test_default_task_destinations_do_not_follow_named_dropoff_positions():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw.pop("drop_zones", None)
    raw.pop("task_destinations", None)
    raw["named_positions"] = deepcopy(named_positions(config))
    raw.pop("position_library", None)
    raw["named_positions"]["dropoff_a"]["target"]["x_mm"] = 999.0
    patched = replace(config, raw=raw)

    destinations = drop_zones(patched)

    assert destinations["dropoff_a"]["x_mm"] == -160.0


def test_malformed_task_destination_is_reported_instead_of_ignored():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["task_destinations"] = {
        "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
        "destinations": {"broken": "not-an-object"},
    }
    patched = replace(config, raw=raw)

    assert task_destination_errors(patched, named_positions(patched)) == {
        "broken": ["task destination broken must be an object"]
    }


def test_task_destination_id_must_match_stable_mapping_key():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["task_destinations"] = {
        "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
        "destinations": {
            "stable_bin": {
                "id": "renamed_bin",
                "x_mm": 80.0,
                "y_mm": 200.0,
                "z_mm": 45.0,
            }
        },
    }
    patched = replace(config, raw=raw)

    assert "must match its stable mapping key" in task_destination_errors(
        patched,
        named_positions(patched),
    )["stable_bin"][0]


def test_public_config_reports_malformed_task_destination_without_crashing():
    original = main.config
    raw = deepcopy(original.raw)
    raw["task_destinations"] = {
        "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
        "destinations": {"broken": "not-an-object"},
    }
    main.config = replace(original, raw=raw)

    payload = main.public_config()

    assert payload["task_destinations"]["destinations"] == {}
    assert payload["validation"]["task_destination_errors"]["broken"]


def test_position_library_api_saves_new_schema_and_legacy_bridge(tmp_path, monkeypatch):
    target = tmp_path / "robot.local.yaml"
    copyfile(EXAMPLE_CONFIG_PATH, target)
    main.cancel_motion_tasks()
    main.config = load_config(target)
    main.state.motion_state = MotionState.IDLE
    main.state.live_motion_enabled = False
    main.state.hardware_armed = False
    monkeypatch.setattr(main, "ensure_local_config", lambda: target)
    client = TestClient(main.app)
    existing = client.get("/api/position-library").json()["positions"]

    payload = client.post(
        "/api/position-library",
        json={
            "positions": {
                **existing,
                "fixture_slot": {
                    "type": "cartesian",
                    "display_name": "Fixture Slot",
                    "target": {"x_mm": -120.0, "y_mm": 150.0, "z_mm": 45.0, "phi_deg": 0.0},
                },
            }
        },
    ).json()
    saved = load_config(target)

    assert payload["ok"], payload
    assert saved.raw["position_library"]["schema_version"] == POSITION_LIBRARY_SCHEMA_VERSION
    assert saved.raw["position_library"]["positions"]["fixture_slot"]["display_name"] == "Fixture Slot"
    assert saved.raw["named_positions"]["fixture_slot"]["label"] == "Fixture Slot"


def test_legacy_named_positions_api_updates_position_library_first_config(tmp_path, monkeypatch):
    target = tmp_path / "robot.local.yaml"
    copyfile(EXAMPLE_CONFIG_PATH, target)
    main.cancel_motion_tasks()
    main.config = load_config(target)
    main.state.motion_state = MotionState.IDLE
    main.state.live_motion_enabled = False
    main.state.hardware_armed = False
    monkeypatch.setattr(main, "ensure_local_config", lambda: target)
    client = TestClient(main.app)
    positions = client.get("/api/named-positions").json()["positions"]
    positions["safe"] = {
        "type": "joint",
        "label": "Safe",
        "angles_deg": [0.0, 30.0, 20.0, 0.0],
    }

    payload = client.post("/api/named-positions", json={"positions": positions}).json()
    saved = load_config(target)

    assert payload["ok"], payload
    assert saved.raw["position_library"]["positions"]["safe"]["angles_deg"] == [0.0, 30.0, 20.0, 0.0]
    assert named_positions(saved)["safe"]["angles_deg"] == [0.0, 30.0, 20.0, 0.0]


def test_position_library_allows_duplicate_display_names_with_stable_ids(tmp_path, monkeypatch):
    target = tmp_path / "robot.local.yaml"
    copyfile(EXAMPLE_CONFIG_PATH, target)
    main.cancel_motion_tasks()
    main.config = load_config(target)
    main.state.motion_state = MotionState.IDLE
    main.state.live_motion_enabled = False
    main.state.hardware_armed = False
    monkeypatch.setattr(main, "ensure_local_config", lambda: target)
    client = TestClient(main.app)

    payload = client.post(
        "/api/position-library",
        json={
            "positions": {
                "fixture_left": {
                    "type": "cartesian",
                    "display_name": "Fixture",
                    "target": {"x_mm": -120.0, "y_mm": 150.0, "z_mm": 45.0, "phi_deg": 0.0},
                },
                "fixture_right": {
                    "type": "cartesian",
                    "display_name": "Fixture",
                    "target": {"x_mm": 120.0, "y_mm": 150.0, "z_mm": 45.0, "phi_deg": 0.0},
                },
            }
        },
    ).json()
    saved = load_config(target)

    assert payload["ok"], payload
    assert set(saved.raw["position_library"]["positions"]) == {"fixture_left", "fixture_right"}
    assert saved.raw["position_library"]["positions"]["fixture_left"]["display_name"] == "Fixture"
    assert saved.raw["position_library"]["positions"]["fixture_right"]["display_name"] == "Fixture"
    assert saved.raw["position_library"]["positions"]["fixture_left"]["created_at"]
    assert saved.raw["position_library"]["positions"]["fixture_left"]["updated_at"]


def test_position_library_rejects_record_id_that_differs_from_stable_key(tmp_path, monkeypatch):
    target = tmp_path / "robot.local.yaml"
    copyfile(EXAMPLE_CONFIG_PATH, target)
    main.cancel_motion_tasks()
    main.config = load_config(target)
    main.state.motion_state = MotionState.IDLE
    main.state.live_motion_enabled = False
    main.state.hardware_armed = False
    monkeypatch.setattr(main, "ensure_local_config", lambda: target)
    client = TestClient(main.app)

    payload = client.post(
        "/api/position-library",
        json={
            "positions": {
                "stable_key": {
                    "id": "changed_key",
                    "type": "joint",
                    "display_name": "Changed key",
                    "angles_deg": [0.0, 20.0, 20.0, 0.0],
                }
            }
        },
    ).json()

    assert not payload["ok"]
    assert "must match its stable library key" in payload["errors"]["stable_key"][0]


def test_position_library_rejects_deleting_position_used_by_task_destination(tmp_path, monkeypatch):
    target = tmp_path / "robot.local.yaml"
    copyfile(EXAMPLE_CONFIG_PATH, target)
    main.cancel_motion_tasks()
    main.config = load_config(target)
    main.state.motion_state = MotionState.IDLE
    main.state.live_motion_enabled = False
    main.state.hardware_armed = False
    monkeypatch.setattr(main, "ensure_local_config", lambda: target)
    client = TestClient(main.app)

    positions = client.get("/api/position-library").json()["positions"]
    positions["fixture_slot"] = {
        "type": "cartesian",
        "display_name": "Fixture Slot",
        "target": {"x_mm": -120.0, "y_mm": 150.0, "z_mm": 45.0, "phi_deg": 0.0},
    }
    assert client.post("/api/position-library", json={"positions": positions}).json()["ok"]
    assert client.post(
        "/api/task-mappings",
        json={
            "color_profiles": {"red": {"enabled": True, "drop_zone": "fixture_bin"}},
            "task_destinations": {
                "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
                "destinations": {
                    "fixture_bin": {
                        "label": "Fixture Bin",
                        "position_id": "fixture_slot",
                    }
                },
            },
        },
    ).json()["ok"]

    positions = client.get("/api/position-library").json()["positions"]
    positions.pop("fixture_slot")
    payload = client.post("/api/position-library", json={"positions": positions}).json()

    assert not payload["ok"]
    assert "break task destination references" in payload["error"]


def test_task_mapping_api_saves_new_schema_legacy_bridge_and_clears_draft(tmp_path, monkeypatch):
    target = tmp_path / "robot.local.yaml"
    copyfile(EXAMPLE_CONFIG_PATH, target)
    main.cancel_motion_tasks()
    main.config = load_config(target)
    main.state.motion_state = MotionState.IDLE
    main.state.live_motion_enabled = False
    main.state.hardware_armed = False
    monkeypatch.setattr(main, "ensure_local_config", lambda: target)
    client = TestClient(main.app)

    payload = client.post(
        "/api/task-mappings",
        json={
            "color_profiles": {
                "red": {
                    "enabled": True,
                    "drop_zone": "red_bin",
                    "draft": True,
                }
            },
            "task_destinations": {
                "schema_version": TASK_DESTINATIONS_SCHEMA_VERSION,
                "destinations": {
                    "red_bin": {
                        "label": "Red Bin",
                        "position_id": "dropoff_a",
                        "grid": {
                            "rows": 1,
                            "columns": 2,
                            "x_spacing_mm": 20.0,
                            "y_spacing_mm": 0.0,
                        },
                    }
                },
            },
        },
    ).json()
    saved = load_config(target)

    assert payload["ok"], payload
    assert saved.raw["task_destinations"]["schema_version"] == TASK_DESTINATIONS_SCHEMA_VERSION
    assert saved.raw["task_destinations"]["destinations"]["red_bin"]["position_id"] == "dropoff_a"
    assert saved.raw["drop_zones"]["red_bin"]["x_mm"] == -160.0
    assert "draft" not in saved.raw["color_profiles"]["red"]


def test_global_settings_save_does_not_persist_color_profile_draft_marker(tmp_path, monkeypatch):
    target = tmp_path / "robot.local.yaml"
    copyfile(EXAMPLE_CONFIG_PATH, target)
    main.cancel_motion_tasks()
    main.config = load_config(target)
    main.state.motion_state = MotionState.IDLE
    main.state.live_motion_enabled = False
    main.state.hardware_armed = False
    monkeypatch.setattr(main, "ensure_local_config", lambda: target)
    client = TestClient(main.app)

    payload = client.post(
        "/api/config/calibration",
        json={
            "color_profiles": {
                "green": {
                    "enabled": True,
                    "drop_zone": "",
                    "draft": True,
                }
            }
        },
    ).json()
    saved = load_config(target)

    assert payload["ok"], payload
    assert "draft" not in saved.raw["color_profiles"]["green"]
