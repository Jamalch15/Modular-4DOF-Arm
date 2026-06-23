import asyncio
import shutil
from copy import deepcopy
from dataclasses import replace

import pytest
from fastapi.testclient import TestClient

from app import main
from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.robot_state import MotionState


def detection(color: str, x: float = -120.0, y: float = 150.0, detection_id: str | None = None) -> dict:
    return {
        "id": detection_id or color,
        "ok": True,
        "color": color,
        "label": color,
        "confidence": 0.95,
        "area_px": 600,
        "robot": {"x_mm": x, "y_mm": y, "z_mm": 999.0},
    }


@pytest.fixture(autouse=True)
def restore_runtime_state():
    snapshot = {
        "simulation": main.state.simulation,
        "connected": main.state.connected,
        "motion_state": main.state.motion_state,
        "motion_execution_state": main.state.motion_execution_state,
        "task_execution": dict(main.state.task_execution or {}),
        "last_error": main.state.last_error,
        "last_command": main.state.last_command,
        "target_angles_deg": list(main.state.target_angles_deg),
        "reported_angles_deg": list(main.state.reported_angles_deg),
        "task_task": main.task_task,
        "simulation_vision_queue": list(main.simulation_vision_queue),
        "latest_vision_snapshot": dict(main.latest_vision_snapshot),
        "task_previews": dict(main.task_previews),
    }
    yield
    main.state.simulation = snapshot["simulation"]
    main.state.connected = snapshot["connected"]
    main.state.motion_state = snapshot["motion_state"]
    main.state.motion_execution_state = snapshot["motion_execution_state"]
    main.state.task_execution = snapshot["task_execution"]
    main.state.last_error = snapshot["last_error"]
    main.state.last_command = snapshot["last_command"]
    main.state.target_angles_deg = snapshot["target_angles_deg"]
    main.state.reported_angles_deg = snapshot["reported_angles_deg"]
    main.task_task = snapshot["task_task"]
    main.simulation_vision_queue[:] = snapshot["simulation_vision_queue"]
    main.latest_vision_snapshot.clear()
    main.latest_vision_snapshot.update(snapshot["latest_vision_snapshot"])
    main.task_previews.clear()
    main.task_previews.update(snapshot["task_previews"])
    main.task_selection_events.clear()
    main.task_selection_choices.clear()
    main.task_confirmation_events.clear()


def configure_task_runtime(monkeypatch, task_settings: dict | None = None) -> dict:
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.motion_execution_state = "idle"
    main.state.task_execution = {}
    main.start_task_execution_state(
        run_id="test-run",
        preview_id="preview-1",
        task="color_sorting",
        strategy="closed_loop",
        total_objects=int((task_settings or {}).get("max_objects", 1)),
        settings=task_settings or {},
    )
    return {
        "id": "preview-1",
        "task": "color_sorting",
        "strategy": "closed_loop",
        "task_settings": task_settings or {"execution_strategy": "closed_loop", "max_objects": 1},
        "settings": {},
        "branch": "auto",
    }


def install_closed_loop_stubs(monkeypatch, captures: list[list[dict]], executed_plans: list[dict]) -> dict[str, int]:
    counters = {"captures": 0, "camera_clear_moves": 0, "preflights": 0}

    async def fake_broadcast():
        return None

    async def fake_move_named_position(name, settings, branch, label):
        counters["camera_clear_moves"] += 1
        return {"ok": True}

    async def fake_capture():
        index = min(counters["captures"], len(captures) - 1)
        counters["captures"] += 1
        return {
            "ok": True,
            "captured_at": counters["captures"],
            "detections": captures[index],
            "workspace": {"status": "test"},
            "provider": "test",
            "calibration_source": "test",
        }

    async def fake_execute_sequence(sequence, settings, branch, *, terminal_on_finish=True):
        executed_plans.append(sequence)
        return {"ok": True}

    def fake_build_preview(**kwargs):
        counters["preflights"] += 1
        return {"ok": True, "preview": {"trajectory": {"mode": "program"}}}

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "move_task_named_position", fake_move_named_position)
    monkeypatch.setattr(main, "closed_loop_capture", fake_capture)
    monkeypatch.setattr(main, "execute_task_sequence", fake_execute_sequence)
    monkeypatch.setattr(main, "build_preview", fake_build_preview)
    monkeypatch.setattr(main, "task_motion_gate_reason", lambda: None)
    return counters


def test_tool_dimension_validation_is_per_active_tool():
    config = load_config(EXAMPLE_CONFIG_PATH)
    raw = deepcopy(config.raw)
    raw["tools"]["presets"]["gripper"]["dimensions_validated"] = True
    raw["tools"]["presets"]["magnet"]["dimensions_validated"] = False

    assert main.active_tool_dimensions_validated(replace(config, raw=raw)) is True

    raw["tools"]["active"] = "magnet"
    assert main.active_tool_dimensions_validated(replace(config, raw=raw)) is False


def test_closed_loop_preview_rejects_invalid_camera_clear_position(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.motion_execution_state = "idle"
    main.state.reported_angles_deg = list(config.home_pose)

    result = asyncio.run(
        main.preview_task(
            main.TaskPreviewRequest(
                task="color_sorting",
                detections=[detection("red", -120.0, 150.0, "r1")],
                task_settings={
                    "execution_strategy": "closed_loop",
                    "max_objects": 1,
                    "camera_clear_position": "deleted_camera_clear",
                },
                settings={},
                branch="auto",
            )
        )
    )

    assert result["ok"] is False
    assert "camera clear" in result["error"]
    assert "deleted_camera_clear" in result["error"]


def test_changing_tcp_clears_only_that_tool_validation():
    config = load_config(EXAMPLE_CONFIG_PATH)
    previous = main.tools_settings(config)
    previous["presets"]["gripper"]["dimensions_validated"] = True
    previous["presets"]["magnet"]["dimensions_validated"] = True
    updated = deepcopy(previous)
    updated["presets"]["gripper"]["tcp_offset_mm"]["z"] += 1.0

    main._invalidate_changed_tool_validations(previous, updated)

    assert updated["presets"]["gripper"]["dimensions_validated"] is False
    assert updated["presets"]["magnet"]["dimensions_validated"] is True


def test_tool_validation_endpoint_persists_active_tool_acknowledgement(monkeypatch, tmp_path):
    config = load_config(EXAMPLE_CONFIG_PATH)
    config_path = tmp_path / "robot.local.yaml"
    shutil.copyfile(EXAMPLE_CONFIG_PATH, config_path)
    monkeypatch.setattr(main, "config", config)
    monkeypatch.setattr(main, "ensure_local_config", lambda: config_path)
    main.state.hardware_armed = False

    async def fake_broadcast():
        return None

    def fake_reload(loaded_config, change):
        main.config = loaded_config
        main.state.config_change = change
        return change

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "reload_runtime_config", fake_reload)

    result = asyncio.run(
        main.set_active_tool_dimensions_validation(
            main.ToolDimensionsValidationRequest(validated=True)
        )
    )
    saved = load_config(config_path)

    assert result["ok"] is True
    assert saved.raw["tools"]["presets"]["gripper"]["dimensions_validated"] is True
    assert main.active_tool_dimensions_validated(saved) is True


def test_closed_loop_recaptures_until_max_objects(monkeypatch):
    executed: list[dict] = []
    preview = configure_task_runtime(
        monkeypatch,
        {"execution_strategy": "closed_loop", "max_objects": 2, "ordering": {"policy": "left_to_right"}},
    )
    counters = install_closed_loop_stubs(
        monkeypatch,
        [
            [detection("red", -120.0, 150.0, "r1")],
            [detection("blue", 120.0, 180.0, "b1")],
        ],
        executed,
    )

    asyncio.run(main.execute_closed_loop_sorting(preview))

    assert counters["captures"] == 2
    assert counters["camera_clear_moves"] == 2
    assert counters["preflights"] == 2
    assert [plan["objects"][0]["detection_id"] for plan in executed] == ["r1", "b1"]
    assert main.state.task_execution["status"] == "completed"
    assert main.state.task_execution["terminal_reason"] == "max_objects reached"
    assert main.state.task_execution["completed_count"] == 2


def test_closed_loop_can_pause_for_confirmation_between_objects(monkeypatch):
    executed: list[dict] = []
    preview = configure_task_runtime(
        monkeypatch,
        {
            "execution_strategy": "closed_loop",
            "cycle_confirmation": "confirm_each_object",
            "max_objects": 2,
        },
    )
    counters = install_closed_loop_stubs(
        monkeypatch,
        [
            [detection("red", -120.0, 150.0, "r1")],
            [detection("blue", 120.0, 180.0, "b1")],
        ],
        executed,
    )

    async def run_and_continue():
        task = asyncio.create_task(main.execute_closed_loop_sorting(preview))
        for _ in range(100):
            if main.state.task_execution.get("status") == "waiting_for_confirmation":
                break
            await asyncio.sleep(0.01)
        assert main.state.task_execution["status"] == "waiting_for_confirmation"
        assert counters["captures"] == 1
        assert len(executed) == 1
        response = await main.continue_task(main.TaskContinueRequest(run_id="test-run"))
        assert response["ok"] is True
        await task

    asyncio.run(run_and_continue())

    assert counters["captures"] == 2
    assert [plan["objects"][0]["detection_id"] for plan in executed] == ["r1", "b1"]
    assert main.state.task_execution["status"] == "completed"


def test_closed_loop_empty_scene_completes_without_execution(monkeypatch):
    executed: list[dict] = []
    preview = configure_task_runtime(monkeypatch, {"execution_strategy": "closed_loop", "max_objects": 3})
    counters = install_closed_loop_stubs(monkeypatch, [[]], executed)

    asyncio.run(main.execute_closed_loop_sorting(preview))

    assert counters["captures"] == 1
    assert executed == []
    assert main.state.task_execution["status"] == "completed"
    assert main.state.task_execution["terminal_reason"] == "empty scene"


def test_batch_once_can_pause_between_objects(monkeypatch):
    preview = configure_task_runtime(
        monkeypatch,
        {
            "execution_strategy": "batch_once",
            "cycle_confirmation": "confirm_each_object",
            "max_objects": 2,
        },
    )
    sequence = {
        "steps": [
            {
                "kind": "move",
                "label": "object 1 safe",
                "object_index": 1,
                "waypoint": {"type": "joint", "mode": "joint", "angles_deg": [0.0, 35.0, 15.0, 0.0]},
            },
            {
                "kind": "move",
                "label": "object 2 safe",
                "object_index": 2,
                "waypoint": {"type": "joint", "mode": "joint", "angles_deg": [0.0, 35.0, 15.0, 0.0]},
            },
        ]
    }

    async def fake_broadcast():
        return None

    def fake_build_preview(**kwargs):
        return {"ok": True, "preview": {"trajectory": {"mode": "program"}}}

    async def fake_execute(_preview):
        return None

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "build_preview", fake_build_preview)
    monkeypatch.setattr(main, "execute_waypoint_path", fake_execute)

    async def run_and_continue():
        task = asyncio.create_task(
            main.execute_task_sequence(
                sequence,
                {
                    **preview["settings"],
                    "cycle_confirmation": "confirm_each_object",
                },
                preview["branch"],
            )
        )
        for _ in range(100):
            if main.state.task_execution.get("status") == "waiting_for_confirmation":
                break
            await asyncio.sleep(0.01)
        assert main.state.task_execution["status"] == "waiting_for_confirmation"
        assert main.state.task_execution["completed_count"] == 1
        response = await main.continue_task(main.TaskContinueRequest(run_id="test-run"))
        assert response["ok"] is True
        await task

    asyncio.run(run_and_continue())

    assert main.state.task_execution["status"] == "completed"
    assert main.state.task_execution["completed_count"] == 2


def test_closed_loop_manual_selection_waits_and_resumes(monkeypatch):
    executed: list[dict] = []
    preview = configure_task_runtime(
        monkeypatch,
        {"execution_strategy": "closed_loop", "max_objects": 1, "ordering": {"policy": "manual"}},
    )
    install_closed_loop_stubs(
        monkeypatch,
        [[detection("red", -120.0, 150.0, "r1"), detection("blue", 120.0, 180.0, "b1")]],
        executed,
    )

    async def run_and_select():
        task = asyncio.create_task(main.execute_closed_loop_sorting(preview))
        for _ in range(50):
            if main.state.task_execution.get("status") == "waiting_for_selection":
                break
            await asyncio.sleep(0.01)
        assert main.state.task_execution["status"] == "waiting_for_selection"
        event = main.task_selection_events["test-run"]
        main.task_selection_choices["test-run"] = "b1"
        event.set()
        await task

    asyncio.run(run_and_select())

    assert [plan["objects"][0]["detection_id"] for plan in executed] == ["b1"]
    assert main.state.task_execution["status"] == "completed"


def test_closed_loop_planning_failure_sets_failed_status(monkeypatch):
    executed: list[dict] = []
    preview = configure_task_runtime(monkeypatch, {"execution_strategy": "closed_loop", "max_objects": 1})
    install_closed_loop_stubs(monkeypatch, [[detection("green", -120.0, 150.0, "g1")]], executed)
    monkeypatch.setattr(
        main,
        "color_profiles",
        lambda config: {"green": {"enabled": True, "drop_zone": "missing_zone"}},
    )

    asyncio.run(main.execute_closed_loop_sorting(preview))

    assert executed == []
    assert main.state.task_execution["status"] == "failed"
    assert "missing drop zone" in main.state.task_execution["terminal_reason"]


def test_closed_loop_safety_gate_loss_stops_before_capture(monkeypatch):
    executed: list[dict] = []
    preview = configure_task_runtime(monkeypatch, {"execution_strategy": "closed_loop", "max_objects": 1})
    counters = install_closed_loop_stubs(monkeypatch, [[detection("red")]], executed)
    monkeypatch.setattr(main, "task_motion_gate_reason", lambda: "safety gate lost")

    asyncio.run(main.execute_closed_loop_sorting(preview))

    assert counters["captures"] == 0
    assert executed == []
    assert main.state.task_execution["status"] == "failed"
    assert main.state.task_execution["terminal_reason"] == "safety gate lost"


def test_task_stop_before_grip_reports_no_object_held(monkeypatch):
    configure_task_runtime(monkeypatch, {"execution_strategy": "closed_loop", "max_objects": 1})

    result = asyncio.run(main.stop_task())

    assert result["ok"] is True
    assert main.state.task_execution["status"] == "stopped"
    assert main.state.task_execution["object_hold_state"] == "none"
    assert main.state.task_execution["holding_uncertain"] is False


def test_cancelled_task_cannot_overwrite_stopped_status_with_completed(monkeypatch):
    preview = configure_task_runtime(monkeypatch, {"execution_strategy": "batch_once", "max_objects": 1})
    sequence = {
        "steps": [
            {
                "kind": "move",
                "label": "blocking move",
                "object_index": 1,
                "waypoint": {"type": "joint", "mode": "joint", "angles_deg": [0.0, 35.0, 15.0, 0.0]},
            }
        ]
    }

    async def fake_broadcast():
        return None

    def fake_build_preview(**kwargs):
        return {"ok": True, "preview": {"trajectory": {"mode": "program"}}}

    async def blocking_execute(_preview):
        await asyncio.sleep(60)

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "build_preview", fake_build_preview)
    monkeypatch.setattr(main, "execute_waypoint_path", blocking_execute)

    async def run_and_stop():
        main.task_task = asyncio.create_task(
            main.execute_task_sequence(sequence, preview["settings"], preview["branch"])
        )
        for _ in range(50):
            if main.state.task_execution.get("status") == "executing":
                break
            await asyncio.sleep(0.01)
        result = await main.stop_task()
        assert result["ok"] is True

    asyncio.run(run_and_stop())

    assert main.state.task_execution["status"] == "stopped"
    assert main.state.task_execution["terminal_reason"] == "STOP"
    assert main.state.task_execution["holding_uncertain"] is False


@pytest.mark.parametrize(
    ("phase", "steps", "blocked_action", "expected_hold", "last_completed"),
    [
        (
            "pickup_approach",
            [
                {
                    "kind": "move",
                    "label": "above pickup",
                    "phase": "pickup_approach",
                    "safe_retreat_available": True,
                    "recovery_target": {"waypoint": {"type": "joint", "angles_deg": [0, 35, 15, 0]}},
                    "waypoint": {"type": "joint", "mode": "joint", "angles_deg": [0, 35, 15, 0]},
                }
            ],
            None,
            "none",
            None,
        ),
        (
            "grip",
            [
                {
                    "kind": "tool",
                    "label": "close gripper",
                    "phase": "grip",
                    "action": "close",
                    "hold_transition": "possibly_held",
                    "safe_retreat_available": True,
                    "recovery_target": {"waypoint": {"type": "joint", "angles_deg": [0, 35, 15, 0]}},
                }
            ],
            "close",
            "possibly_held",
            None,
        ),
        (
            "transfer",
            [
                {
                    "kind": "tool",
                    "label": "close gripper",
                    "phase": "grip",
                    "action": "close",
                    "hold_transition": "possibly_held",
                },
                {
                    "kind": "move",
                    "label": "above dropoff",
                    "phase": "transfer",
                    "safe_retreat_available": True,
                    "recovery_target": {"waypoint": {"type": "joint", "angles_deg": [0, 35, 15, 0]}},
                    "waypoint": {"type": "joint", "mode": "joint", "angles_deg": [0, 35, 15, 0]},
                },
            ],
            None,
            "possibly_held",
            "close gripper",
        ),
        (
            "release",
            [
                {
                    "kind": "tool",
                    "label": "close gripper",
                    "phase": "grip",
                    "action": "close",
                    "hold_transition": "possibly_held",
                },
                {
                    "kind": "tool",
                    "label": "open gripper",
                    "phase": "release",
                    "action": "open",
                    "hold_transition": "release_unconfirmed",
                    "safe_retreat_available": True,
                    "recovery_target": {"waypoint": {"type": "joint", "angles_deg": [0, 35, 15, 0]}},
                },
            ],
            "open",
            "release_unconfirmed",
            "close gripper",
        ),
        (
            "drop_retreat",
            [
                {
                    "kind": "tool",
                    "label": "close gripper",
                    "phase": "grip",
                    "action": "close",
                    "hold_transition": "possibly_held",
                },
                {
                    "kind": "tool",
                    "label": "open gripper",
                    "phase": "release",
                    "action": "open",
                    "hold_transition": "release_unconfirmed",
                },
                {
                    "kind": "move",
                    "label": "lift from dropoff",
                    "phase": "drop_retreat",
                    "safe_retreat_available": True,
                    "recovery_target": {"waypoint": {"type": "joint", "angles_deg": [0, 35, 15, 0]}},
                    "waypoint": {"type": "joint", "mode": "joint", "angles_deg": [0, 35, 15, 0]},
                },
            ],
            None,
            "release_unconfirmed",
            "open gripper",
        ),
    ],
)
def test_abort_reports_phase_hold_state_last_step_and_recovery(
    monkeypatch,
    phase,
    steps,
    blocked_action,
    expected_hold,
    last_completed,
):
    preview = configure_task_runtime(monkeypatch, {"execution_strategy": "batch_once", "max_objects": 1})
    main.state.known_pose = True

    async def fake_broadcast():
        return None

    async def fake_tool_action(action, value=None):
        if action == blocked_action:
            await asyncio.sleep(60)
        return {"ok": True}

    def fake_build_preview(**kwargs):
        return {"ok": True, "preview": {"trajectory": {"mode": "program"}}}

    async def blocking_execute(_preview):
        await asyncio.sleep(60)

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "apply_tool_action", fake_tool_action)
    monkeypatch.setattr(main, "build_preview", fake_build_preview)
    monkeypatch.setattr(main, "execute_waypoint_path", blocking_execute)

    async def run_and_stop():
        main.task_task = asyncio.create_task(
            main.execute_task_sequence({"steps": steps}, preview["settings"], preview["branch"])
        )
        for _ in range(100):
            current = (main.state.task_execution.get("current_step") or {}).get("phase")
            if current == phase:
                break
            await asyncio.sleep(0.01)
        assert (main.state.task_execution.get("current_step") or {}).get("phase") == phase
        await main.stop_task()

    asyncio.run(run_and_stop())

    execution = main.state.task_execution
    assert execution["status"] == "stopped"
    assert execution["current_step"]["phase"] == phase
    assert execution["object_hold_state"] == expected_hold
    assert execution["holding_uncertain"] is (expected_hold != "none")
    assert execution["safe_retreat_available"] is True
    assert execution["recovery_options"]
    if last_completed is None:
        assert execution["last_completed_step"] is None
    else:
        assert execution["last_completed_step"]["label"] == last_completed


def test_task_motion_fault_is_reported_as_failed(monkeypatch):
    preview = configure_task_runtime(monkeypatch, {"execution_strategy": "batch_once", "max_objects": 1})
    sequence = {
        "steps": [
            {
                "kind": "move",
                "label": "failing move",
                "object_index": 1,
                "waypoint": {"type": "joint", "mode": "joint", "angles_deg": [0.0, 35.0, 15.0, 0.0]},
            }
        ]
    }

    async def fake_broadcast():
        return None

    def fake_build_preview(**kwargs):
        return {"ok": True, "preview": {"trajectory": {"mode": "program"}}}

    async def failing_execute(_preview):
        main.state.motion_state = MotionState.FAULT
        main.state.motion_execution_state = "failed"
        main.state.last_error = "simulated motion failure"

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "build_preview", fake_build_preview)
    monkeypatch.setattr(main, "execute_waypoint_path", failing_execute)

    result = asyncio.run(main.execute_task_sequence(sequence, preview["settings"], preview["branch"]))

    assert result["ok"] is False
    assert main.state.task_execution["status"] == "failed"
    assert main.state.task_execution["terminal_reason"] == "simulated motion failure"


def test_unexpected_task_step_error_is_reported_with_step_context(monkeypatch):
    preview = configure_task_runtime(monkeypatch, {"execution_strategy": "batch_once", "max_objects": 1})
    sequence = {
        "steps": [
            {
                "kind": "tool",
                "label": "open gripper",
                "phase": "tool_prepare",
                "action": "open",
            }
        ]
    }

    async def fake_broadcast():
        return None

    async def failing_tool_action(_action, _value=None):
        raise RuntimeError("controller response could not be parsed")

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "apply_tool_action", failing_tool_action)

    result = asyncio.run(main.execute_task_sequence(sequence, preview["settings"], preview["branch"]))

    assert result["ok"] is False
    assert main.state.task_execution["status"] == "failed"
    assert main.state.task_execution["current_step"]["label"] == "open gripper"
    assert main.state.task_execution["terminal_reason"] == (
        "task execution error: controller response could not be parsed"
    )


def test_task_run_ui_displays_terminal_failure_reason():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")

    assert "Failed: ${terminalDetail}" in app_js
    assert 'class="task-run-failure"' in app_js
    assert "Stopped at ${escapeHtml(step.label)}" in app_js


def test_successful_batch_sequence_updates_completed_and_remaining_counts(monkeypatch):
    preview = configure_task_runtime(monkeypatch, {"execution_strategy": "batch_once", "max_objects": 2})
    main.state.task_execution["total_count"] = 2
    main.state.task_execution["remaining_count"] = 2
    sequence = {
        "steps": [
            {
                "kind": "move",
                "label": "object one",
                "object_index": 1,
                "waypoint": {"type": "joint", "mode": "joint", "angles_deg": [0.0, 35.0, 15.0, 0.0]},
            },
            {
                "kind": "move",
                "label": "object two",
                "object_index": 2,
                "waypoint": {"type": "joint", "mode": "joint", "angles_deg": [0.0, 35.0, 15.0, 0.0]},
            },
        ]
    }

    async def fake_broadcast():
        return None

    def fake_build_preview(**kwargs):
        return {"ok": True, "preview": {"trajectory": {"mode": "program"}}}

    async def successful_execute(_preview):
        main.state.motion_state = MotionState.IDLE
        main.state.motion_execution_state = "reached"

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "build_preview", fake_build_preview)
    monkeypatch.setattr(main, "execute_waypoint_path", successful_execute)

    result = asyncio.run(main.execute_task_sequence(sequence, preview["settings"], preview["branch"]))

    assert result["ok"] is True
    assert main.state.task_execution["status"] == "completed"
    assert main.state.task_execution["completed_count"] == 2
    assert main.state.task_execution["remaining_count"] == 0


def test_task_sequence_dispatches_program_substeps_instead_of_whole_program_upload(monkeypatch):
    preview = configure_task_runtime(monkeypatch, {"execution_strategy": "batch_once", "max_objects": 1})
    requested_settings = {
        **preview["settings"],
        "global_speed_deg_s": 37.5,
        "global_accel_deg_s2": 88.0,
    }
    sequence = {
        "steps": [
            {
                "kind": "move",
                "label": "above pickup",
                "object_index": 1,
                "waypoint": {"type": "cartesian", "mode": "joint", "target": {"x_mm": -120.0, "y_mm": 150.0, "z_mm": 80.0, "phi_deg": -100.0}},
            }
        ]
    }
    calls: list[dict] = []

    async def fake_broadcast():
        return None

    def fake_build_preview(**kwargs):
        return {
            "ok": True,
            "preview_id": "preview-program-step",
            "preview": {
                **main.pose_snapshot_fields(),
                "mode": "program",
                "source": "task",
                "settings": kwargs["settings"],
                "trajectory": {
                    "mode": "program",
                    "waypoints": [[0.0, 35.0, 15.0, 0.0]],
                    "duration_s": 0.1,
                    "execution_steps": [
                        {
                            "kind": "motion",
                            "settings": kwargs["settings"],
                            "trajectory": {"mode": "joint", "waypoints": [[0.0, 35.0, 15.0, 0.0]], "duration_s": 0.1},
                        }
                    ],
                },
                "motion_contract": {},
            },
        }

    async def fake_execute_program(preview_arg):
        calls.append(preview_arg)
        main.state.motion_state = MotionState.IDLE
        main.state.motion_execution_state = "reached"

    async def fail_if_whole_program_upload(_preview):
        raise AssertionError("task step should execute through program substeps")

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "build_preview", fake_build_preview)
    monkeypatch.setattr(main, "execute_program_sequence", fake_execute_program)
    monkeypatch.setattr(main, "execute_waypoint_path", fail_if_whole_program_upload)

    result = asyncio.run(main.execute_task_sequence(sequence, requested_settings, preview["branch"]))

    assert result["ok"] is True
    assert len(calls) == 1
    execution_step = calls[0]["trajectory"]["execution_steps"][0]
    assert execution_step["settings"]["global_speed_deg_s"] == 37.5
    assert execution_step["settings"]["global_accel_deg_s2"] == 88.0


def test_joint_endpoint_execution_uploads_preview_trajectory_settings(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = False
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.motion_execution_state = "idle"
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()
    captured: dict[str, float | list[float] | str] = {}

    async def fake_broadcast():
        return None

    class FakeSerial:
        is_connected = True

    def fake_send_trajectory(trajectory, speed_deg_s, accel_deg_s2):
        captured["targets"] = list(trajectory["waypoints"][-1])
        captured["point_count"] = len(trajectory["waypoints"])
        captured["speed"] = speed_deg_s
        captured["accel"] = accel_deg_s2
        main.state.last_controller_response = "OK command=TRAJ_START count=2 duration=0.250"
        return {"response": main.state.last_controller_response, "point_count": len(trajectory["waypoints"]), "duration_s": 0.25}

    async def fake_wait_for_hardware_target(*_args, **_kwargs):
        main.state.update_reported_pose(target, source="open_loop_estimate", known_pose=True)
        main.state.motion_state = MotionState.IDLE
        main.state.motion_execution_state = "reached"
        return True, "reached"

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "serial_client", FakeSerial())
    monkeypatch.setattr(main, "hardware_trajectory_start_blocking_reason", lambda _preview=None: None)
    monkeypatch.setattr(main, "send_trajectory_and_read_response", fake_send_trajectory)
    monkeypatch.setattr(main, "wait_for_hardware_target", fake_wait_for_hardware_target)

    target = [0.0, 35.0, 15.0, 0.0]
    preview = {
        "source": "task",
        "settings": {"global_speed_deg_s": 42.0, "global_accel_deg_s2": 91.0},
        "trajectory": {"mode": "joint", "waypoints": [config.home_pose.copy(), target], "duration_s": 0.25},
    }

    asyncio.run(main.execute_joint_endpoint_move(preview))

    assert captured["targets"] == target
    assert captured["point_count"] == 2
    assert captured["speed"] == 42.0
    assert captured["accel"] == 91.0
    assert main.state.motion_diagnostics["motion_contract"]["controller_command"]["command"] == "TRAJ"


def test_single_waypoint_simulation_trajectory_is_a_successful_noop(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = list(config.home_pose)
    main.state.target_angles_deg = list(config.home_pose)
    run_id = main.start_motion_diagnostics(
        source="test",
        mode="program",
        target_deg=list(config.home_pose),
        expected_duration_s=0.0,
        waypoint_count=1,
    )

    asyncio.run(
        main.execute_simulated_waypoint_trajectory(
            {
                "waypoints": [list(config.home_pose)],
                "segment_durations_s": [0.0],
                "time_from_start_s": [0.0],
            },
            run_id,
        )
    )

    assert main.state.motion_state == MotionState.IDLE
    assert main.state.motion_execution_state == "reached"
    assert main.state.last_error == ""


def test_simulation_vision_queue_peeks_then_closed_loop_capture_consumes(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    client = TestClient(main.app)
    frames = [
        {"detections": [detection("red", detection_id="r1")]},
        {"detections": [detection("blue", detection_id="b1")]},
    ]

    queued = client.post("/api/simulation/vision/queue", json={"frames": frames}).json()
    peek = client.get("/api/vision/frame").json()
    status_after_peek = client.get("/api/simulation/vision/queue").json()
    consumed = asyncio.run(main.closed_loop_capture())
    status_after_consume = client.get("/api/simulation/vision/queue").json()

    assert queued["ok"] is True
    assert peek["detections"][0]["id"] == "r1"
    assert status_after_peek["queue"]["remaining_frames"] == 2
    assert consumed["detections"][0]["id"] == "r1"
    assert status_after_consume["queue"]["remaining_frames"] == 1


def test_simulation_vision_queue_never_falls_back_to_camera(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.simulation_vision_queue.clear()
    monkeypatch.setattr(main, "capture_camera_frame", lambda camera: pytest.fail("camera must not be used"))

    with pytest.raises(RuntimeError, match="simulation vision queue is empty"):
        asyncio.run(main.closed_loop_capture())


def test_closed_loop_queue_exhaustion_fails_explicitly_after_completed_cycle(monkeypatch):
    preview = configure_task_runtime(
        monkeypatch,
        {"execution_strategy": "closed_loop", "max_objects": 2},
    )
    main.simulation_vision_queue[:] = [
        {"detections": [detection("red", detection_id="r1")]},
    ]
    executed: list[dict] = []

    async def fake_broadcast():
        return None

    async def fake_move_named_position(name, settings, branch, label):
        return {"ok": True}

    async def fake_execute_sequence(sequence, settings, branch, *, terminal_on_finish=True):
        executed.append(sequence)
        return {"ok": True}

    def fake_build_preview(**kwargs):
        return {"ok": True, "preview": {"trajectory": {"mode": "program"}}}

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "move_task_named_position", fake_move_named_position)
    monkeypatch.setattr(main, "execute_task_sequence", fake_execute_sequence)
    monkeypatch.setattr(main, "build_preview", fake_build_preview)
    monkeypatch.setattr(main, "task_motion_gate_reason", lambda: None)

    asyncio.run(main.execute_closed_loop_sorting(preview))

    assert len(executed) == 1
    assert main.state.task_execution["completed_count"] == 1
    assert main.state.task_execution["status"] == "failed"
    assert "simulation vision queue is empty" in main.state.task_execution["terminal_reason"]


def test_simulation_vision_queue_is_rejected_in_hardware_mode(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = False
    client = TestClient(main.app)

    payload = client.post(
        "/api/simulation/vision/queue",
        json={"frames": [{"detections": []}]},
    ).json()

    assert payload["ok"] is False
    assert "only in simulation" in payload["error"]


def test_task_preview_requires_detections_and_valid_motion_settings(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    client = TestClient(main.app)
    valid_settings = {
        "global_speed_deg_s": 25,
        "global_accel_deg_s2": 120,
        "waypoint_rate_hz": 12,
        "cartesian_step_mm": 10,
        "planner_type": "s_curve",
        "jerk_percent": 25,
        "blend_percent": 0,
        "per_joint_speed_deg_s": [45, 35, 60, 80],
        "per_joint_accel_deg_s2": [12, 12, 18, 22],
    }

    missing = client.post(
        "/api/task/preview",
        json={"task": "color_sorting", "detections": [], "settings": valid_settings},
    ).json()
    invalid_motion = client.post(
        "/api/task/preview",
        json={
            "task": "color_sorting",
            "detections": [detection("red")],
            "settings": {**valid_settings, "planner_type": "magic"},
        },
    ).json()

    assert missing["ok"] is False
    assert "refresh detections" in missing["error"]
    assert invalid_motion["ok"] is False
    assert "planner_type" in invalid_motion["error"]


def test_task_preview_carries_program_motion_contract(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()
    client = TestClient(main.app)

    payload = client.post(
        "/api/task/preview",
        json={
            "task": "color_sorting",
            "detections": [detection("red")],
            "settings": {
                "global_speed_deg_s": 25,
                "global_accel_deg_s2": 120,
                "waypoint_rate_hz": 12,
                "cartesian_step_mm": 10,
                "planner_type": "s_curve",
                "per_joint_speed_deg_s": [45, 35, 60, 80],
                "per_joint_accel_deg_s2": [12, 12, 18, 22],
            },
        },
    ).json()

    assert payload["ok"], payload
    contract = payload["preview"]["motion_contract"]
    assert contract["path_mode"] == "program"
    assert contract["limits"]["path_mode"] == "program"
    assert contract["limits"]["segment_limits"]
    assert contract["limits"]["limiting_constraint"]["type"] in {"speed", "acceleration", "waypoint_rate"}


def test_task_preview_skips_unreachable_first_closed_loop_candidate(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()
    failed_detection_ids: set[str] = set()
    successful_detection_ids: list[str] = []

    def fake_build_preview(**kwargs):
        waypoint_program = kwargs.get("waypoint_program") or []
        detection_ids = [
            str(waypoint.get("detection_id"))
            for waypoint in waypoint_program
            if isinstance(waypoint, dict) and waypoint.get("detection_id") is not None
        ]
        if detection_ids and not failed_detection_ids:
            failed_detection_ids.add(detection_ids[0])
            return {
                "ok": False,
                "error": "program waypoint 1 has no valid IK solution",
                "trajectory": {
                    "errors": ["no valid IK solution"],
                    "step_results": [
                        {
                            "index": 0,
                            "label": "above pickup",
                            "status": "invalid",
                            "errors": ["no valid IK solution"],
                        }
                    ],
                },
            }
        successful_detection_ids[:] = sorted(set(detection_ids))
        return {
            "ok": True,
            "preview_id": "preview-after-skip",
            "preview": {
                **main.pose_snapshot_fields(),
                "mode": "program",
                "source": "task",
                "settings": kwargs.get("settings") or {},
                "trajectory": {
                    "mode": "program",
                    "duration_s": 0.2,
                    "waypoints": [config.home_pose.copy()],
                    "execution_steps": [],
                },
                "motion_contract": {},
                "limit_summary": {},
            },
        }

    monkeypatch.setattr(main, "build_preview", fake_build_preview)
    client = TestClient(main.app)
    payload = client.post(
        "/api/task/preview",
        json={
            "task": "color_sorting",
            "detections": [
                detection("red", -120.0, 150.0, "r1"),
                detection("blue", 120.0, 180.0, "b1"),
            ],
            "task_settings": {"execution_strategy": "closed_loop", "max_objects": 2},
        },
    ).json()

    assert payload["ok"], payload
    assert len(failed_detection_ids) == 1
    skipped_id = next(iter(failed_detection_ids))
    selected_ids = [item["detection_id"] for item in payload["sequence"]["objects"]]
    assert selected_ids == successful_detection_ids
    assert skipped_id not in selected_ids
    assert any(item.get("reason_code") == "ik_unreachable" and item.get("detection_id") == skipped_id for item in payload["task_preview"]["ignored_detections"])
    assert any(skipped_id in warning for warning in payload["task_preview"]["warnings"])


def test_task_preview_binds_detection_pose_model_settings_and_destinations(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()
    client = TestClient(main.app)
    client.post(
        "/api/simulation/vision/queue",
        json={"frames": [{"detections": [detection("red", detection_id="r1")]}]},
    )
    frame = client.get("/api/vision/frame").json()

    payload = client.post(
        "/api/task/preview",
        json={
            "task": "color_sorting",
            "detections": frame["detections"],
            "detection_snapshot_id": frame["detection_snapshot_id"],
            "detection_captured_at": frame["captured_at"],
            "task_settings": {"execution_strategy": "closed_loop", "max_objects": 1},
        },
    ).json()

    assert payload["ok"], payload
    bindings = payload["task_preview"]["bindings"]
    assert bindings["detection_snapshot_id"] == frame["detection_snapshot_id"]
    assert bindings["detection_captured_at"] == frame["captured_at"]
    assert bindings["pose_revision"] == main.state.pose_revision
    assert bindings["config_id"] == main.RUNNING_CONFIG_ID
    assert bindings["model_fingerprint"] == main.robot_model_fingerprint()
    assert bindings["task_settings_revision"]
    assert bindings["destination_revision"] == main.task_mapping_fingerprint()


def test_task_preview_accepts_browser_roundtrip_detection_metadata(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()
    client = TestClient(main.app)
    client.post(
        "/api/simulation/vision/queue",
        json={"frames": [{"detections": [detection("red", detection_id="r1")]}]},
    )
    frame = client.get("/api/vision/frame").json()
    browser_detections = deepcopy(frame["detections"])
    browser_detections[0]["timestamp"] = "browser-local-render-time"
    browser_detections[0]["ui_debug"] = {"row_visible": True}

    payload = client.post(
        "/api/task/preview",
        json={
            "task": "color_sorting",
            "detections": browser_detections,
            "detection_snapshot_id": frame["detection_snapshot_id"],
            "detection_captured_at": frame["captured_at"],
            "task_settings": {"execution_strategy": "closed_loop", "max_objects": 1},
        },
    ).json()

    assert payload["ok"], payload
    bindings = payload["task_preview"]["bindings"]
    assert bindings["detection_snapshot_id"] == frame["detection_snapshot_id"]
    assert bindings["detection_task_fingerprint"] == frame["detection_task_fingerprint"]
    assert bindings["detection_fingerprint"] != frame["detection_fingerprint"]


def test_task_preview_rejects_same_snapshot_id_with_changed_task_content(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()
    client = TestClient(main.app)
    client.post(
        "/api/simulation/vision/queue",
        json={"frames": [{"detections": [detection("red", detection_id="r1")]}]},
    )
    frame = client.get("/api/vision/frame").json()
    changed_detections = deepcopy(frame["detections"])
    changed_detections[0]["robot"]["x_mm"] += 50.0

    payload = client.post(
        "/api/task/preview",
        json={
            "task": "color_sorting",
            "detections": changed_detections,
            "detection_snapshot_id": frame["detection_snapshot_id"],
            "detection_captured_at": frame["captured_at"],
            "task_settings": {"execution_strategy": "closed_loop", "max_objects": 1},
        },
    ).json()

    assert payload["ok"] is False
    assert "detection snapshot contents do not match" in payload["error"]


def test_task_preview_uses_live_destination_mapping_without_explicit_save(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()
    client = TestClient(main.app)

    payload = client.post(
        "/api/task/preview",
        json={
            "task": "color_sorting",
            "detections": [detection("red", 120.0, 150.0, detection_id="r1")],
            "task_settings": {
                "execution_strategy": "batch_once",
                "color_profile_overrides": {
                    "red": {"enabled": True, "drop_zone": "dropoff_b"},
                },
                "task_destination_overrides": {
                    "schema_version": 1,
                    "destinations": {
                        "dropoff_b": {"position_id": "dropoff_b"},
                    },
                },
            },
        },
    ).json()

    assert payload["ok"], payload
    assert payload["sequence"]["objects"][0]["drop_zone"] == "dropoff_b"


def test_task_execute_rejects_preview_after_detection_snapshot_changes(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()
    client = TestClient(main.app)
    client.post(
        "/api/simulation/vision/queue",
        json={"frames": [{"detections": [detection("red", detection_id="r1")]}]},
    )
    frame = client.get("/api/vision/frame").json()
    preview = client.post(
        "/api/task/preview",
        json={
            "task": "color_sorting",
            "detections": frame["detections"],
            "detection_snapshot_id": frame["detection_snapshot_id"],
            "detection_captured_at": frame["captured_at"],
            "task_settings": {"execution_strategy": "closed_loop", "max_objects": 1},
        },
    ).json()
    assert preview["ok"], preview

    client.post(
        "/api/simulation/vision/queue",
        json={"frames": [{"detections": [detection("blue", detection_id="b1")]}]},
    )
    client.get("/api/vision/frame")
    payload = client.post("/api/task/execute", json={"preview_id": preview["preview_id"]}).json()

    assert payload["ok"] is False
    assert "detection snapshot changed" in payload["error"]


def test_task_execute_rejects_modified_task_settings_contract(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()
    client = TestClient(main.app)
    preview = client.post(
        "/api/task/preview",
        json={
            "task": "color_sorting",
            "detections": [detection("red", detection_id="r1")],
            "task_settings": {"execution_strategy": "batch_once", "max_objects": 1},
        },
    ).json()
    assert preview["ok"], preview
    main.task_previews[preview["preview_id"]]["task_settings"]["pickup_z_mm"] += 1.0

    payload = client.post("/api/task/execute", json={"preview_id": preview["preview_id"]}).json()

    assert payload["ok"] is False
    assert "task settings changed" in payload["error"]


def test_closed_loop_task_execute_recovers_from_previous_stopped_state(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.motion_execution_state = "idle"
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()

    async def fake_broadcast():
        return None

    observed_start_states: list[MotionState] = []

    async def fake_execute_closed_loop(_preview):
        observed_start_states.append(main.state.motion_state)
        main.finish_task_execution("completed", "test complete")

    monkeypatch.setattr(main, "broadcast_state", fake_broadcast)
    monkeypatch.setattr(main, "execute_closed_loop_sorting", fake_execute_closed_loop)

    preview = asyncio.run(
        main.preview_task(
            main.TaskPreviewRequest(
                task="color_sorting",
                detections=[detection("red", detection_id="r1")],
                task_settings={"execution_strategy": "closed_loop", "max_objects": 1},
                settings={},
                branch="auto",
            )
        )
    )
    assert preview["ok"], preview

    main.simulation_vision_queue[:] = [{"detections": [detection("red", detection_id="r1")]}]
    main.state.motion_state = MotionState.STOPPED
    main.state.last_error = "previous stop"

    async def run_task():
        result = await main.execute_task(main.TaskExecuteRequest(preview_id=preview["preview_id"]))
        assert result["ok"], result
        assert main.task_task is not None
        await main.task_task

    asyncio.run(run_task())

    assert observed_start_states == [MotionState.IDLE]
    assert main.state.last_error == ""


def test_closed_loop_camera_clear_recovers_stopped_state_refreshed_after_task_start(monkeypatch):
    executed: list[dict] = []
    preview = configure_task_runtime(
        monkeypatch,
        {"execution_strategy": "closed_loop", "max_objects": 1},
    )
    counters = install_closed_loop_stubs(
        monkeypatch,
        [[detection("red", detection_id="r1")]],
        executed,
    )
    observed_camera_clear_states: list[MotionState] = []

    def stale_post_arm_gate():
        main.state.motion_state = MotionState.STOPPED
        return None

    async def observe_camera_clear(name, settings, branch, label):
        counters["camera_clear_moves"] += 1
        observed_camera_clear_states.append(main.state.motion_state)
        return {"ok": True}

    monkeypatch.setattr(main, "task_motion_gate_reason", stale_post_arm_gate)
    monkeypatch.setattr(main, "move_task_named_position", observe_camera_clear)

    asyncio.run(main.execute_closed_loop_sorting(preview))

    assert observed_camera_clear_states == [MotionState.IDLE]
    assert counters["captures"] == 1
    assert len(executed) == 1
    assert main.state.task_execution["status"] == "completed"


def test_fresh_task_stopped_recovery_does_not_clear_stop_after_progress(monkeypatch):
    configure_task_runtime(
        monkeypatch,
        {"execution_strategy": "closed_loop", "max_objects": 2},
    )
    main.state.task_execution["completed_count"] = 1
    main.state.task_execution["last_completed_step"] = {
        "label": "release",
        "completed_at": 1.0,
    }
    main.state.motion_state = MotionState.STOPPED

    resumed = main.resume_fresh_task_motion_from_stopped()

    assert resumed is False
    assert main.state.motion_state == MotionState.STOPPED


def test_task_execute_rejects_preview_after_destination_mapping_changes(monkeypatch):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = config.home_pose.copy()
    main.state.target_angles_deg = config.home_pose.copy()
    client = TestClient(main.app)
    preview = client.post(
        "/api/task/preview",
        json={
            "task": "color_sorting",
            "detections": [detection("red", detection_id="r1")],
            "task_settings": {"execution_strategy": "batch_once", "max_objects": 1},
        },
    ).json()
    assert preview["ok"], preview
    changed_raw = deepcopy(config.raw)
    changed_raw["color_profiles"]["red"]["drop_zone"] = "dropoff_b"
    monkeypatch.setattr(main, "config", replace(config, raw=changed_raw))

    payload = client.post("/api/task/execute", json={"preview_id": preview["preview_id"]}).json()

    assert payload["ok"] is False
    assert "destinations or color mappings changed" in payload["error"]


@pytest.mark.parametrize(
    ("preview_patch", "message"),
    [
        ({"consumed": True}, "already been executed"),
        ({"created_at": 0.0}, "expired"),
        ({"config_id": "different"}, "configuration changed"),
    ],
)
def test_task_execute_rejects_stale_reused_or_mismatched_preview(monkeypatch, preview_patch, message):
    config = load_config(EXAMPLE_CONFIG_PATH)
    monkeypatch.setattr(main, "config", config)
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = MotionState.IDLE
    preview_id = "preview-freshness"
    main.task_previews[preview_id] = {
        "id": preview_id,
        "created_at": main.time(),
        "config_id": main.RUNNING_CONFIG_ID,
        "consumed": False,
        **preview_patch,
    }
    client = TestClient(main.app)

    payload = client.post("/api/task/execute", json={"preview_id": preview_id}).json()

    assert payload["ok"] is False
    assert message in payload["error"]
