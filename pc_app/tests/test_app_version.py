import pytest
from fastapi.testclient import TestClient

import app.main as main


@pytest.fixture(autouse=True)
def stable_git_state(monkeypatch):
    monkeypatch.setattr(main, "_git_revision", lambda: dict(main.RUNNING_GIT))
    monkeypatch.setattr(main, "_origin_main_revision", lambda: main.RUNNING_GIT["commit"])


def test_version_endpoint_reports_independent_frontend_and_backend_builds():
    client = TestClient(main.app)

    payload = client.get("/api/version").json()

    assert payload["ok"]
    assert payload["frontend_build_id"] == main.frontend_fingerprint()
    assert payload["running_backend_build_id"] == main.RUNNING_BACKEND_BUILD_ID
    assert payload["disk_backend_build_id"] == main.backend_fingerprint()
    assert payload["backend_restart_required"] is False
    assert payload["config_reload_required"] is False
    assert payload["started_at"]


def test_frontend_change_requires_browser_refresh_not_server_restart(monkeypatch):
    client = TestClient(main.app)
    monkeypatch.setattr(main, "frontend_fingerprint", lambda: "newfrontend1")
    monkeypatch.setattr(main, "backend_fingerprint", lambda: main.RUNNING_BACKEND_BUILD_ID)

    payload = client.get("/api/version").json()

    assert payload["frontend_build_id"] == "newfrontend1"
    assert payload["backend_restart_required"] is False
    assert payload["restart_required"] is False
    assert payload["running_build_id"] == "newfrontend1"


def test_backend_change_requires_server_restart(monkeypatch):
    client = TestClient(main.app)
    monkeypatch.setattr(main, "backend_fingerprint", lambda: "newbackend12")

    payload = client.get("/api/version").json()

    assert payload["running_backend_build_id"] == main.RUNNING_BACKEND_BUILD_ID
    assert payload["disk_backend_build_id"] == "newbackend12"
    assert payload["backend_restart_required"] is True
    assert payload["restart_required"] is True
    assert "backend Python files changed" in payload["reasons"][0]


def test_version_endpoint_reports_config_reload_separately(monkeypatch):
    client = TestClient(main.app)
    monkeypatch.setattr(main, "RUNNING_CONFIG_ID", "loadedconfig1")
    monkeypatch.setattr(main, "config_fingerprint", lambda: "diskconfig22")

    payload = client.get("/api/version").json()

    assert payload["backend_restart_required"] is False
    assert payload["config_reload_required"] is True
    assert payload["running_config_id"] == "loadedconfig1"
    assert payload["disk_config_id"] == "diskconfig22"
    assert "configuration file changed" in payload["reasons"][0]


def test_remote_difference_does_not_make_localhost_stale(monkeypatch):
    client = TestClient(main.app)
    monkeypatch.setattr(main, "_origin_main_revision", lambda: "abcdef123456")

    payload = client.get("/api/version").json()

    assert payload["backend_restart_required"] is False
    assert payload["remote_differs"] is True
    assert payload["pull_required"] is True
    assert payload["origin_main_commit"] == "abcdef123456"


def test_checkout_change_without_backend_change_does_not_require_restart(monkeypatch):
    client = TestClient(main.app)
    monkeypatch.setattr(
        main,
        "_git_revision",
        lambda: {"commit": "123456abcdef", "dirty": False},
    )
    monkeypatch.setattr(main, "_origin_main_revision", lambda: "123456abcdef")
    monkeypatch.setattr(main, "backend_fingerprint", lambda: main.RUNNING_BACKEND_BUILD_ID)

    payload = client.get("/api/version").json()

    assert payload["checkout_changed_since_start"] is True
    assert payload["backend_restart_required"] is False


def test_index_injects_current_frontend_build_and_disables_cache(monkeypatch):
    client = TestClient(main.app)
    monkeypatch.setattr(main, "frontend_fingerprint", lambda: "currentfront")

    response = client.get("/")

    assert response.status_code == 200
    assert "__APP_BUILD_ID__" not in response.text
    assert 'name="app-build-id" content="currentfront"' in response.text
    assert "/static/app.js?v=currentfront" in response.text
    assert "/static/styles.css?v=currentfront" in response.text
    assert "no-store" in response.headers["cache-control"]


def test_app_js_uses_page_build_id_for_robot_view_cache_busting():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")

    assert "20260618-workspace-projection" not in app_js
    assert "robot_view.js?v=${encodeURIComponent(PAGE_BUILD_ID)}" in app_js


def test_app_js_uses_independent_frontend_and_backend_version_fields():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")

    assert "version?.frontend_build_id || version?.running_build_id" in app_js
    assert "version?.running_backend_build_id || version?.running_build_id" in app_js
    assert "version?.disk_backend_build_id || version?.disk_build_id" in app_js
    assert "version?.backend_restart_required ?? version?.restart_required" in app_js
    assert "Backend outdated - restart localhost" in app_js


def test_config_endpoint_exposes_model_truth_summary():
    client = TestClient(main.app)

    payload = client.get("/api/config").json()

    assert "model_truth" in payload
    assert payload["model_truth"]["transform_chain"][0]["id"] == "actuator"
    assert payload["model_truth"]["active_tool"]["tcp_axis_mapping"]["tool_z"] == "local DH +X / tool-forward"


def test_frontend_contains_model_truth_and_tcp_frame_hooks():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")
    robot_view_js = (main.STATIC_DIR / "robot_view.js").read_text(encoding="utf-8")
    index_html = (main.STATIC_DIR / "index.html").read_text(encoding="utf-8")

    assert "renderModelTruthSummary" in app_js
    assert "Tool/TCP dimensions" in app_js
    assert "settingsModelTruth" in index_html
    assert "makeTcpFrameAxes" in robot_view_js
    assert "currentTcpAxisZ" in robot_view_js


def test_frontend_places_encoder_controls_in_hardware_settings():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")
    index_html = (main.STATIC_DIR / "index.html").read_text(encoding="utf-8")
    joint_section = index_html.split('id="settingsJoints"', 1)[1].split('id="settingsMotion"', 1)[0]
    hardware_section = index_html.split('id="settingsHardware"', 1)[1].split("</section>", 1)[0]

    assert 'id="encoderCalibration"' not in joint_section
    assert 'id="encoderCalibration"' in hardware_section
    assert "Shoulder encoder hardware" in hardware_section
    assert "Enable encoder readback" in app_js
    assert "Use shoulder AS5048A" in app_js
    assert "Quick shoulder encoder setup" in app_js
    assert "Quick calibrate" in app_js
    assert "Run backlash check" in app_js
    assert "Run assisted sweep" in app_js
    assert "encoderActionStatus" in app_js
    assert "Start + capture reference 1" in app_js
    assert "Move shoulder to angle" in app_js
    assert "data-encoder-correction-field" in app_js
    assert "settings.correction.enabled = false;" in app_js
    assert "Encoder hardware, calibration, or policy settings changed" in app_js
    assert "Fault threshold only controls when mismatch becomes a fault" in app_js
    assert "Correction max delta deg (movement cap)" in app_js
    assert "Measured shoulder live" in app_js
    assert "Raw shoulder sensor" in app_js
    assert "Correction gate" in app_js
    assert "Latest shoulder encoder readback" in app_js
    sweep_handler = app_js.split("async function runAssistedEncoderSweep()", 1)[1].split(
        "async function cancelAssistedEncoderSweep()",
        1,
    )[0]
    assert "window.confirm" not in sweep_handler
    assert 'postJson("/api/encoder/calibration/sweep/start"' in sweep_handler


def test_frontend_contains_position_library_hooks():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")
    index_html = (main.STATIC_DIR / "index.html").read_text(encoding="utf-8")

    assert "Position Library" in index_html
    assert "positionLibraryList" in app_js
    assert "addJointPositionBtn" in app_js
    assert "addCartesianPositionBtn" in app_js
    assert "data-position-duplicate" in app_js
    assert "/api/position-library" in app_js
    assert "Tasks select these positions directly" in index_html
    assert "availableTaskDestinations" in app_js
    assert "taskDestinationsForSave" in app_js
    assert "/api/task-mappings" in app_js
    assert "Color-to-destination defaults" not in index_html
    assert 'const CORE_POSITION_IDS = new Set(["home"])' in app_js
    assert "position-library-error" in app_js


def test_frontend_contains_sweep_feedback_hooks():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")
    index_html = (main.STATIC_DIR / "index.html").read_text(encoding="utf-8")

    assert "setPoseModal" in index_html
    assert "Set Pose does not move or physically home the robot. It asserts" not in app_js
    assert "confirmCurrentPoseKnown" in app_js
    assert "Changing to ${active}..." in app_js
    assert "scheduleDiagnosticsRender" in app_js
    assert 'fetch("/api/events?limit=80")' in app_js
    assert 'postJson("/api/home", { settings: pathSettings() })' in app_js
    assert 'postJson("/api/path/go"' in app_js
    assert index_html.count("collapsible-panel") >= 2


def test_frontend_contains_program_library_workflow_and_demo_hooks():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")
    index_html = (main.STATIC_DIR / "index.html").read_text(encoding="utf-8")

    for stage in ("library", "build", "preview", "run"):
        assert f'data-program-stage="{stage}"' in index_html
        assert f'data-program-panel="{stage}"' in index_html
    assert "Built-in demos" in index_html
    assert "My programs" in index_html
    assert "Save as copy" in index_html
    assert "previewProgramStepBtn" not in index_html
    assert "Preview selected step" not in index_html
    assert "programPlaybackProgress" in index_html
    assert "programPlaybackToggle" in index_html
    assert 'option value="end_effector"' in index_html
    assert "loadProgramLibrary" in app_js
    assert 'requestJson("/api/programs")' in app_js
    assert "startProgramPlayback" in app_js
    assert "programMotionLimitFields" in app_js
    assert "copyLibraryProgram" in app_js
    assert "renderProgramRunMonitor" in app_js
    assert "Preview target" in app_js
    assert "Go to target" in app_js
    assert "previewSelectedProgramTarget" in app_js
    assert "executeSelectedProgramTarget" in app_js
    assert 'postJson("/api/path/preview"' in app_js
    assert "Save and apply manual offsets" in index_html
    assert 'enabled: true' in app_js
    execute_preview = app_js.split("async function executePreview()", 1)[1].split(
        "function activeProgramRecord()",
        1,
    )[0]
    assert "state.ikUserEdited = true;" in execute_preview
    assert "state.ikUserEdited = false;" not in execute_preview
    assert "state.latestPreview = null;" in execute_preview
    preview_ik = app_js.split("async function previewIkPath()", 1)[1].split(
        "function scheduleIkPreview(",
        1,
    )[0]
    assert "const requestedTarget = ikTargetPayload();" in preview_ik
    assert "setIkTargetFromRequestedTarget(requestedTarget);" in preview_ik
    render_state = app_js.split("function renderState(robotState)", 1)[1].split(
        "function renderPreview(",
        1,
    )[0]
    assert "const preserveRequestedIkTarget = state.ikUserEdited;" in render_state
    assert "state.ikUserEdited = preserveRequestedIkTarget;" in render_state
