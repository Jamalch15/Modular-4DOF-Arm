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


def test_frontend_contains_position_library_hooks():
    app_js = (main.STATIC_DIR / "app.js").read_text(encoding="utf-8")
    index_html = (main.STATIC_DIR / "index.html").read_text(encoding="utf-8")

    assert "Position Library" in index_html
    assert "positionLibraryList" in app_js
    assert "addJointPositionBtn" in app_js
    assert "addCartesianPositionBtn" in app_js
    assert "data-position-duplicate" in app_js
    assert "/api/position-library" in app_js
    assert "Task destinations" in index_html
    assert "taskDestinationEditor" in app_js
    assert "positionReferenceOptionsHtml" in app_js
    assert "/api/task-mappings" in app_js
    assert "Color-to-destination defaults" not in index_html
