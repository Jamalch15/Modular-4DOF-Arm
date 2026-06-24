from __future__ import annotations

import base64

import cv2
import numpy as np
from fastapi.testclient import TestClient
from pytest import approx

from app import main
from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.demo_settings import camera_settings, color_profiles
from app.vision import VisionPipeline, encode_image_b64, workspace_aruco_settings
from app.workspace_calibration import detect_fiducials, saved_homography


def working_camera() -> dict:
    camera = camera_settings(load_config(EXAMPLE_CONFIG_PATH))
    camera["enabled"] = True
    camera["source_index"] = 1
    camera["resolution"] = {"width": 640, "height": 480}
    camera["detection"] = {**camera.get("detection", {}), "provider": "workspace_color"}
    workspace = camera["calibration"]["workspace_aruco"]
    workspace["reference_points_px"] = {
        "0": [99.0, 88.75],
        "1": [545.25, 94.0],
        "2": [96.0, 367.0],
        "3": [541.75, 374.0],
    }
    workspace["reference_workspace_corners_px"] = {
        "0": [70.0, 59.75],
        "1": [574.25, 65.0],
        "2": [67.0, 396.0],
        "3": [570.75, 403.0],
    }
    workspace["reference_resolution"] = {"width": 640, "height": 480}
    return camera


def paste_aruco_marker(
    image: np.ndarray,
    dictionary_name: str,
    marker_id: int,
    center_px: tuple[float, float],
    size_px: int = 58,
) -> None:
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    x0 = int(round(center_px[0] - size_px / 2))
    y0 = int(round(center_px[1] - size_px / 2))
    image[y0 : y0 + size_px, x0 : x0 + size_px] = marker_bgr


def image_point_for_robot(settings: dict, robot_xy: list[float]) -> tuple[int, int]:
    homography, metrics = saved_homography(settings, (480, 640, 3))
    assert homography is not None, metrics
    robot_to_image = np.linalg.inv(homography)
    point = cv2.perspectiveTransform(
        np.asarray([[[float(robot_xy[0]), float(robot_xy[1])]]], dtype=np.float64),
        robot_to_image,
    )[0][0]
    return int(round(float(point[0]))), int(round(float(point[1])))


def test_workspace_pipeline_detects_multiple_objects_and_masks_outside_workspace():
    camera = working_camera()
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    cv2.circle(image, (260, 220), 18, (0, 0, 255), -1)
    cv2.circle(image, (380, 250), 20, (0, 0, 255), -1)
    cv2.circle(image, (20, 20), 18, (0, 0, 255), -1)

    result = VisionPipeline().process(image, camera, color_profiles(load_config(EXAMPLE_CONFIG_PATH)))

    assert result["ok"]
    assert result["workspace"]["homography_source"] == "workspace_aruco_saved"
    assert result["workspace"]["status"] == "saved_calibration"
    assert result["workspace"]["live_tags_required"] is False
    assert result["workspace"]["workspace_polygon_source"] == "saved_workspace_config"
    assert len(result["detections"]) == 2
    assert all(detection["label"] == "red" for detection in result["detections"])
    assert all(detection["task_eligible"] for detection in result["detections"])
    assert all(detection["coordinate_source"] == "workspace_aruco_saved" for detection in result["detections"])
    assert all(detection["robot"]["z_mm"] == 0.0 for detection in result["detections"])
    assert [detection["object_id"] for detection in result["detections"]] == [1, 2]
    assert all({"id", "confidence", "center_px", "bbox_px", "timestamp"} <= detection.keys() for detection in result["detections"])


def test_workspace_color_profile_detects_red_hue_wraparound_without_broad_fallback():
    camera = working_camera()
    camera["detection"] = {
        **camera["detection"],
        "show_unconfigured_colors": False,
    }
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    wrapped_red = cv2.cvtColor(
        np.asarray([[[176, 230, 235]]], dtype=np.uint8),
        cv2.COLOR_HSV2BGR,
    )[0][0]
    cv2.circle(image, (260, 220), 18, tuple(int(value) for value in wrapped_red), -1)

    result = VisionPipeline().process(image, camera, color_profiles(load_config(EXAMPLE_CONFIG_PATH)))

    assert len(result["detections"]) == 1
    detection = result["detections"][0]
    assert detection["label"] == "red"
    assert detection["task_eligible"]
    assert detection["detector"] == "workspace_color"


def test_workspace_color_uses_custom_profile_label_before_hardcoded_hue_name():
    camera = working_camera()
    camera["detection"] = {
        **camera["detection"],
        "show_unconfigured_colors": False,
    }
    profiles = {
        "fixture_orange": {
            "enabled": True,
            "hsv_min": [15, 80, 60],
            "hsv_max": [25, 255, 255],
            "min_area_px": 50,
            "drop_zone": "dropoff_a",
        }
    }
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    orange = cv2.cvtColor(
        np.asarray([[[20, 220, 235]]], dtype=np.uint8),
        cv2.COLOR_HSV2BGR,
    )[0][0]
    cv2.circle(image, (260, 220), 18, tuple(int(value) for value in orange), -1)

    result = VisionPipeline().process(image, camera, profiles)

    assert len(result["detections"]) == 1
    detection = result["detections"][0]
    assert detection["label"] == "fixture_orange"
    assert detection["task_eligible"]


def test_vision_detect_min_area_override_allows_smaller_profile_objects(monkeypatch):
    camera = working_camera()
    camera["detection"] = {
        **camera["detection"],
        "show_unconfigured_colors": False,
        "min_object_area_px": 400,
    }
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    cv2.circle(image, (260, 220), 6, (0, 0, 255), -1)
    image_b64 = encode_image_b64(image, ".png")
    profiles = color_profiles(load_config(EXAMPLE_CONFIG_PATH))
    assert profiles["red"]["min_area_px"] > 40

    monkeypatch.setattr(main, "config", load_config(EXAMPLE_CONFIG_PATH))
    monkeypatch.setattr(main, "runtime_vision_detection_overrides", {})
    monkeypatch.setattr(main.state, "simulation", False)
    monkeypatch.setattr(main, "camera_settings", lambda _config: camera)
    monkeypatch.setattr(main, "color_profiles", lambda _config: profiles)
    client = TestClient(main.app)

    default_payload = client.post(
        "/api/vision/detect",
        json={"image_b64": image_b64, "profile_names": ["red"]},
    ).json()
    smaller_payload = client.post(
        "/api/vision/detect",
        json={"image_b64": image_b64, "profile_names": ["red"], "min_object_area_px": 40},
    ).json()

    assert default_payload["ok"]
    assert default_payload["detections"] == []
    assert smaller_payload["ok"]
    assert smaller_payload["detection_tuning"]["min_object_area_px"] == 40
    assert len(smaller_payload["detections"]) == 1
    assert smaller_payload["detections"][0]["label"] == "red"


def test_workspace_margin_expands_color_detection_mask():
    camera = working_camera()
    camera["detection"] = {
        **camera["detection"],
        "min_object_area_px": 20,
    }
    outside_left_edge_px = image_point_for_robot(
        camera["calibration"]["workspace_aruco"],
        [-250.0, 250.0],
    )
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    cv2.circle(image, outside_left_edge_px, 7, (0, 0, 255), -1)

    without_margin = VisionPipeline().process(
        image,
        camera,
        {},
    )

    camera["calibration"]["workspace_aruco"]["workspace_margin_mm"] = 30.0
    with_margin = VisionPipeline().process(
        image,
        camera,
        {},
    )

    assert without_margin["detections"] == []
    assert len(with_margin["detections"]) == 1
    assert with_margin["detections"][0]["coordinate_source"] == "workspace_aruco_saved"


def test_workspace_tag_detection_falls_back_to_apriltag_dictionary():
    camera = working_camera()
    settings = camera["calibration"]["workspace_aruco"]
    settings["dictionary"] = "DICT_4X4_50"
    settings["dictionary_candidates"] = ["DICT_4X4_50", "DICT_APRILTAG_36H11"]
    image = np.full((480, 640, 3), 220, dtype=np.uint8)

    for marker_id, center in settings["reference_points_px"].items():
        paste_aruco_marker(
            image,
            "DICT_APRILTAG_36H11",
            int(marker_id),
            (float(center[0]), float(center[1])),
        )

    detection = detect_fiducials(image, settings)

    assert detection.dictionary == "DICT_APRILTAG_36H11"
    assert detection.mode == "normal"
    assert detection.visible_ids == [0, 1, 2, 3]


def test_workspace_tag_detection_handles_mirrored_camera_frames():
    camera = working_camera()
    settings = workspace_aruco_settings(camera)
    image = np.full((480, 640, 3), 220, dtype=np.uint8)

    for marker_id, center in settings["reference_points_px"].items():
        paste_aruco_marker(
            image,
            "DICT_4X4_50",
            int(marker_id),
            (float(center[0]), float(center[1])),
        )

    mirrored = cv2.flip(image, 1)
    detection = detect_fiducials(mirrored, settings)
    centers = {
        int(marker_id): np.mean(marker_corners, axis=0)
        for marker_corners, marker_id in zip(
            detection.corners,
            detection.ids.reshape(-1),
            strict=True,
        )
    }

    assert detection.dictionary == "DICT_4X4_50"
    assert detection.mode == "normal+mirror_x"
    assert detection.visible_ids == [0, 1, 2, 3]
    for marker_id, reference in settings["reference_points_px"].items():
        center = centers[int(marker_id)]
        assert center[0] == approx(mirrored.shape[1] - 1 - float(reference[0]), abs=1.5)
        assert center[1] == approx(float(reference[1]), abs=1.5)


def test_normal_pipeline_never_recalibrates_from_live_tags():
    camera = working_camera()
    settings = camera["calibration"]["workspace_aruco"]
    image = np.full((480, 640, 3), 220, dtype=np.uint8)
    moved_layout = {
        0: [260, 180],
        1: [380, 180],
        2: [260, 300],
        3: [380, 300],
    }

    for marker_id, center in moved_layout.items():
        paste_aruco_marker(
            image,
            "DICT_4X4_50",
            marker_id,
            (float(center[0]), float(center[1])),
        )

    result = VisionPipeline().process(image, camera, color_profiles(load_config(EXAMPLE_CONFIG_PATH)))
    workspace = result["workspace"]

    assert workspace["visible_ids"] == []
    assert workspace["detection_mode"] == "not_checked"
    assert workspace["homography_source"] == "workspace_aruco_saved"
    assert workspace["status"] == "saved_calibration"
    assert workspace["warning"] is None
    assert workspace["layout_mismatches"] == []


def test_configured_hsv_provider_uses_workspace_projection_when_available():
    camera = working_camera()
    camera["detection"]["provider"] = "configured_hsv"
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    cv2.circle(image, (260, 220), 18, (0, 0, 255), -1)

    result = VisionPipeline().process(image, camera, color_profiles(load_config(EXAMPLE_CONFIG_PATH)))
    red = next(detection for detection in result["detections"] if detection["label"] == "red")

    assert red["ok"]
    assert red["coordinate_source"] == "workspace_aruco_saved"
    assert red["robot"] is not None
    assert red["robot"]["z_mm"] == 0.0


def test_workspace_projection_texture_uses_saved_homography_bounds():
    camera = working_camera()
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    cv2.circle(image, (260, 220), 18, (0, 0, 255), -1)

    result = VisionPipeline().process(
        image,
        camera,
        color_profiles(load_config(EXAMPLE_CONFIG_PATH)),
        include_workspace_projection=True,
    )
    projection = result["workspace_projection"]

    assert projection["ok"]
    assert projection["homography_source"] == "workspace_aruco_saved"
    assert projection["workspace_polygon_source"] == "saved_workspace_config"
    assert projection["image_b64"].startswith("data:image/jpeg;base64,")
    assert projection["texture_size_px"] == {"width": 479, "height": 316}
    assert projection["robot_bounds_mm"] == {
        "min_x": approx(-239.0),
        "max_x": approx(239.0),
        "min_y": approx(86.5),
        "max_y": approx(401.5),
        "z": approx(1.2),
    }
    assert projection["robot_polygon_mm"] == [
        [-239.0, 86.5],
        [239.0, 86.5],
        [239.0, 401.5],
        [-239.0, 401.5],
    ]

    raw = base64.b64decode(projection["image_b64"].split(",", 1)[1])
    texture = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert texture.shape == (316, 479, 3)


def test_workspace_margin_expands_projection_texture_bounds():
    camera = working_camera()
    camera["calibration"]["workspace_aruco"]["workspace_margin_mm"] = 10.0
    image = np.full((480, 640, 3), 80, dtype=np.uint8)

    result = VisionPipeline().process(
        image,
        camera,
        color_profiles(load_config(EXAMPLE_CONFIG_PATH)),
        include_workspace_projection=True,
    )
    projection = result["workspace_projection"]

    assert projection["robot_bounds_mm"] == {
        "min_x": approx(-249.0),
        "max_x": approx(249.0),
        "min_y": approx(76.5),
        "max_y": approx(411.5),
        "z": approx(1.2),
    }
    assert projection["texture_size_px"] == {"width": 499, "height": 336}


def test_reference_guided_detection_does_not_invent_tags_on_empty_workplate():
    settings = workspace_aruco_settings(working_camera())
    image = np.full((480, 640, 3), 170, dtype=np.uint8)
    cv2.rectangle(image, (55, 45), (585, 420), (205, 205, 205), 8)
    cv2.line(image, (60, 390), (580, 390), (25, 25, 25), 20)

    detection = detect_fiducials(image, settings)

    assert detection.visible_ids == []
    assert detection.mode == "none"


def test_workspace_detection_ignores_ids_outside_the_required_layout():
    camera = working_camera()
    settings = workspace_aruco_settings(camera)
    image = np.full((480, 640, 3), 220, dtype=np.uint8)
    paste_aruco_marker(image, "DICT_4X4_50", 17, (320, 240))

    detection = detect_fiducials(image, settings)

    assert 17 not in detection.visible_ids


def test_workspace_calibrate_endpoint_collects_solves_and_saves(monkeypatch):
    camera = working_camera()
    settings = workspace_aruco_settings(camera)
    image = np.full((480, 640, 3), 220, dtype=np.uint8)
    for marker_id, center in settings["reference_points_px"].items():
        paste_aruco_marker(
            image,
            "DICT_4X4_50",
            int(marker_id),
            (float(center[0]), float(center[1])),
        )
    saved_results = []
    monkeypatch.setattr(main, "camera_settings", lambda _config: camera)
    monkeypatch.setattr(main.camera_capture, "read", lambda _camera: image.copy())
    monkeypatch.setattr(
        main,
        "persist_workspace_calibration_result",
        lambda result: saved_results.append(result) or {},
    )

    response = TestClient(main.app).post(
        "/api/vision/workspace/calibrate",
        json={"max_frames": 12, "sample_interval_ms": 0},
    )
    payload = response.json()

    assert payload["ok"]
    assert payload["calibrated"]
    assert payload["result"]["frame_count"] == 12
    assert payload["result"]["metrics"]["fit_source"] == "workspace_outer_corners"
    assert len(saved_results) == 1


def test_workspace_calibrate_endpoint_accepts_mirrored_camera_frames(monkeypatch):
    camera = working_camera()
    settings = workspace_aruco_settings(camera)
    image = np.full((480, 640, 3), 220, dtype=np.uint8)
    for marker_id, center in settings["reference_points_px"].items():
        paste_aruco_marker(
            image,
            "DICT_4X4_50",
            int(marker_id),
            (float(center[0]), float(center[1])),
        )
    mirrored = cv2.flip(image, 1)
    saved_results = []
    monkeypatch.setattr(main, "camera_settings", lambda _config: camera)
    monkeypatch.setattr(main.camera_capture, "read", lambda _camera: mirrored.copy())
    monkeypatch.setattr(
        main,
        "persist_workspace_calibration_result",
        lambda result: saved_results.append(result) or {},
    )

    response = TestClient(main.app).post(
        "/api/vision/workspace/calibrate",
        json={"max_frames": 12, "sample_interval_ms": 0},
    )
    payload = response.json()

    assert payload["ok"], payload.get("error")
    assert payload["calibrated"]
    assert payload["result"]["detection_mode"] == "normal+mirror_x"
    assert payload["detections"][0]["center_px"]["x"] > 500
    assert len(saved_results) == 1


def test_external_ai_detections_use_the_same_projection_contract():
    pipeline = VisionPipeline()

    detections = pipeline.project_external_detections(
        [
            {
                "id": "yolo-7",
                "label": "red",
                "confidence": 0.91,
                "cx": 260.0,
                "cy": 220.0,
                "width": 36,
                "height": 40,
            }
        ],
        working_camera(),
    )

    assert len(detections) == 1
    detection = detections[0]
    assert detection["id"] == "yolo-7"
    assert detection["detector"] == "external_ai"
    assert detection["confidence"] == approx(0.91)
    assert detection["coordinate_source"] == "workspace_aruco_saved"
    assert detection["robot"]["x_mm"] == approx(-56.88, abs=0.1)
    assert detection["robot"]["y_mm"] == approx(234.39, abs=0.1)


def test_external_ai_projection_api_exposes_normalized_detections():
    client = TestClient(main.app)

    response = client.post(
        "/api/vision/project",
        json={
            "detections": [
                {
                    "label": "blue",
                    "confidence": 0.8,
                    "center_px": {"x": 380, "y": 250},
                    "bbox_px": {"x": 360, "y": 230, "w": 40, "h": 40},
                }
            ]
        },
    )

    payload = response.json()
    assert payload["ok"]
    assert payload["provider"] == "external_ai"
    assert payload["detections"][0]["label"] == "blue"
    assert payload["detections"][0]["robot"] is not None
