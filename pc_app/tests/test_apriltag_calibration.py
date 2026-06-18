from dataclasses import replace
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
from fastapi.testclient import TestClient
from pytest import approx

import app.main as main
from app.apriltag_calibration import (
    AprilTagCalibrationSession,
    AprilTagDetection,
    april_tag_settings,
    detect_apriltags,
    estimate_camera_pose,
    project_image_point_to_plane,
    tag_world_center,
    tag_world_corners,
)
from app.config import load_config
from app.demo_settings import camera_settings
from app.vision import encode_image_b64


def calibrated_camera() -> dict:
    config = load_config()
    camera = camera_settings(config)
    camera["intrinsics"] = {
        "source": "synthetic_test",
        "fx_px": 900.0,
        "fy_px": 900.0,
        "cx_px": 640.0,
        "cy_px": 360.0,
        "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0],
    }
    camera["resolution"] = {"width": 1280, "height": 720}
    return camera


def synthetic_detections(camera: dict, noise_px: float = 0.0, seed: int = 1) -> list[AprilTagDetection]:
    settings = april_tag_settings(camera)
    camera_matrix = np.array(
        [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    rotation_camera_to_robot = np.diag([1.0, -1.0, -1.0])
    rotation_robot_to_camera = rotation_camera_to_robot.T
    camera_position = np.array([0.0, 244.0, 700.0], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(rotation_robot_to_camera)
    tvec = -rotation_robot_to_camera @ camera_position.reshape(3, 1)
    rng = np.random.default_rng(seed)
    detections = []
    for tag_id in [0, 1, 2, 3]:
        projected, _ = cv2.projectPoints(
            tag_world_corners(tag_id, settings),
            rvec,
            tvec,
            camera_matrix,
            np.zeros(5),
        )
        corners = projected.reshape(4, 2)
        if noise_px:
            corners = corners + rng.normal(0.0, noise_px, corners.shape)
        detections.append(AprilTagDetection(tag_id, corners))
    return detections


def synthetic_tag_image(camera: dict) -> np.ndarray:
    image = np.full((720, 1280, 3), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    source = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], dtype=np.float32)
    for detection in synthetic_detections(camera):
        marker = cv2.aruco.generateImageMarker(dictionary, detection.tag_id, 200)
        marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        transform = cv2.getPerspectiveTransform(source, detection.corners_px.astype(np.float32))
        warped = cv2.warpPerspective(marker_bgr, transform, (1280, 720), borderValue=(255, 255, 255))
        mask = cv2.warpPerspective(np.full((200, 200), 255, dtype=np.uint8), transform, (1280, 720))
        image[mask > 0] = warped[mask > 0]
    return image


def test_detect_apriltags_finds_generated_dictionary_markers():
    camera = calibrated_camera()
    settings = april_tag_settings(camera)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    image = np.full((700, 900, 3), 255, dtype=np.uint8)
    positions = [(80, 80), (650, 80), (650, 430), (80, 430)]
    for tag_id, (x, y) in enumerate(positions):
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, 160)
        image[y : y + 160, x : x + 160] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    detections = detect_apriltags(image, settings)

    assert sorted(detection.tag_id for detection in detections) == [0, 1, 2, 3]


def test_camera_pose_recovers_synthetic_overhead_camera():
    camera = calibrated_camera()

    result = estimate_camera_pose(synthetic_detections(camera), camera)

    assert result["ok"]
    assert result["accepted"]
    assert result["tags_used"] == [0, 1, 2, 3]
    assert result["camera_to_robot"]["position_mm"] == approx([0.0, 244.0, 700.0], abs=1e-3)
    assert result["metrics"]["reprojection_rmse_px"] < 1e-4
    assert result["metrics"]["tilt_from_down_deg"] < 1e-4
    assert result["planar"]["ok"]


def test_workspace_corners_anchor_outer_tag_corners_and_move_centers_inward():
    camera = calibrated_camera()
    settings = april_tag_settings(camera)

    assert tag_world_center(0, settings) == approx([-219.0, 106.5, 0.0])
    assert tag_world_center(1, settings) == approx([219.0, 106.5, 0.0])
    assert tag_world_center(2, settings) == approx([219.0, 381.5, 0.0])
    assert tag_world_center(3, settings) == approx([-219.0, 381.5, 0.0])
    assert tag_world_corners(0, settings)[3] == approx([-239.0, 86.5, 0.0])
    assert tag_world_corners(1, settings)[2] == approx([239.0, 86.5, 0.0])
    assert tag_world_corners(2, settings)[1] == approx([239.0, 401.5, 0.0])
    assert tag_world_corners(3, settings)[0] == approx([-239.0, 401.5, 0.0])


def test_multi_frame_session_reduces_noisy_corner_jitter():
    camera = calibrated_camera()
    session = AprilTagCalibrationSession(camera)
    for index in range(20):
        session.add(synthetic_detections(camera, noise_px=0.7, seed=index))

    result = session.solve()

    assert result["accepted"]
    assert result["frames_used"] == 20
    assert result["minimum_samples_met"]
    assert set(result["corner_jitter_rms_px"]) == {"0", "1", "2", "3"}
    assert result["camera_to_robot"]["position_mm"] == approx([0.0, 244.0, 700.0], abs=2.0)


def test_saved_pose_projects_image_ray_to_workspace_plane():
    camera = calibrated_camera()
    result = estimate_camera_pose(synthetic_detections(camera), camera)
    world_point = np.array([[100.0, 200.0, 0.0]], dtype=np.float64)
    rvec = np.asarray(result["world_to_camera"]["rvec"], dtype=np.float64)
    tvec = np.asarray(result["world_to_camera"]["tvec_mm"], dtype=np.float64)
    camera_matrix = np.asarray(result["camera_matrix"], dtype=np.float64)
    pixel, _ = cv2.projectPoints(world_point, rvec, tvec, camera_matrix, np.zeros(5))

    projected = project_image_point_to_plane(pixel.reshape(2).tolist(), result)

    assert projected == approx({"x_mm": 100.0, "y_mm": 200.0, "z_mm": 0.0}, abs=1e-4)


def test_saved_pose_projection_corrects_lens_distortion():
    camera = calibrated_camera()
    result = estimate_camera_pose(synthetic_detections(camera), camera)
    distortion = np.array([0.16, -0.08, 0.002, -0.001, 0.02], dtype=np.float64)
    result["distortion_coefficients"] = distortion.tolist()
    world_point = np.array([[180.0, 330.0, 0.0]], dtype=np.float64)
    pixel, _ = cv2.projectPoints(
        world_point,
        np.asarray(result["world_to_camera"]["rvec"], dtype=np.float64),
        np.asarray(result["world_to_camera"]["tvec_mm"], dtype=np.float64),
        np.asarray(result["camera_matrix"], dtype=np.float64),
        distortion,
    )

    projected = project_image_point_to_plane(pixel.reshape(2).tolist(), result)

    assert projected == approx({"x_mm": 180.0, "y_mm": 330.0, "z_mm": 0.0}, abs=1e-3)


def test_session_requires_minimum_observations_for_each_required_tag():
    camera = calibrated_camera()
    session = AprilTagCalibrationSession(camera)
    detections = synthetic_detections(camera)
    for _ in range(12):
        session.add([detections[0]])
    session.add(detections[1:])

    result = session.solve()

    assert result["minimum_samples_met"]
    assert not result["required_tag_samples_met"]
    assert result["tag_observation_counts"] == {"0": 12, "1": 1, "2": 1, "3": 1}


def test_session_resets_samples_when_camera_pixel_geometry_changes():
    camera = calibrated_camera()
    session = AprilTagCalibrationSession(camera)
    session.add(synthetic_detections(camera))
    changed_camera = {
        **camera,
        "resolution": {"width": 640, "height": 480},
    }

    session.configure(changed_camera, preserve_frames=True)

    assert session.summary()["frame_count"] == 0


def test_session_resets_samples_when_tag_anchor_geometry_changes():
    camera = calibrated_camera()
    session = AprilTagCalibrationSession(camera)
    session.add(synthetic_detections(camera))
    changed_camera = {
        **camera,
        "calibration": {
            **camera["calibration"],
            "apriltag": {
                **camera["calibration"]["apriltag"],
                "tags": {
                    **camera["calibration"]["apriltag"]["tags"],
                    "0": {
                        **camera["calibration"]["apriltag"]["tags"]["0"],
                        "workspace_corner_mm": [-238.0, 86.5, 0.0],
                    },
                },
            },
        },
    }

    session.configure(changed_camera, preserve_frames=True)

    assert session.summary()["frame_count"] == 0


def test_camera_settings_deep_merge_keeps_apriltag_defaults():
    config = load_config()
    patched = replace(config, raw={**config.raw, "camera": {"source_index": 2, "intrinsics": {"fx_px": 800.0}}})

    camera = camera_settings(patched)

    assert camera["source_index"] == 2
    assert camera["intrinsics"]["fx_px"] == 800.0
    assert camera["intrinsics"]["distortion_coefficients"] == [0.0] * 5
    assert camera["calibration"]["apriltag"]["required_ids"] == [0, 1, 2, 3]


def test_apriltag_capture_api_accumulates_synthetic_frames():
    original_config = main.config
    camera = calibrated_camera()
    main.config = replace(original_config, raw={**original_config.raw, "camera": camera})
    main.april_tag_session.configure(camera, preserve_frames=False)
    client = TestClient(main.app)
    image_b64 = encode_image_b64(synthetic_tag_image(camera), ".png")
    try:
        payload = None
        for _ in range(12):
            response = client.post(
                "/api/vision/apriltag/capture",
                json={"image_b64": image_b64, "sample_count": 1, "accumulate": True},
            )
            payload = response.json()
            assert payload["ok"]
        assert payload is not None
        assert payload["session"]["frame_count"] == 12
        assert payload["result"]["accepted"]
        assert payload["result"]["tags_used"] == [0, 1, 2, 3]
        status = client.get("/api/vision/apriltag/status").json()
        assert status["ok"]
        assert status["session"]["tag_ids"] == [0, 1, 2, 3]
    finally:
        main.config = original_config
        main.april_tag_session.configure(camera_settings(original_config), preserve_frames=False)


def test_calibration_cli_documented_invocation_loads_local_app_package():
    root = Path(__file__).resolve().parents[1]

    result = subprocess.run(
        [sys.executable, "tools/calibrate_apriltags.py", "--help"],
        cwd=root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "Collect and verify the fixed workspace AprilTag camera pose." in result.stdout
