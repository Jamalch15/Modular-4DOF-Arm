from pytest import approx

from app.config import (
    EXAMPLE_CONFIG_PATH,
    MATLAB_PROTOTYPE_GEOMETRY,
    LinkConfig,
    load_config,
    matlab_geometry_to_dh_rows,
)
from app.kinematics import differential_ik_step, forward_kinematics, inverse_kinematics


def test_forward_kinematics_zero_pose_extends_up_from_shoulder():
    links = LinkConfig(
        base_height_mm=80.0,
        upper_arm_mm=140.0,
        forearm_mm=120.0,
        wrist_mm=55.0,
        tool_mm=35.0,
    )

    result = forward_kinematics([0.0, 0.0, 0.0, 0.0], links)

    assert abs(result["x_mm"]) < 1e-9
    assert abs(result["y_mm"]) < 1e-9
    assert result["z_mm"] == 430.0
    assert result["tool_phi_deg"] == 0.0


def test_matlab_geometry_builds_documented_dh_rows():
    rows = matlab_geometry_to_dh_rows(MATLAB_PROTOTYPE_GEOMETRY)

    assert [row.joint_index for row in rows] == [0, 1, 2, 3]
    assert rows[0].d_mm == approx(157.95)
    assert rows[0].a_mm == approx(0.0)
    assert rows[1].d_mm == approx(-42.69)
    assert rows[2].d_mm == approx(-41.39)
    assert rows[3].d_mm == approx(49.20)
    assert rows[1].a_mm == approx(160.15)
    assert rows[2].a_mm == approx(142.55)
    assert rows[3].a_mm == approx(15.0)
    assert rows[0].alpha_deg == approx(90.0)


def test_forward_kinematics_matches_measured_prototype_dh_poses():
    rows = matlab_geometry_to_dh_rows(MATLAB_PROTOTYPE_GEOMETRY)
    links = LinkConfig(
        base_height_mm=157.95,
        upper_arm_mm=160.15,
        forearm_mm=142.55,
        wrist_mm=15.0,
        tool_mm=0.0,
        base_side_offset_mm=23.2,
        dh_rows=rows,
    )

    zero = forward_kinematics([0.0, 0.0, 0.0, 0.0], links)
    assert zero["x_mm"] == approx(58.08)
    assert zero["y_mm"] == approx(-317.70)
    assert zero["z_mm"] == approx(157.95)
    assert zero["tool_phi_deg"] == approx(0.0)
    assert zero["dh_frames"][1]["x_mm"] == approx(23.2)
    assert zero["dh_frames"][1]["y_mm"] == approx(0.0)

    start = forward_kinematics([0.0, 0.0, -70.0, -20.0], links)
    assert start["x_mm"] == approx(58.08)
    assert start["y_mm"] == approx(-208.9049714311)
    assert start["z_mm"] == approx(8.9968169070)
    assert start["tool_phi_deg"] == approx(-90.0)


def test_forward_kinematics_base_zero_points_along_y_axis():
    links = LinkConfig(0.0, 100.0, 0.0, 0.0, 0.0)

    result = forward_kinematics([0.0, 90.0, 0.0, 0.0], links)

    assert abs(result["x_mm"]) < 1e-9
    assert round(result["y_mm"], 6) == 100.0
    assert abs(result["z_mm"]) < 1e-9


def test_forward_kinematics_base_yaw_rotates_positive_toward_negative_x():
    links = LinkConfig(0.0, 100.0, 0.0, 0.0, 0.0)

    result = forward_kinematics([90.0, 90.0, 0.0, 0.0], links)

    assert round(result["x_mm"], 6) == -100.0
    assert abs(result["y_mm"]) < 1e-9


def test_tool_tcp_z_offset_extends_along_tool_axis():
    links = LinkConfig(
        0.0,
        100.0,
        0.0,
        0.0,
        0.0,
        tool_tcp_offset_mm={"x": 0.0, "y": 0.0, "z": 20.0},
    )

    result = forward_kinematics([0.0, 90.0, 0.0, 0.0], links)

    assert abs(result["x_mm"]) < 1e-9
    assert result["y_mm"] == approx(120.0)
    assert abs(result["z_mm"]) < 1e-9


def test_inverse_kinematics_round_trips_reachable_target():
    config = load_config()
    original = [15.0, 35.0, 30.0, -20.0]
    target_fk = forward_kinematics(original, config.links)

    result = inverse_kinematics(
        {
            "x_mm": target_fk["x_mm"],
            "y_mm": target_fk["y_mm"],
            "z_mm": target_fk["z_mm"],
            "phi_deg": target_fk["tool_phi_deg"],
        },
        config.links,
        config.joints,
        original,
    )

    assert result["ok"]
    selected_fk = result["selected"]["fk"]
    assert selected_fk["x_mm"] == approx(target_fk["x_mm"], abs=1e-6)
    assert selected_fk["y_mm"] == approx(target_fk["y_mm"], abs=1e-6)
    assert selected_fk["z_mm"] == approx(target_fk["z_mm"], abs=1e-6)
    assert selected_fk["tool_phi_deg"] == approx(target_fk["tool_phi_deg"], abs=1e-6)


def test_differential_ik_step_moves_tcp_in_requested_direction():
    config = load_config()
    start = [0.0, 45.0, 25.0, -20.0]
    start_fk = forward_kinematics(start, config.links)

    result = differential_ik_step(
        start,
        {"x_mm": 1.0, "y_mm": 0.0, "z_mm": 0.0, "phi_deg": 0.0},
        config.links,
        config.joints,
    )

    assert result["ok"], result
    end_fk = forward_kinematics(result["target_angles_deg"], config.links)
    assert end_fk["x_mm"] > start_fk["x_mm"]
    assert abs(end_fk["y_mm"] - start_fk["y_mm"]) < 2.0
    assert abs(end_fk["z_mm"] - start_fk["z_mm"]) < 2.0


def test_differential_ik_step_reports_low_authority_direction():
    config = load_config()

    result = differential_ik_step(
        [0.0, 90.0, 0.0, 0.0],
        {"x_mm": 1.0, "y_mm": 0.0, "z_mm": 0.0, "phi_deg": 0.0},
        config.links,
        config.joints,
    )

    assert result["ok"], result
    assert result["blocked"]
    assert result["failure_code"] == "local_step_unreachable"
    assert "locally unreachable" in result["failure_reason"]
    assert result["target_angles_deg"] == [0.0, 90.0, 0.0, 0.0]


def test_differential_ik_step_blocks_lateral_drift_direction():
    config = load_config()

    result = differential_ik_step(
        [0.0, 62.3, 0.0, 0.0],
        {"x_mm": 0.0, "y_mm": 0.0, "z_mm": 3.0, "phi_deg": 0.0},
        config.links,
        config.joints,
    )

    assert result["ok"], result
    assert result["blocked"]
    assert result["position_alignment"] < 0.70
    assert result["failure_code"] == "excessive_lateral_drift"
    assert "excessive lateral TCP drift" in result["failure_reason"]
    assert result["target_angles_deg"] == [0.0, 62.3, 0.0, 0.0]


def test_inverse_kinematics_returns_seed_candidates():
    config = load_config()
    target = {"x_mm": -80.0, "y_mm": 180.0, "z_mm": 190.0, "phi_deg": 0.0}

    result = inverse_kinematics(target, config.links, config.joints, config.home_pose)

    branches = {candidate["branch"] for candidate in result["candidates"]}
    assert {"current_seed", "elbow_up", "elbow_down", "home_seed"}.issubset(branches)
    assert all("iterations" in candidate for candidate in result["candidates"])


def test_inverse_kinematics_rejects_unreachable_target():
    config = load_config()

    result = inverse_kinematics(
        {"x_mm": 2000.0, "y_mm": 0.0, "z_mm": 2000.0, "phi_deg": 0.0},
        config.links,
        config.joints,
        config.home_pose,
    )

    assert not result["ok"]
    assert "unreachable" in result["notes"][0]


def test_inverse_kinematics_auto_phi_chooses_reachable_orientation():
    config = load_config(EXAMPLE_CONFIG_PATH)
    # Use the tracked example geometry so private/local calibration changes do
    # not invalidate the reachability assumption behind this regression.
    target = {"x_mm": -250.0, "y_mm": 0.0, "z_mm": 400.0}

    fixed_phi = inverse_kinematics(
        {**target, "phi_deg": 0.0},
        config.links,
        config.joints,
        config.home_pose,
    )
    auto_phi = inverse_kinematics(
        {**target, "phi_auto": True},
        config.links,
        config.joints,
        config.home_pose,
    )

    assert not fixed_phi["ok"], f"Expected fixed_phi to fail but got: {fixed_phi['notes']}"
    assert auto_phi["ok"], auto_phi["notes"]
    assert auto_phi["target"]["phi_auto"] is True
    assert auto_phi["target"]["phi_deg"] == approx(auto_phi["selected"]["fk"]["tool_phi_deg"])
    selected_fk = auto_phi["selected"]["fk"]
    assert selected_fk["x_mm"] == approx(target["x_mm"], abs=1e-6)
    assert selected_fk["y_mm"] == approx(target["y_mm"], abs=1e-6)
    assert selected_fk["z_mm"] == approx(target["z_mm"], abs=1e-6)


def test_inverse_kinematics_filters_joint_limits():
    config = load_config()
    target_fk = forward_kinematics([180.0, 60.0, -40.0, 10.0], config.links)

    result = inverse_kinematics(
        {
            "x_mm": target_fk["x_mm"],
            "y_mm": target_fk["y_mm"],
            "z_mm": target_fk["z_mm"],
            "phi_deg": target_fk["tool_phi_deg"],
        },
        config.links,
        config.joints,
        config.home_pose,
    )

    assert not result["ok"]
    assert any(candidate["reasons"] for candidate in result["candidates"])


def test_inverse_kinematics_prefers_nearest_valid_solution():
    config = load_config(EXAMPLE_CONFIG_PATH)
    target_fk = forward_kinematics([-60.0, -20.0, 30.0, -80.0], config.links)
    target = {
        "x_mm": target_fk["x_mm"],
        "y_mm": target_fk["y_mm"],
        "z_mm": target_fk["z_mm"],
        "phi_deg": target_fk["tool_phi_deg"],
    }
    first = inverse_kinematics(target, config.links, config.joints, config.home_pose)
    valid = [candidate for candidate in first["candidates"] if candidate["valid"]]
    assert len(valid) >= 2

    expected = valid[0]
    second = inverse_kinematics(target, config.links, config.joints, expected["angles_deg"], expected["branch"])

    assert second["selected_branch"] == expected["branch"]


def test_inverse_kinematics_auto_prefers_continuity_over_tiny_error_difference():
    config = load_config(EXAMPLE_CONFIG_PATH)
    current = [40.0, 45.0, 20.0, 40.0]
    current_fk = forward_kinematics(current, config.links)
    target = {
        "x_mm": current_fk["x_mm"] + 3.333,
        "y_mm": current_fk["y_mm"],
        "z_mm": current_fk["z_mm"],
        "phi_deg": current_fk["tool_phi_deg"],
    }

    result = inverse_kinematics(target, config.links, config.joints, current)

    assert result["ok"], result["notes"]
    selected = result["selected"]["angles_deg"]
    assert sum(abs(angle - current[index]) for index, angle in enumerate(selected)) < 5.0


def test_analytic_seed_round_trips_multiple_fk_targets_from_home():
    config = load_config(EXAMPLE_CONFIG_PATH)
    poses = [
        [15.0, 35.0, 30.0, -20.0],
        [-45.0, 50.0, -35.0, 15.0],
        [70.0, 25.0, 45.0, -60.0],
    ]

    for pose in poses:
        target_fk = forward_kinematics(pose, config.links)
        result = inverse_kinematics(
            {
                "x_mm": target_fk["x_mm"],
                "y_mm": target_fk["y_mm"],
                "z_mm": target_fk["z_mm"],
                "phi_deg": target_fk["tool_phi_deg"],
            },
            config.links,
            config.joints,
            config.home_pose,
        )

        assert result["ok"], result["notes"]
        assert "analytic_seed" in result["notes"]
        assert result["selected"]["position_error_mm"] <= 1.0
        assert result["selected"]["phi_error_deg"] <= 1.0


def test_forward_kinematics_exposes_dh_frames():
    config = load_config()

    result = forward_kinematics(config.home_pose, config.links)

    assert len(result["dh_frames"]) == 5
    assert result["dh_frames"][-1]["x_mm"] == approx(result["wrist_frame"]["x_mm"])
    assert set(result["tool_tcp_offset_mm"]) == {"x", "y", "z"}


def test_forward_kinematics_exposes_dh_d_and_a_segments():
    rows = matlab_geometry_to_dh_rows(MATLAB_PROTOTYPE_GEOMETRY)
    links = LinkConfig(
        base_height_mm=157.95,
        upper_arm_mm=160.15,
        forearm_mm=142.55,
        wrist_mm=15.0,
        tool_mm=0.0,
        base_side_offset_mm=23.2,
        dh_rows=rows,
    )

    result = forward_kinematics([0.0, 0.0, 0.0, 0.0], links)
    segments = result["dh_segments"]

    assert [segment["label"] for segment in segments] == [
        "L1+L3",
        "L2",
        "s4*L4",
        "L5",
        "s6*L6",
        "L7",
        "s8*L8",
        "L9",
    ]
    assert [segment["kind"] for segment in segments] == ["d", "side", "d", "a", "d", "a", "d", "a"]
    assert segments[0]["length_mm"] == approx(157.95)
    assert segments[1]["length_mm"] == approx(23.2)
    assert segments[2]["signed_length_mm"] == approx(-42.69)
    assert segments[3]["length_mm"] == approx(160.15)
    assert segments[-1]["end"]["x_mm"] == approx(result["wrist_frame"]["x_mm"])
    assert segments[-1]["end"]["y_mm"] == approx(result["wrist_frame"]["y_mm"])
    assert segments[-1]["end"]["z_mm"] == approx(result["wrist_frame"]["z_mm"])
