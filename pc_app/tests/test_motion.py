from dataclasses import replace

from pytest import approx

from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.kinematics import forward_kinematics, inverse_kinematics
from app.motion import (
    RateLimitedMotion,
    build_joint_trajectory,
    build_linear_cartesian_trajectory,
    build_program_trajectory,
    has_reached_target,
    ik_selection_policy,
)


def test_motion_rate_limits_large_jump():
    config = load_config()
    motion = RateLimitedMotion(config, config.home_pose.copy(), config.home_pose.copy())
    target = config.home_pose.copy()
    target[0] = 90.0
    motion.set_target(target)

    first = motion.step(1.0 / config.motion.update_rate_hz)

    assert first[0] < 90.0
    assert first[0] > config.home_pose[0]


def test_motion_eventually_reaches_target():
    config = load_config()
    motion = RateLimitedMotion(config, config.home_pose.copy(), config.home_pose.copy())
    target = config.home_pose.copy()
    target[0] = 5.0
    motion.set_target(target)

    current = config.home_pose
    for _ in range(200):
        current = motion.step(1.0 / config.motion.update_rate_hz)

    assert has_reached_target(current, target, tolerance_deg=0.1)


def test_joint_trajectory_starts_and_ends_at_expected_targets():
    config = load_config()
    start = config.home_pose.copy()
    target = start.copy()
    target[0] += 10.0
    target[1] += 5.0

    trajectory = build_joint_trajectory(
        start,
        target,
        config.joints,
        {"global_speed_deg_s": 30.0, "global_accel_deg_s2": 120.0, "waypoint_rate_hz": 10.0},
    )

    assert trajectory["ok"]
    assert trajectory["waypoints"][0] == start
    assert trajectory["waypoints"][-1] == target
    assert trajectory["duration_s"] > 0


def test_joint_trajectory_uses_requested_profile():
    config = load_config()
    start = config.home_pose.copy()
    target = start.copy()
    target[0] += 20.0

    trajectory = build_joint_trajectory(
        start,
        target,
        config.joints,
        {
            "global_speed_deg_s": 30.0,
            "global_accel_deg_s2": 120.0,
            "waypoint_rate_hz": 20.0,
            "planner_type": "trapezoid",
            "blend_percent": 25.0,
        },
    )

    assert trajectory["ok"]
    assert trajectory["profile"] == "trapezoid"
    assert trajectory["waypoints"][0] == start
    assert trajectory["waypoints"][-1] == target
    assert trajectory["segment_durations_s"][0] == 0.0
    assert trajectory["segment_durations_s"][1] > 0


def test_joint_trajectory_reports_effective_motion_contract():
    config = load_config(EXAMPLE_CONFIG_PATH)
    start = config.home_pose.copy()
    target = start.copy()
    target[2] += 20.0

    trajectory = build_joint_trajectory(
        start,
        target,
        config.joints,
        {
            "global_speed_deg_s": 100.0,
            "global_accel_deg_s2": 1000.0,
            "per_joint_speed_deg_s": [100.0, 100.0, 2.0, 100.0],
            "per_joint_accel_deg_s2": [1000.0, 1000.0, 1000.0, 1000.0],
            "planner_type": "s_curve",
            "jerk_percent": 40.0,
            "blend_percent": 20.0,
        },
    )

    assert trajectory["ok"]
    contract = trajectory["motion_contract"]
    limits = contract["limits"]
    assert contract["schema"] == "motion_plan_contract_v1"
    assert contract["path_mode"] == "joint"
    assert limits["schema"] == "motion_limit_summary_v1"
    assert limits["effective_joint_speed_deg_s"][2] == 2.0
    assert limits["limiting_constraint"]["joint_index"] == 2
    assert limits["limiting_constraint"]["type"] == "speed"
    assert "jerk_percent and blend_percent do not affect timing" in limits["notes"][0]


def test_trapezoid_ramp_is_reported_as_ramp_not_waypoint_blending():
    config = load_config(EXAMPLE_CONFIG_PATH)
    start = config.home_pose.copy()
    target = start.copy()
    target[0] += 10.0

    trajectory = build_joint_trajectory(
        start,
        target,
        config.joints,
        {
            "global_speed_deg_s": 30.0,
            "global_accel_deg_s2": 120.0,
            "planner_type": "trapezoid",
            "blend_percent": 30.0,
        },
    )

    assert trajectory["ok"]
    limits = trajectory["limit_summary"]
    assert limits["profile"] == "trapezoid"
    assert limits["trapezoid_ramp_fraction"] == approx(0.30)
    assert any("not waypoint blending" in note for note in limits["notes"])


def test_joint_trajectory_rejects_limit_violation():
    config = load_config()
    target = config.home_pose.copy()
    target[0] = 999.0

    trajectory = build_joint_trajectory(config.home_pose, target, config.joints)

    assert not trajectory["ok"]
    assert "base" in trajectory["errors"][0]


def test_linear_cartesian_trajectory_generates_waypoints():
    config = load_config()
    start = [0.0, 40.0, 25.0, -15.0]
    fk = forward_kinematics(start, config.links)
    target = {
        "x_mm": fk["x_mm"] - 15.0,
        "y_mm": fk["y_mm"] + 10.0,
        "z_mm": fk["z_mm"] - 10.0,
        "phi_deg": fk["tool_phi_deg"],
    }

    trajectory = build_linear_cartesian_trajectory(
        start,
        target,
        config.links,
        config.joints,
        {"cartesian_step_mm": 5.0, "waypoint_rate_hz": 10.0},
    )

    assert trajectory["ok"]
    assert trajectory["mode"] == "linear"
    assert trajectory["waypoint_count"] > 2


def test_linear_cartesian_trajectory_zero_distance_is_noop():
    config = load_config(EXAMPLE_CONFIG_PATH)
    start = [0.0, 40.0, 25.0, -15.0]
    fk = forward_kinematics(start, config.links)

    trajectory = build_linear_cartesian_trajectory(
        start,
        {
            "x_mm": fk["x_mm"],
            "y_mm": fk["y_mm"],
            "z_mm": fk["z_mm"],
            "phi_deg": fk["tool_phi_deg"],
        },
        config.links,
        config.joints,
        {"cartesian_step_mm": 5.0, "waypoint_rate_hz": 10.0},
    )

    assert trajectory["ok"]
    assert trajectory["waypoint_count"] == 1
    assert trajectory["duration_s"] == 0.0
    assert trajectory["waypoints"] == [[float(value) for value in start]]


def test_linear_cartesian_trajectory_rejects_unreachable_waypoint():
    config = load_config()
    target = {"x_mm": 2000.0, "y_mm": 0.0, "z_mm": 2000.0, "phi_deg": 0.0}

    trajectory = build_linear_cartesian_trajectory(
        config.home_pose,
        target,
        config.links,
        config.joints,
        {"cartesian_step_mm": 20.0},
    )

    assert not trajectory["ok"]


def test_linear_cartesian_trajectory_does_not_lock_to_seed_branch_label():
    config = load_config(EXAMPLE_CONFIG_PATH)
    start = [-45.0, 0.0, -45.0, -45.0]
    reachable_end = [-45.0, 0.0, 15.0, -45.0]
    target_fk = forward_kinematics(reachable_end, config.links)

    trajectory = build_linear_cartesian_trajectory(
        start,
        {
            "x_mm": target_fk["x_mm"],
            "y_mm": target_fk["y_mm"],
            "z_mm": target_fk["z_mm"],
            "phi_deg": target_fk["tool_phi_deg"],
        },
        config.links,
        config.joints,
        {"cartesian_step_mm": 40.0, "waypoint_rate_hz": 10.0},
    )

    assert trajectory["ok"], trajectory.get("errors")
    final_fk = forward_kinematics(trajectory["waypoints"][-1], config.links)
    assert final_fk["x_mm"] == approx(target_fk["x_mm"], abs=1.0)
    assert final_fk["y_mm"] == approx(target_fk["y_mm"], abs=1.0)
    assert final_fk["z_mm"] == approx(target_fk["z_mm"], abs=1.0)
    assert any(result["selected_branch"] != "current_seed" for result in trajectory["ik_results"])


def test_linear_cartesian_trajectory_uses_streaming_path_timing():
    config = load_config(EXAMPLE_CONFIG_PATH)
    start = [-45.0, 0.0, -45.0, -45.0]
    reachable_end = [-30.0, 15.0, 15.0, -30.0]
    target_fk = forward_kinematics(reachable_end, config.links)

    trajectory = build_linear_cartesian_trajectory(
        start,
        {
            "x_mm": target_fk["x_mm"],
            "y_mm": target_fk["y_mm"],
            "z_mm": target_fk["z_mm"],
            "phi_deg": target_fk["tool_phi_deg"],
        },
        config.links,
        config.joints,
        {"cartesian_step_mm": 15.0, "waypoint_rate_hz": 20.0},
    )

    assert trajectory["ok"], trajectory.get("errors")
    assert len(trajectory["segment_durations_s"]) == trajectory["waypoint_count"]
    assert sum(trajectory["segment_durations_s"]) == approx(trajectory["duration_s"])
    assert min(trajectory["segment_durations_s"][1:]) < 0.5


def test_program_trajectory_accepts_joint_and_cartesian_waypoints():
    config = load_config()
    start = [0.0, 40.0, 25.0, -15.0]
    first_joint = [10.0, 45.0, 20.0, -10.0]
    second_joint = [12.0, 45.0, 18.0, -8.0]
    fk = forward_kinematics(second_joint, config.links)
    program = [
        {"type": "joint", "angles_deg": first_joint, "mode": "joint"},
        {
            "type": "cartesian",
            "mode": "linear",
            "target": {
                "x_mm": fk["x_mm"],
                "y_mm": fk["y_mm"],
                "z_mm": fk["z_mm"],
                "phi_deg": fk["tool_phi_deg"],
            },
        },
    ]

    trajectory = build_program_trajectory(
        start,
        program,
        config.links,
        config.joints,
        {"waypoint_rate_hz": 12.0, "cartesian_step_mm": 6.0, "planner_type": "s_curve"},
    )

    assert trajectory["ok"]
    assert trajectory["mode"] == "program"
    assert trajectory["waypoint_count"] > 2
    assert len(trajectory["segments"]) == 2


def test_program_joint_transfer_can_change_automatic_ik_branch():
    config = load_config(EXAMPLE_CONFIG_PATH)
    rows = [
        replace(row, d_mm=42.69)
        if row.joint_index == 1
        else replace(row, a_mm=50.0)
        if row.joint_index == 3
        else row
        for row in config.links.dh_rows
    ]
    links = replace(config.links, wrist_mm=50.0, dh_rows=rows)
    joints = [
        joint
        if index == 0
        else replace(
            joint,
            min_deg=0.0 if index == 1 else -120.0,
            max_deg=180.0 if index == 1 else 120.0,
        )
        for index, joint in enumerate(config.joints)
    ]
    target = lambda x, y: {
        "x_mm": x,
        "y_mm": y,
        "z_mm": 80.0,
        "phi_auto": True,
        "preferred_phi_deg": -90.0,
    }

    trajectory = build_program_trajectory(
        [0.0, 35.0, 15.0, 0.0],
        [
            {"type": "cartesian", "mode": "joint", "label": "above pickup", "target": target(120.0, -50.0)},
            {"type": "cartesian", "mode": "joint", "label": "above dropoff", "target": target(120.0, 180.0)},
        ],
        links,
        joints,
        {"waypoint_rate_hz": 12.0},
        "auto",
    )

    assert trajectory["ok"], trajectory.get("errors")
    assert len(trajectory["segments"]) == 2


def test_pick_lift_and_cross_base_dropoff_rejects_configuration_flip():
    config = load_config(EXAMPLE_CONFIG_PATH)
    rows = [
        replace(row, d_mm=157.3, a_mm=0.0)
        if row.joint_index == 0
        else replace(row, d_mm=42.7, a_mm=160.0)
        if row.joint_index == 1
        else replace(row, d_mm=-41.5, a_mm=142.5)
        if row.joint_index == 2
        else replace(row, d_mm=48.7, a_mm=41.99)
        for row in config.links.dh_rows
    ]
    links = replace(
        config.links,
        base_height_mm=157.3,
        upper_arm_mm=160.0,
        forearm_mm=142.5,
        wrist_mm=41.99,
        base_side_offset_mm=33.7,
        dh_rows=rows,
        tool_tcp_offset_mm={"x": -20.0, "y": 0.0, "z": 145.6},
    )
    joints = [
        joint
        if index == 0
        else replace(
            joint,
            min_deg=0.0 if index == 1 else -120.0,
            max_deg=180.0 if index == 1 else 120.0,
            home_deg=90.0 if index == 1 else 0.0,
        )
        for index, joint in enumerate(config.joints)
    ]
    target = lambda x, y, z: {
        "x_mm": x,
        "y_mm": y,
        "z_mm": z,
        "phi_auto": True,
    }

    trajectory = build_program_trajectory(
        [0.0, 90.0, 0.0, 0.0],
        [
            {"type": "cartesian", "mode": "joint", "label": "above pickup", "target": target(-180.0, 40.0, 80.0)},
            {"type": "cartesian", "mode": "linear", "label": "pickup", "target": target(-180.0, 40.0, 25.0)},
            {"type": "cartesian", "mode": "linear", "label": "lift", "target": target(-180.0, 40.0, 80.0)},
            {"type": "cartesian", "mode": "joint", "label": "above dropoff", "target": target(150.0, 40.0, 80.0)},
        ],
        links,
        joints,
        {"cartesian_step_mm": 10.0, "waypoint_rate_hz": 12.0},
        "auto",
    )

    assert trajectory["ok"] is False
    assert "configuration continuity rejected all IK solutions" in trajectory["errors"][0]


def test_pick_lift_and_same_side_dropoff_stays_continuous():
    config = load_config(EXAMPLE_CONFIG_PATH)
    rows = [
        replace(row, d_mm=157.3, a_mm=0.0)
        if row.joint_index == 0
        else replace(row, d_mm=42.7, a_mm=160.0)
        if row.joint_index == 1
        else replace(row, d_mm=-41.5, a_mm=142.5)
        if row.joint_index == 2
        else replace(row, d_mm=48.7, a_mm=41.99)
        for row in config.links.dh_rows
    ]
    links = replace(
        config.links,
        base_height_mm=157.3,
        upper_arm_mm=160.0,
        forearm_mm=142.5,
        wrist_mm=41.99,
        base_side_offset_mm=33.7,
        dh_rows=rows,
        tool_tcp_offset_mm={"x": -20.0, "y": 0.0, "z": 145.6},
    )
    joints = [
        joint
        if index == 0
        else replace(
            joint,
            min_deg=0.0 if index == 1 else -120.0,
            max_deg=180.0 if index == 1 else 120.0,
            home_deg=90.0 if index == 1 else 0.0,
        )
        for index, joint in enumerate(config.joints)
    ]
    target = lambda x, y, z: {
        "x_mm": x,
        "y_mm": y,
        "z_mm": z,
        "phi_auto": True,
    }

    trajectory = build_program_trajectory(
        [0.0, 90.0, 0.0, 0.0],
        [
            {"type": "cartesian", "mode": "joint", "label": "above pickup", "target": target(-180.0, 40.0, 80.0)},
            {"type": "cartesian", "mode": "linear", "label": "pickup", "target": target(-180.0, 40.0, 25.0)},
            {"type": "cartesian", "mode": "linear", "label": "lift", "target": target(-180.0, 40.0, 80.0)},
            {"type": "cartesian", "mode": "joint", "label": "above dropoff", "target": target(-150.0, 40.0, 80.0)},
        ],
        links,
        joints,
        {"cartesian_step_mm": 10.0, "waypoint_rate_hz": 12.0},
        "auto",
    )

    assert trajectory["ok"], trajectory.get("errors")
    transfer = next(
        step["trajectory"]
        for step in trajectory["execution_steps"]
        if step["label"] == "above dropoff"
    )
    deltas = [
        abs(end - start)
        for start, end in zip(transfer["waypoints"][0], transfer["waypoints"][-1], strict=True)
    ]
    assert max(deltas) < 35.0


def test_near_base_target_matrix_never_accepts_a_configuration_flip():
    config = load_config(EXAMPLE_CONFIG_PATH)
    rows = [
        replace(row, d_mm=157.3, a_mm=0.0)
        if row.joint_index == 0
        else replace(row, d_mm=42.7, a_mm=160.0)
        if row.joint_index == 1
        else replace(row, d_mm=-41.5, a_mm=142.5)
        if row.joint_index == 2
        else replace(row, d_mm=48.7, a_mm=41.99)
        for row in config.links.dh_rows
    ]
    links = replace(
        config.links,
        base_height_mm=157.3,
        upper_arm_mm=160.0,
        forearm_mm=142.5,
        wrist_mm=41.99,
        base_side_offset_mm=33.7,
        dh_rows=rows,
        tool_tcp_offset_mm={"x": -20.0, "y": 0.0, "z": 145.6},
    )
    joints = [
        joint
        if index == 0
        else replace(
            joint,
            min_deg=0.0 if index == 1 else -120.0,
            max_deg=180.0 if index == 1 else 120.0,
            home_deg=90.0 if index == 1 else 0.0,
        )
        for index, joint in enumerate(config.joints)
    ]
    current = [-97.49, 53.33, -63.18, -90.15]
    policy = ik_selection_policy({})
    accepted = 0
    rejected = 0

    for x_mm in range(-200, 201, 50):
        for y_mm in (20.0, 40.0, 80.0, 120.0, 180.0):
            result = inverse_kinematics(
                {
                    "x_mm": float(x_mm),
                    "y_mm": y_mm,
                    "z_mm": 80.0,
                    "phi_auto": True,
                    "preferred_phi_deg": -100.0,
                },
                links,
                joints,
                current,
                "auto",
                selection_policy=policy,
            )
            if result["ok"]:
                accepted += 1
                selected = result["selected"]
                assert selected["configuration_continuous"] is True
                assert selected["continuity_violations"] == []
                assert selected["base_delta_deg"] <= policy["max_base_delta_deg"]
                assert selected["tool_winding_delta_deg"] <= policy["max_tool_winding_delta_deg"]
            else:
                rejected += 1

    assert accepted > 0
    assert rejected > 0

    cross_base = inverse_kinematics(
        {"x_mm": 150.0, "y_mm": 40.0, "z_mm": 80.0, "phi_auto": True},
        links,
        joints,
        current,
        "auto",
        selection_policy=policy,
    )
    same_side = inverse_kinematics(
        {"x_mm": -150.0, "y_mm": 40.0, "z_mm": 80.0, "phi_auto": True},
        links,
        joints,
        current,
        "auto",
        selection_policy=policy,
    )

    assert cross_base["ok"] is False
    assert "configuration continuity rejected" in cross_base["failure_reason"]
    assert same_side["ok"] is True


def test_linear_cross_base_move_fails_instead_of_flipping_orientation_in_place():
    config = load_config(EXAMPLE_CONFIG_PATH)
    rows = [
        replace(row, d_mm=157.3, a_mm=0.0)
        if row.joint_index == 0
        else replace(row, d_mm=42.7, a_mm=160.0)
        if row.joint_index == 1
        else replace(row, d_mm=-41.5, a_mm=142.5)
        if row.joint_index == 2
        else replace(row, d_mm=48.7, a_mm=41.99)
        for row in config.links.dh_rows
    ]
    links = replace(
        config.links,
        base_height_mm=157.3,
        upper_arm_mm=160.0,
        forearm_mm=142.5,
        wrist_mm=41.99,
        base_side_offset_mm=33.7,
        dh_rows=rows,
        tool_tcp_offset_mm={"x": -20.0, "y": 0.0, "z": 145.6},
    )
    joints = [
        joint
        if index == 0
        else replace(
            joint,
            min_deg=0.0 if index == 1 else -120.0,
            max_deg=180.0 if index == 1 else 120.0,
            home_deg=90.0 if index == 1 else 0.0,
        )
        for index, joint in enumerate(config.joints)
    ]

    trajectory = build_linear_cartesian_trajectory(
        [-97.49, 53.33, -63.18, -90.15],
        {"x_mm": 150.0, "y_mm": 40.0, "z_mm": 80.0, "phi_auto": True},
        links,
        joints,
        {"cartesian_step_mm": 10.0},
        "auto",
    )

    assert trajectory["ok"] is False
    assert "configuration continuity rejected" in trajectory["errors"][0]


def test_program_trajectory_rejects_missing_joint_angles():
    config = load_config()

    trajectory = build_program_trajectory(
        config.home_pose,
        [{"type": "joint"}],
        config.links,
        config.joints,
    )

    assert not trajectory["ok"]
    assert "missing joint angles" in trajectory["errors"][0]


def test_program_trajectory_skips_disabled_steps_and_reports_each_step():
    config = load_config(EXAMPLE_CONFIG_PATH)
    target = config.home_pose.copy()
    target[0] = min(config.joints[0].max_deg, target[0] + 5.0)

    trajectory = build_program_trajectory(
        config.home_pose,
        [
            {
                "label": "disabled draft",
                "type": "joint",
                "mode": "joint",
                "enabled": False,
                "angles_deg": config.home_pose,
            },
            {
                "label": "active move",
                "type": "joint",
                "mode": "joint",
                "enabled": True,
                "angles_deg": target,
            },
        ],
        config.links,
        config.joints,
        {"waypoint_rate_hz": 12.0},
    )

    assert trajectory["ok"], trajectory.get("errors")
    assert trajectory["step_count"] == 2
    assert trajectory["move_count"] == 1
    assert [result["status"] for result in trajectory["step_results"]] == ["disabled", "valid"]
    assert trajectory["segments"][0]["index"] == 1


def test_program_trajectory_exposes_the_failing_step_error():
    config = load_config(EXAMPLE_CONFIG_PATH)
    unsafe = config.home_pose.copy()
    unsafe[0] = config.joints[0].max_deg + 1.0

    trajectory = build_program_trajectory(
        config.home_pose,
        [
            {"label": "safe start", "type": "joint", "mode": "joint", "angles_deg": config.home_pose},
            {"label": "unsafe target", "type": "joint", "mode": "joint", "angles_deg": unsafe},
        ],
        config.links,
        config.joints,
        {"waypoint_rate_hz": 12.0},
    )

    assert not trajectory["ok"]
    assert trajectory["step_count"] == 2
    assert trajectory["move_count"] == 2
    assert trajectory["step_results"][0]["status"] == "valid"
    assert trajectory["step_results"][1]["index"] == 1
    assert trajectory["step_results"][1]["status"] == "invalid"
    assert "outside" in trajectory["step_results"][1]["errors"][0]


def test_program_trajectory_applies_per_step_motion_limits():
    config = load_config(EXAMPLE_CONFIG_PATH)
    target = config.home_pose.copy()
    target[0] = min(config.joints[0].max_deg, target[0] + 20.0)
    base_settings = {
        "global_speed_deg_s": 30.0,
        "global_accel_deg_s2": 120.0,
        "waypoint_rate_hz": 20.0,
    }

    default_trajectory = build_program_trajectory(
        config.home_pose,
        [{"label": "default", "type": "joint", "angles_deg": target}],
        config.links,
        config.joints,
        base_settings,
    )
    limited_trajectory = build_program_trajectory(
        config.home_pose,
        [
            {
                "label": "limited",
                "type": "joint",
                "angles_deg": target,
                "settings": {
                    "global_speed_deg_s": 5.0,
                    "global_accel_deg_s2": 20.0,
                },
            }
        ],
        config.links,
        config.joints,
        base_settings,
    )

    assert default_trajectory["ok"]
    assert limited_trajectory["ok"]
    assert limited_trajectory["duration_s"] > default_trajectory["duration_s"]
    assert limited_trajectory["segments"][0]["settings"]["global_speed_deg_s"] == 5.0


def test_program_trajectory_includes_end_effector_actions_in_timeline():
    config = load_config(EXAMPLE_CONFIG_PATH)

    trajectory = build_program_trajectory(
        config.home_pose,
        [
            {
                "label": "Close gripper",
                "type": "tool",
                "tool": "gripper",
                "action": "close",
                "settle_ms": 250,
            }
        ],
        config.links,
        config.joints,
    )

    assert trajectory["ok"], trajectory.get("errors")
    assert trajectory["move_count"] == 0
    assert trajectory["action_count"] == 1
    assert trajectory["duration_s"] == approx(0.25)
    assert trajectory["step_results"][0]["type"] == "tool"
    assert trajectory["step_results"][0]["start_time_s"] == 0.0
    assert trajectory["step_results"][0]["end_time_s"] == approx(0.25)
    assert trajectory["execution_steps"][0]["action"] == "close"
