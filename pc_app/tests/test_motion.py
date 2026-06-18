from pytest import approx

from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.kinematics import forward_kinematics
from app.motion import (
    RateLimitedMotion,
    build_joint_trajectory,
    build_linear_cartesian_trajectory,
    build_program_trajectory,
    has_reached_target,
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
