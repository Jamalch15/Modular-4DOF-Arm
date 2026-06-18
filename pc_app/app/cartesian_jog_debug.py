from __future__ import annotations

from typing import Any

import numpy as np

from .config import RobotConfig
from .kinematics import differential_ik_step, forward_kinematics


def _fk_position(joints_deg: list[float], config: RobotConfig) -> list[float]:
    fk = forward_kinematics(joints_deg, config.links)
    return [float(fk["x_mm"]), float(fk["y_mm"]), float(fk["z_mm"])]


def _fk_pose(joints_deg: list[float], config: RobotConfig) -> dict[str, float]:
    fk = forward_kinematics(joints_deg, config.links)
    return {
        "x_mm": float(fk["x_mm"]),
        "y_mm": float(fk["y_mm"]),
        "z_mm": float(fk["z_mm"]),
        "phi_deg": float(fk["tool_phi_deg"]),
    }


def cartesian_path_metrics(points: list[list[float]], direction_xyz: list[float]) -> dict[str, float | int]:
    """Measure how well a sampled TCP path follows the requested Cartesian direction."""
    if len(points) < 2:
        return {
            "progress_mm": 0.0,
            "lateral_mm": 0.0,
            "max_lateral_mm": 0.0,
            "alignment": 0.0,
            "backward_steps": 0,
        }

    direction = np.array([float(value) for value in direction_xyz], dtype=float)
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm <= 1e-9:
        raise ValueError("direction_xyz must contain a non-zero xyz direction")
    unit = direction / direction_norm
    start = np.array(points[0], dtype=float)
    end = np.array(points[-1], dtype=float)
    displacement = end - start
    progress = float(displacement @ unit)
    lateral = float(np.linalg.norm(displacement - progress * unit))
    displacement_norm = float(np.linalg.norm(displacement))
    alignment = progress / max(displacement_norm, 1e-9)

    max_lateral = 0.0
    backward_steps = 0
    previous_progress = 0.0
    for point in points[1:]:
        relative = np.array(point, dtype=float) - start
        point_progress = float(relative @ unit)
        point_lateral = float(np.linalg.norm(relative - point_progress * unit))
        max_lateral = max(max_lateral, point_lateral)
        if point_progress < previous_progress - 1e-3:
            backward_steps += 1
        previous_progress = point_progress

    return {
        "progress_mm": progress,
        "lateral_mm": lateral,
        "max_lateral_mm": max_lateral,
        "alignment": alignment,
        "backward_steps": backward_steps,
    }


def _joint_limits(config: RobotConfig) -> tuple[list[float], list[float]]:
    return (
        [float(joint.max_speed_deg_s) for joint in config.joints],
        [float(joint.max_accel_deg_s2) for joint in config.joints],
    )


def _limit_joint_velocity(
    config: RobotConfig,
    seed_deg: list[float],
    requested_target_deg: list[float],
    previous_velocity_deg_s: list[float],
    dt_s: float,
) -> tuple[list[float], list[float], list[str]]:
    speed_limits, accel_limits = _joint_limits(config)
    desired_velocity = [
        (float(requested) - float(seed)) / max(float(dt_s), 1e-6)
        for seed, requested in zip(seed_deg, requested_target_deg, strict=True)
    ]
    speed_scale = 1.0
    for requested_velocity, limit in zip(desired_velocity, speed_limits, strict=True):
        if abs(requested_velocity) > limit:
            speed_scale = min(speed_scale, limit / max(abs(requested_velocity), 1e-9))
    desired_velocity = [requested_velocity * speed_scale for requested_velocity in desired_velocity]

    velocity_delta = [
        desired - previous
        for desired, previous in zip(desired_velocity, previous_velocity_deg_s, strict=True)
    ]
    accel_scale = 1.0
    for delta_v, limit in zip(velocity_delta, accel_limits, strict=True):
        max_velocity_delta = limit * float(dt_s)
        if abs(delta_v) > max_velocity_delta:
            accel_scale = min(accel_scale, max_velocity_delta / max(abs(delta_v), 1e-9))
    limited_velocity = [
        previous + delta_v * accel_scale
        for previous, delta_v in zip(previous_velocity_deg_s, velocity_delta, strict=True)
    ]

    target: list[float] = []
    velocity: list[float] = []
    notes: list[str] = []
    for seed, next_velocity, joint in zip(
        seed_deg, limited_velocity, config.joints, strict=True
    ):
        next_angle = float(seed) + next_velocity * float(dt_s)
        clamped_angle = max(joint.min_deg, min(joint.max_deg, next_angle))
        if abs(clamped_angle - next_angle) > 1e-6:
            next_velocity = (clamped_angle - float(seed)) / max(float(dt_s), 1e-6)
            notes.append(f"{joint.name} limit")
        target.append(clamped_angle)
        velocity.append(next_velocity)
    return target, velocity, notes


def simulate_cartesian_jog(
    config: RobotConfig,
    start_deg: list[float],
    velocity_xyz_mm_s: list[float],
    *,
    vphi_deg_s: float = 0.0,
    steps: int = 36,
    dt_s: float = 1.0 / 12.0,
    damping: float | None = None,
    apply_joint_limits: bool = True,
) -> dict[str, Any]:
    """Run a repeatable differential-IK jog simulation for debugging.

    This intentionally does not depend on FastAPI state. It lets tests and local
    scripts reproduce the exact Cartesian-jog solver behavior from a known joint
    pose and commanded Cartesian velocity.
    """
    if len(start_deg) != len(config.joints):
        raise ValueError(f"expected {len(config.joints)} start joint angles")
    if len(velocity_xyz_mm_s) != 3:
        raise ValueError("velocity_xyz_mm_s must contain x/y/z velocities")
    if steps < 1:
        raise ValueError("steps must be positive")
    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")

    current = [float(value) for value in start_deg]
    joint_velocity = [0.0 for _ in config.joints]
    points = [_fk_position(current, config)]
    samples: list[dict[str, Any]] = []
    notes: list[str] = []
    blocked_steps = 0
    speed_limits, _ = _joint_limits(config)
    solve_damping = float(config.kinematics.damping if damping is None else damping)

    for step_index in range(steps):
        task_delta = {
            "x_mm": float(velocity_xyz_mm_s[0]) * dt_s,
            "y_mm": float(velocity_xyz_mm_s[1]) * dt_s,
            "z_mm": float(velocity_xyz_mm_s[2]) * dt_s,
            "phi_deg": float(vphi_deg_s) * dt_s,
        }
        ik_step = differential_ik_step(
            current,
            task_delta,
            config.links,
            config.joints,
            damping=solve_damping,
            max_joint_step_deg=max(speed_limits) * dt_s,
        )
        if not ik_step["ok"]:
            samples.append({"step": step_index, "ik": ik_step, "error": ik_step.get("error")})
            notes.append(str(ik_step.get("error", "IK failed")))
            break

        if ik_step.get("blocked"):
            next_angles = current.copy()
            joint_velocity = [0.0 for _ in config.joints]
            limit_notes = []
        elif apply_joint_limits:
            next_angles, joint_velocity, limit_notes = _limit_joint_velocity(
                config,
                current,
                [float(value) for value in ik_step["target_angles_deg"]],
                joint_velocity,
                dt_s,
            )
        else:
            next_angles = [float(value) for value in ik_step["target_angles_deg"]]
            joint_velocity = [(next_value - current_value) / dt_s for current_value, next_value in zip(current, next_angles)]
            limit_notes = []

        step_notes = [*ik_step.get("notes", []), *limit_notes]
        if ik_step.get("blocked") or limit_notes:
            blocked_steps += 1
        notes.extend(str(note) for note in step_notes)
        current = next_angles
        points.append(_fk_position(current, config))
        samples.append(
            {
                "step": step_index,
                "joints_deg": current,
                "joint_velocity_deg_s": joint_velocity,
                "ik": ik_step,
                "notes": step_notes,
            }
        )

    metrics = cartesian_path_metrics(points, velocity_xyz_mm_s)
    return {
        "start_deg": [float(value) for value in start_deg],
        "final_deg": current,
        "velocity_xyz_mm_s": [float(value) for value in velocity_xyz_mm_s],
        "vphi_deg_s": float(vphi_deg_s),
        "dt_s": float(dt_s),
        "steps": len(samples),
        "points": points,
        "samples": samples,
        "blocked_steps": blocked_steps,
        "notes": list(dict.fromkeys(notes)),
        "metrics": metrics,
    }
