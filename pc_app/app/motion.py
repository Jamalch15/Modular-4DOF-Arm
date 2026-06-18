from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil, hypot, sqrt
from typing import Any

from .config import JointConfig, LinkConfig, RobotConfig
from .kinematics import angle_distance_deg, forward_kinematics, inverse_kinematics


@dataclass
class RateLimitedMotion:
    config: RobotConfig
    current_deg: list[float]
    target_deg: list[float]
    velocities_deg_s: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.velocities_deg_s:
            self.velocities_deg_s = [0.0 for _ in self.current_deg]

    def set_target(self, targets: list[float]) -> None:
        self.target_deg = [float(value) for value in targets]

    def reset(self, pose: list[float]) -> None:
        self.current_deg = [float(value) for value in pose]
        self.target_deg = [float(value) for value in pose]
        self.velocities_deg_s = [0.0 for _ in pose]

    def step(self, dt_s: float) -> list[float]:
        if self.config.motion.allow_sudden_jumps:
            self.current_deg = self.target_deg.copy()
            self.velocities_deg_s = [0.0 for _ in self.current_deg]
            return self.current_deg

        next_angles: list[float] = []
        alpha = self.config.motion.smoothing_alpha
        for index, (current, target, velocity, joint) in enumerate(
            zip(self.current_deg, self.target_deg, self.velocities_deg_s, self.config.joints, strict=True)
        ):
            delta = target - current
            if abs(delta) < 0.001:
                next_angles.append(target)
                self.velocities_deg_s[index] = 0.0
                continue

            desired_velocity = max(-joint.max_speed_deg_s, min(joint.max_speed_deg_s, delta / max(dt_s, 1e-6)))
            velocity_delta = desired_velocity - velocity
            max_velocity_delta = min(self.config.motion.acceleration_deg_s2, joint.max_accel_deg_s2) * dt_s
            if velocity_delta > max_velocity_delta:
                velocity += max_velocity_delta
            elif velocity_delta < -max_velocity_delta:
                velocity -= max_velocity_delta
            else:
                velocity = desired_velocity

            speed_limited_step = velocity * dt_s
            smoothed_step = delta * alpha
            step = max(-abs(speed_limited_step), min(abs(speed_limited_step), smoothed_step))
            if abs(step) > abs(delta):
                step = delta

            next_angles.append(current + step)
            self.velocities_deg_s[index] = velocity

        self.current_deg = next_angles
        return next_angles


def has_reached_target(current: list[float], target: list[float], tolerance_deg: float = 0.05) -> bool:
    return all(abs(a - b) <= tolerance_deg for a, b in zip(current, target, strict=True))


def _limit_value(requested: float | None, fallback: float, minimum: float = 1e-6) -> float:
    if requested is None:
        return fallback
    return max(float(requested), minimum)


def _profile_name(settings: dict[str, Any]) -> str:
    value = str(settings.get("planner_type") or settings.get("profile") or "s_curve").lower()
    if value in {"s_curve", "scurve", "s-curve"}:
        return "s_curve"
    if value in {"linear", "none"}:
        return "linear"
    return "trapezoid"


def _trapezoid_ramp_fraction(settings: dict[str, Any]) -> float:
    blend = settings.get("blend_percent", settings.get("jerk_percent", 25.0))
    try:
        value = float(blend) / 100.0
    except (TypeError, ValueError):
        value = 0.25
    return min(0.45, max(0.05, value))


def _profile_progress(t: float, profile: str, ramp_fraction: float) -> float:
    t = min(1.0, max(0.0, t))
    if profile == "linear":
        return t
    if profile == "s_curve":
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)

    ramp = min(0.45, max(0.05, ramp_fraction))
    denom = ramp * (1.0 - ramp)
    if t < ramp:
        return 0.5 * t * t / denom
    if t > 1.0 - ramp:
        remaining = 1.0 - t
        return 1.0 - 0.5 * remaining * remaining / denom
    return (t - 0.5 * ramp) / (1.0 - ramp)


def _signed_angle_delta_deg(start: float, end: float) -> float:
    return (float(end) - float(start) + 180.0) % 360.0 - 180.0


def _joint_speed_limits(
    joints: list[JointConfig],
    global_speed_deg_s: float | None,
    per_joint_speed_deg_s: list[float] | None = None,
) -> list[float]:
    limits: list[float] = []
    for index, joint in enumerate(joints):
        requested = None
        if per_joint_speed_deg_s and index < len(per_joint_speed_deg_s):
            requested = per_joint_speed_deg_s[index]
        global_limit = _limit_value(global_speed_deg_s, joint.max_speed_deg_s)
        per_joint_limit = _limit_value(requested, joint.max_speed_deg_s)
        limits.append(min(joint.max_speed_deg_s, global_limit, per_joint_limit))
    return limits


def _joint_accel_limits(
    joints: list[JointConfig],
    global_accel_deg_s2: float | None,
    per_joint_accel_deg_s2: list[float] | None = None,
) -> list[float]:
    limits: list[float] = []
    for index, joint in enumerate(joints):
        requested = None
        if per_joint_accel_deg_s2 and index < len(per_joint_accel_deg_s2):
            requested = per_joint_accel_deg_s2[index]
        global_limit = _limit_value(global_accel_deg_s2, joint.max_accel_deg_s2)
        per_joint_limit = _limit_value(requested, joint.max_accel_deg_s2)
        limits.append(min(joint.max_accel_deg_s2, global_limit, per_joint_limit))
    return limits


def _validate_joint_targets(joints: list[JointConfig], angles_deg: list[float]) -> list[str]:
    reasons: list[str] = []
    if len(angles_deg) != len(joints):
        return [f"expected {len(joints)} joint targets"]
    for angle, joint in zip(angles_deg, joints, strict=True):
        if angle < joint.min_deg or angle > joint.max_deg:
            reasons.append(
                f"{joint.name} target {angle:.2f} deg outside {joint.min_deg:.2f}..{joint.max_deg:.2f} deg"
            )
    return reasons


def _segment_duration_s(
    start_deg: list[float],
    end_deg: list[float],
    speed_limits_deg_s: list[float],
    accel_limits_deg_s2: list[float],
    profile: str = "s_curve",
    ramp_fraction: float = 0.25,
) -> float:
    deltas = [abs(end - start) for start, end in zip(start_deg, end_deg, strict=True)]
    return _joint_delta_duration_s(deltas, speed_limits_deg_s, accel_limits_deg_s2, profile, ramp_fraction)


def _joint_delta_duration_s(
    deltas_deg: list[float],
    speed_limits_deg_s: list[float],
    accel_limits_deg_s2: list[float],
    profile: str = "s_curve",
    ramp_fraction: float = 0.25,
) -> float:
    duration = 0.0
    for delta, speed, accel in zip(deltas_deg, speed_limits_deg_s, accel_limits_deg_s2, strict=True):
        if delta <= 1e-9:
            continue
        if profile == "linear":
            speed_time = delta / max(speed, 1e-6)
            accel_time = 0.0
        elif profile == "trapezoid":
            speed_time = delta / (max(speed, 1e-6) * (1.0 - ramp_fraction))
            accel_time = sqrt(delta / (max(accel, 1e-6) * ramp_fraction * (1.0 - ramp_fraction)))
        else:
            speed_time = 1.9 * delta / max(speed, 1e-6)
            accel_time = 2.5 * sqrt(delta / max(accel, 1e-6))
        duration = max(duration, speed_time, accel_time)
    return duration


def _joint_path_deltas(waypoints: list[list[float]]) -> list[float]:
    if not waypoints:
        return []
    deltas = [0.0 for _ in waypoints[0]]
    for start, end in zip(waypoints, waypoints[1:], strict=False):
        for index, (start_angle, end_angle) in enumerate(zip(start, end, strict=True)):
            deltas[index] += abs(float(end_angle) - float(start_angle))
    return deltas


def _joint_segment_distance(start_deg: list[float], end_deg: list[float]) -> float:
    return sum(abs(float(end) - float(start)) for start, end in zip(start_deg, end_deg, strict=True))


def _cumulative_times(segment_durations_s: list[float]) -> list[float]:
    elapsed = 0.0
    times: list[float] = []
    for duration in segment_durations_s:
        elapsed += max(0.0, float(duration))
        times.append(elapsed)
    return times


def _continuous_ik_branch(branch: str) -> str:
    return branch if branch in {"elbow_up", "elbow_down"} else "auto"


def _select_continuous_ik_candidate(ik: dict[str, Any], previous_deg: list[float], branch: str) -> dict[str, Any] | None:
    if branch in {"elbow_up", "elbow_down"}:
        return ik.get("selected")
    valid_candidates = [candidate for candidate in ik.get("candidates", []) if candidate.get("valid")]
    if not valid_candidates:
        return None
    return min(
        valid_candidates,
        key=lambda candidate: (
            sum(
                angle_distance_deg(float(angle), float(previous_deg[index]))
                for index, angle in enumerate(candidate.get("angles_deg", []))
            ),
            float(candidate.get("position_error_mm", 0.0)),
            float(candidate.get("phi_error_deg", 0.0)),
            0 if candidate.get("branch") in {"current_seed", "elbow_down"} else 1,
        ),
    )


def _linear_ik_error(index: int, ik: dict[str, Any]) -> str:
    details: list[str] = []
    for note in ik.get("notes", [])[:3]:
        details.append(str(note))
    for candidate in ik.get("candidates", [])[:4]:
        reasons = candidate.get("reasons") or []
        if reasons:
            details.append(f"{candidate.get('branch', 'candidate')}: {reasons[0]}")
    suffix = f": {'; '.join(details)}" if details else ""
    return f"linear waypoint {index} has no valid position/orientation IK solution{suffix}"


def build_joint_trajectory(
    start_deg: list[float],
    target_deg: list[float],
    joints: list[JointConfig],
    settings: dict[str, Any] | None = None,
) -> dict[str, Any]:
    settings = settings or {}
    reasons = _validate_joint_targets(joints, target_deg)
    if reasons:
        return {"ok": False, "mode": "joint", "waypoints": [], "errors": reasons}

    waypoint_rate_hz = _limit_value(settings.get("waypoint_rate_hz"), 12.0)
    speed_limits = _joint_speed_limits(
        joints,
        settings.get("global_speed_deg_s"),
        settings.get("per_joint_speed_deg_s"),
    )
    accel_limits = _joint_accel_limits(
        joints,
        settings.get("global_accel_deg_s2"),
        settings.get("per_joint_accel_deg_s2"),
    )
    profile = _profile_name(settings)
    ramp_fraction = _trapezoid_ramp_fraction(settings)
    duration_s = _segment_duration_s(start_deg, target_deg, speed_limits, accel_limits, profile, ramp_fraction)
    if duration_s <= 1e-9:
        return {
            "ok": True,
            "mode": "joint",
            "profile": profile,
            "duration_s": 0.0,
            "waypoint_count": 1,
            "waypoints": [target_deg],
            "segment_durations_s": [0.0],
            "time_from_start_s": [0.0],
            "errors": [],
        }

    steps = max(2, int(ceil(duration_s * waypoint_rate_hz)) + 1)
    waypoints: list[list[float]] = []
    for index in range(steps):
        t = index / (steps - 1)
        smooth_t = _profile_progress(t, profile, ramp_fraction)
        waypoints.append(
            [start + (target - start) * smooth_t for start, target in zip(start_deg, target_deg, strict=True)]
        )

    segment_duration = duration_s / max(steps - 1, 1)
    segment_durations = [0.0] + [segment_duration for _ in waypoints[1:]]
    return {
        "ok": True,
        "mode": "joint",
        "profile": profile,
        "duration_s": duration_s,
        "waypoint_count": len(waypoints),
        "waypoints": waypoints,
        "segment_durations_s": segment_durations,
        "time_from_start_s": _cumulative_times(segment_durations),
        "speed_limits_deg_s": speed_limits,
        "accel_limits_deg_s2": accel_limits,
        "errors": [],
    }


def build_linear_cartesian_trajectory(
    start_deg: list[float],
    target: dict[str, Any],
    links: LinkConfig,
    joints: list[JointConfig],
    settings: dict[str, Any] | None = None,
    branch: str = "auto",
) -> dict[str, Any]:
    settings = settings or {}
    start_fk = forward_kinematics(start_deg, links)
    raw_phi = target.get("phi_deg", target.get("tool_phi_deg"))
    auto_phi = bool(target.get("phi_auto", False)) or raw_phi is None
    if auto_phi:
        final_ik = inverse_kinematics(
            {
                "x_mm": float(target.get("x_mm", start_fk["x_mm"])),
                "y_mm": float(target.get("y_mm", start_fk["y_mm"])),
                "z_mm": float(target.get("z_mm", start_fk["z_mm"])),
                "phi_auto": True,
            },
            links,
            joints,
            start_deg,
            branch,
        )
        if not final_ik["ok"] or not final_ik["selected"]:
            return {
                "ok": False,
                "mode": "linear",
                "waypoints": [],
                "cartesian_waypoints": [],
                "ik_results": [
                    {
                        "index": "end",
                        "ok": False,
                        "selected_branch": final_ik.get("selected_branch"),
                        "notes": final_ik.get("notes", []),
                    }
                ],
                "errors": ["linear target has no reachable auto-phi solution"],
            }
        end_phi_deg = float(final_ik["selected"]["fk"]["tool_phi_deg"])
    else:
        end_phi_deg = float(raw_phi)
    end_pose = {
        "x_mm": float(target.get("x_mm", start_fk["x_mm"])),
        "y_mm": float(target.get("y_mm", start_fk["y_mm"])),
        "z_mm": float(target.get("z_mm", start_fk["z_mm"])),
        "phi_deg": end_phi_deg,
    }
    start_pose = {
        "x_mm": start_fk["x_mm"],
        "y_mm": start_fk["y_mm"],
        "z_mm": start_fk["z_mm"],
        "phi_deg": start_fk["tool_phi_deg"],
    }

    distance_mm = hypot(
        hypot(end_pose["x_mm"] - start_pose["x_mm"], end_pose["y_mm"] - start_pose["y_mm"]),
        end_pose["z_mm"] - start_pose["z_mm"],
    )
    max_step_mm = _limit_value(settings.get("cartesian_step_mm"), 10.0)
    steps = max(2, int(ceil(distance_mm / max_step_mm)) + 1)
    if distance_mm <= 1e-9 and angle_distance_deg(end_pose["phi_deg"], start_pose["phi_deg"]) > 1e-9:
        steps = max(steps, 8)

    waypoints: list[list[float]] = []
    cartesian_waypoints: list[dict[str, float]] = []
    ik_results: list[dict[str, Any]] = []
    previous = start_deg
    ik_branch = _continuous_ik_branch(branch)
    phi_delta = _signed_angle_delta_deg(start_pose["phi_deg"], end_pose["phi_deg"])

    for index in range(steps):
        t = index / (steps - 1)
        pose = {
            "x_mm": start_pose["x_mm"] + (end_pose["x_mm"] - start_pose["x_mm"]) * t,
            "y_mm": start_pose["y_mm"] + (end_pose["y_mm"] - start_pose["y_mm"]) * t,
            "z_mm": start_pose["z_mm"] + (end_pose["z_mm"] - start_pose["z_mm"]) * t,
            "phi_deg": start_pose["phi_deg"] + phi_delta * t,
        }
        ik = inverse_kinematics(pose, links, joints, previous, ik_branch)
        selected = _select_continuous_ik_candidate(ik, previous, ik_branch)
        ik_results.append(
            {
                "index": index,
                "ok": selected is not None,
                "selected_branch": selected["branch"] if selected else ik["selected_branch"],
                "notes": ik["notes"],
            }
        )
        if selected is None:
            return {
                "ok": False,
                "mode": "linear",
                "waypoints": waypoints,
                "cartesian_waypoints": cartesian_waypoints,
                "ik_results": ik_results,
                "errors": [_linear_ik_error(index, ik)],
            }
        previous = [float(value) for value in selected["angles_deg"]]
        waypoints.append(previous)
        cartesian_waypoints.append(pose)

    waypoint_rate_hz = _limit_value(settings.get("waypoint_rate_hz"), 12.0)
    speed_limits = _joint_speed_limits(
        joints,
        settings.get("global_speed_deg_s"),
        settings.get("per_joint_speed_deg_s"),
    )
    accel_limits = _joint_accel_limits(
        joints,
        settings.get("global_accel_deg_s2"),
        settings.get("per_joint_accel_deg_s2"),
    )
    profile = _profile_name(settings)
    ramp_fraction = _trapezoid_ramp_fraction(settings)
    path_deltas = _joint_path_deltas(waypoints)
    duration_s = _joint_delta_duration_s(path_deltas, speed_limits, accel_limits, profile, ramp_fraction)
    duration_s = max(duration_s, (len(waypoints) - 1) / waypoint_rate_hz)
    segment_distances = [
        _joint_segment_distance(start, end)
        for start, end in zip(waypoints, waypoints[1:], strict=False)
    ]
    total_segment_distance = sum(segment_distances)
    if total_segment_distance <= 1e-9:
        segment_durations = [0.0] + [
            duration_s / max(len(waypoints) - 1, 1)
            for _ in waypoints[1:]
        ]
    else:
        segment_durations = [0.0] + [
            duration_s * distance / total_segment_distance
            for distance in segment_distances
        ]

    return {
        "ok": True,
        "mode": "linear",
        "profile": profile,
        "duration_s": duration_s,
        "waypoint_count": len(waypoints),
        "waypoints": waypoints,
        "cartesian_waypoints": cartesian_waypoints,
        "segment_durations_s": segment_durations,
        "time_from_start_s": _cumulative_times(segment_durations),
        "speed_limits_deg_s": speed_limits,
        "accel_limits_deg_s2": accel_limits,
        "ik_results": ik_results,
        "errors": [],
    }


def _settings_for_waypoint(base: dict[str, Any], waypoint: dict[str, Any]) -> dict[str, Any]:
    settings = dict(base)
    waypoint_settings = waypoint.get("settings")
    if isinstance(waypoint_settings, dict):
        settings.update(waypoint_settings)
    for key in (
        "global_speed_deg_s",
        "global_accel_deg_s2",
        "waypoint_rate_hz",
        "cartesian_step_mm",
        "planner_type",
        "profile",
        "jerk_percent",
        "blend_percent",
    ):
        if key in waypoint and waypoint[key] is not None:
            settings[key] = waypoint[key]
    return settings


def _append_segment(
    combined_waypoints: list[list[float]],
    combined_durations: list[float],
    segment: dict[str, Any],
) -> None:
    waypoints = segment.get("waypoints", [])
    durations = segment.get("segment_durations_s", [])
    if not waypoints:
        return
    start_index = 1 if combined_waypoints else 0
    for index, waypoint in enumerate(waypoints[start_index:], start=start_index):
        combined_waypoints.append([float(value) for value in waypoint])
        duration = float(durations[index]) if index < len(durations) else 0.0
        combined_durations.append(duration)


def build_program_trajectory(
    start_deg: list[float],
    waypoints: list[dict[str, Any]],
    links: LinkConfig,
    joints: list[JointConfig],
    settings: dict[str, Any] | None = None,
    branch: str = "auto",
) -> dict[str, Any]:
    settings = settings or {}
    if not waypoints:
        return {"ok": False, "mode": "program", "waypoints": [], "errors": ["program has no waypoints"]}

    current = [float(value) for value in start_deg]
    selected_branch = branch
    combined_waypoints: list[list[float]] = []
    combined_durations: list[float] = []
    segment_summaries: list[dict[str, Any]] = []
    cartesian_waypoints: list[dict[str, float]] = []

    for index, waypoint in enumerate(waypoints):
        waypoint_settings = _settings_for_waypoint(settings, waypoint)
        kind = str(waypoint.get("type") or waypoint.get("kind") or "cartesian").lower()
        mode = str(waypoint.get("mode") or ("linear" if kind == "cartesian" else "joint")).lower()
        waypoint_branch = str(waypoint.get("branch") or selected_branch or "auto")

        if kind == "joint":
            target_angles = waypoint.get("angles_deg") or waypoint.get("joints_deg")
            if isinstance(waypoint.get("target"), dict):
                target_angles = target_angles or waypoint["target"].get("angles_deg")
            if not isinstance(target_angles, list):
                return {
                    "ok": False,
                    "mode": "program",
                    "waypoints": combined_waypoints,
                    "errors": [f"program waypoint {index + 1} missing joint angles"],
                    "segments": segment_summaries,
                }
            segment = build_joint_trajectory(current, [float(value) for value in target_angles], joints, waypoint_settings)
        else:
            raw_target = waypoint.get("target") if isinstance(waypoint.get("target"), dict) else waypoint
            raw_phi = raw_target.get("phi_deg", raw_target.get("phi"))
            target = {
                "x_mm": float(raw_target.get("x_mm", raw_target.get("x", 0.0))),
                "y_mm": float(raw_target.get("y_mm", raw_target.get("y", 0.0))),
                "z_mm": float(raw_target.get("z_mm", raw_target.get("z", 0.0))),
            }
            if bool(raw_target.get("phi_auto", False)) or raw_phi is None:
                target["phi_auto"] = True
            else:
                target["phi_deg"] = float(raw_phi)
            if mode == "linear":
                segment = build_linear_cartesian_trajectory(
                    current,
                    target,
                    links,
                    joints,
                    waypoint_settings,
                    waypoint_branch,
                )
            else:
                ik = inverse_kinematics(target, links, joints, current, waypoint_branch)
                if not ik["ok"] or not ik["selected"]:
                    return {
                        "ok": False,
                        "mode": "program",
                        "waypoints": combined_waypoints,
                        "errors": [f"program waypoint {index + 1} has no valid IK solution"],
                        "ik": ik,
                        "segments": segment_summaries,
                    }
                selected_branch = ik["selected_branch"] or waypoint_branch
                segment = build_joint_trajectory(
                    current,
                    [float(value) for value in ik["selected"]["angles_deg"]],
                    joints,
                    waypoint_settings,
                )
            cartesian_waypoints.append(target)

        if not segment["ok"]:
            return {
                "ok": False,
                "mode": "program",
                "waypoints": combined_waypoints,
                "errors": [f"program waypoint {index + 1}: {'; '.join(segment.get('errors', []))}"],
                "segments": segment_summaries,
            }

        _append_segment(combined_waypoints, combined_durations, segment)
        current = [float(value) for value in segment["waypoints"][-1]]
        segment_summaries.append(
            {
                "index": index,
                "type": kind,
                "mode": segment.get("mode", mode),
                "duration_s": segment.get("duration_s", 0.0),
                "waypoint_count": segment.get("waypoint_count", 0),
                "profile": segment.get("profile", _profile_name(waypoint_settings)),
            }
        )

    return {
        "ok": True,
        "mode": "program",
        "profile": _profile_name(settings),
        "duration_s": sum(combined_durations),
        "waypoint_count": len(combined_waypoints),
        "waypoints": combined_waypoints,
        "segment_durations_s": combined_durations,
        "time_from_start_s": _cumulative_times(combined_durations),
        "segments": segment_summaries,
        "cartesian_waypoints": cartesian_waypoints,
        "errors": [],
    }
