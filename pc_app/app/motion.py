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


def _joint_duration_components(
    delta_deg: float,
    speed_limit_deg_s: float,
    accel_limit_deg_s2: float,
    profile: str,
    ramp_fraction: float,
) -> dict[str, float]:
    delta = abs(float(delta_deg))
    speed = max(float(speed_limit_deg_s), 1e-6)
    accel = max(float(accel_limit_deg_s2), 1e-6)
    if delta <= 1e-9:
        return {"speed_duration_s": 0.0, "accel_duration_s": 0.0, "duration_s": 0.0}
    if profile == "linear":
        speed_time = delta / speed
        accel_time = 0.0
    elif profile == "trapezoid":
        ramp = min(0.45, max(0.05, ramp_fraction))
        speed_time = delta / (speed * (1.0 - ramp))
        accel_time = sqrt(delta / (accel * ramp * (1.0 - ramp)))
    else:
        speed_time = 1.9 * delta / speed
        accel_time = 2.5 * sqrt(delta / accel)
    return {
        "speed_duration_s": speed_time,
        "accel_duration_s": accel_time,
        "duration_s": max(speed_time, accel_time),
    }


def _profile_notes(profile: str, settings: dict[str, Any]) -> list[str]:
    notes: list[str] = []
    if profile == "s_curve" and (
        settings.get("jerk_percent") is not None or settings.get("blend_percent") is not None
    ):
        notes.append("s_curve uses fixed quintic progress; jerk_percent and blend_percent do not affect timing")
    if profile == "trapezoid":
        notes.append("blend_percent is interpreted as trapezoid ramp fraction, not waypoint blending")
    if profile == "linear":
        notes.append("linear profile uses constant joint interpolation and ignores acceleration shaping")
    return notes


def _motion_limit_summary(
    *,
    path_mode: str,
    target_type: str,
    joints: list[JointConfig],
    deltas_deg: list[float],
    speed_limits_deg_s: list[float],
    accel_limits_deg_s2: list[float],
    profile: str,
    ramp_fraction: float,
    settings: dict[str, Any],
    duration_floor_s: float = 0.0,
    duration_floor_reason: str = "",
) -> dict[str, Any]:
    per_joint: list[dict[str, Any]] = []
    limiting: dict[str, Any] | None = None
    joint_limited_duration = 0.0

    for index, (joint, delta, speed, accel) in enumerate(
        zip(joints, deltas_deg, speed_limits_deg_s, accel_limits_deg_s2, strict=True)
    ):
        components = _joint_duration_components(delta, speed, accel, profile, ramp_fraction)
        duration = float(components["duration_s"])
        if duration <= 1e-9:
            constraint = "none"
        elif components["accel_duration_s"] > components["speed_duration_s"]:
            constraint = "acceleration"
        else:
            constraint = "speed"
        row = {
            "joint_index": index,
            "joint_name": joint.name,
            "delta_deg": abs(float(delta)),
            "speed_limit_deg_s": float(speed),
            "accel_limit_deg_s2": float(accel),
            "speed_duration_s": float(components["speed_duration_s"]),
            "accel_duration_s": float(components["accel_duration_s"]),
            "duration_s": duration,
            "limiting_constraint": constraint,
        }
        per_joint.append(row)
        if duration > joint_limited_duration + 1e-9:
            joint_limited_duration = duration
            limiting = row

    duration_s = joint_limited_duration
    if duration_floor_s > duration_s + 1e-9:
        duration_s = float(duration_floor_s)
        limiting_constraint = {
            "type": duration_floor_reason or "duration_floor",
            "joint_index": None,
            "joint_name": "",
            "duration_s": duration_s,
        }
    elif limiting is not None:
        limiting_constraint = {
            "type": limiting["limiting_constraint"],
            "joint_index": limiting["joint_index"],
            "joint_name": limiting["joint_name"],
            "duration_s": limiting["duration_s"],
        }
    else:
        limiting_constraint = {
            "type": "none",
            "joint_index": None,
            "joint_name": "",
            "duration_s": 0.0,
        }

    return {
        "schema": "motion_limit_summary_v1",
        "path_mode": path_mode,
        "target_type": target_type,
        "profile": profile,
        "duration_s": duration_s,
        "joint_limited_duration_s": joint_limited_duration,
        "duration_floor_s": float(duration_floor_s),
        "duration_floor_reason": duration_floor_reason,
        "limiting_constraint": limiting_constraint,
        "effective_joint_speed_deg_s": [float(value) for value in speed_limits_deg_s],
        "effective_joint_accel_deg_s2": [float(value) for value in accel_limits_deg_s2],
        "trapezoid_ramp_fraction": ramp_fraction if profile == "trapezoid" else None,
        "per_joint": per_joint,
        "notes": _profile_notes(profile, settings),
    }


def _motion_contract(
    *,
    path_mode: str,
    target_type: str,
    profile: str,
    duration_s: float,
    waypoint_count: int,
    limit_summary: dict[str, Any],
) -> dict[str, Any]:
    if path_mode == "linear":
        interpolation = "sampled Cartesian line with IK at each waypoint"
    elif path_mode == "program":
        interpolation = "program sequence of joint or sampled Cartesian segments"
    else:
        interpolation = "joint-space interpolation"
    return {
        "schema": "motion_plan_contract_v1",
        "path_mode": path_mode,
        "target_type": target_type,
        "profile": profile,
        "interpolation": interpolation,
        "duration_s": float(duration_s),
        "waypoint_count": int(waypoint_count),
        "limits": limit_summary,
    }


def _program_limit_summary(
    segment_summaries: list[dict[str, Any]],
    duration_s: float,
    waypoint_count: int,
) -> dict[str, Any]:
    segment_limits = [
        summary.get("limit_summary")
        for summary in segment_summaries
        if isinstance(summary.get("limit_summary"), dict)
    ]
    profiles = sorted({str(summary.get("profile", "")) for summary in segment_summaries if summary.get("profile")})
    limiting_segment = None
    limiting_constraint = {
        "type": "none",
        "joint_index": None,
        "joint_name": "",
        "duration_s": 0.0,
        "segment_index": None,
        "segment_label": "",
    }
    for summary in segment_summaries:
        limit = summary.get("limit_summary")
        if not isinstance(limit, dict):
            continue
        constraint = limit.get("limiting_constraint") or {}
        candidate_duration = float(constraint.get("duration_s") or limit.get("duration_s") or 0.0)
        if limiting_segment is None or candidate_duration > limiting_constraint["duration_s"]:
            limiting_segment = summary
            limiting_constraint = {
                "type": str(constraint.get("type", "none")),
                "joint_index": constraint.get("joint_index"),
                "joint_name": str(constraint.get("joint_name", "")),
                "duration_s": candidate_duration,
                "segment_index": int(summary.get("index", 0)),
                "segment_label": str(summary.get("label", "")),
            }
    return {
        "schema": "motion_limit_summary_v1",
        "path_mode": "program",
        "target_type": "program",
        "profile": profiles[0] if len(profiles) == 1 else "mixed",
        "duration_s": float(duration_s),
        "waypoint_count": int(waypoint_count),
        "limiting_constraint": limiting_constraint,
        "segment_count": len(segment_summaries),
        "segment_limits": segment_limits,
        "notes": [
            "program limits are reported per segment because step-level settings may differ"
        ],
    }


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
        components = _joint_duration_components(delta, speed, accel, profile, ramp_fraction)
        duration = max(duration, float(components["duration_s"]))
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


def ik_selection_policy(
    settings: dict[str, Any] | None = None,
    *,
    linear: bool = False,
) -> dict[str, Any]:
    settings = settings or {}
    max_joint_delta = float(settings.get("ik_max_joint_delta_deg", 270.0))
    max_base_delta = float(settings.get("ik_max_base_delta_deg", 180.0))
    max_tool_winding = float(settings.get("ik_max_tool_winding_delta_deg", 200.0))
    if bool(settings.get("preserve_tool_orientation", False)):
        max_tool_winding = min(max_tool_winding, 60.0)
    if linear:
        jump_limit = float(settings.get("ik_jump_threshold_deg", 35.0))
        max_joint_delta = min(max_joint_delta, jump_limit)
        max_base_delta = min(max_base_delta, jump_limit)
        max_tool_winding = min(max_tool_winding, 45.0)
    return {
        "enabled": True,
        "max_joint_delta_deg": max_joint_delta,
        "max_base_delta_deg": max_base_delta,
        "max_tool_winding_delta_deg": max_tool_winding,
        "joint_weights": [2.5, 1.0, 1.0, 1.0],
        "prefer_forward_posture": bool(settings.get("prefer_forward_posture", True)),
    }


def _select_continuous_ik_candidate(ik: dict[str, Any], previous_deg: list[float], branch: str) -> dict[str, Any] | None:
    if branch in {"elbow_up", "elbow_down"}:
        return ik.get("selected")
    valid_candidates = [
        candidate
        for candidate in ik.get("candidates", [])
        if candidate.get("valid") and candidate.get("configuration_continuous", True)
    ]
    if not valid_candidates:
        return None
    return min(
        valid_candidates,
        key=lambda candidate: (
            1 if candidate.get("posture") == "backward" else 0,
            sum(
                abs(float(angle) - float(previous_deg[index]))
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
    deltas = [abs(end - start) for start, end in zip(start_deg, target_deg, strict=True)]
    duration_s = _joint_delta_duration_s(deltas, speed_limits, accel_limits, profile, ramp_fraction)
    limit_summary = _motion_limit_summary(
        path_mode="joint",
        target_type="joint_angles",
        joints=joints,
        deltas_deg=deltas,
        speed_limits_deg_s=speed_limits,
        accel_limits_deg_s2=accel_limits,
        profile=profile,
        ramp_fraction=ramp_fraction,
        settings=settings,
    )
    if duration_s <= 1e-9:
        contract = _motion_contract(
            path_mode="joint",
            target_type="joint_angles",
            profile=profile,
            duration_s=0.0,
            waypoint_count=1,
            limit_summary=limit_summary,
        )
        return {
            "ok": True,
            "mode": "joint",
            "profile": profile,
            "duration_s": 0.0,
            "waypoint_count": 1,
            "waypoints": [target_deg],
            "segment_durations_s": [0.0],
            "time_from_start_s": [0.0],
            "speed_limits_deg_s": speed_limits,
            "accel_limits_deg_s2": accel_limits,
            "limit_summary": limit_summary,
            "motion_contract": contract,
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
    contract = _motion_contract(
        path_mode="joint",
        target_type="joint_angles",
        profile=profile,
        duration_s=duration_s,
        waypoint_count=len(waypoints),
        limit_summary=limit_summary,
    )
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
        "limit_summary": limit_summary,
        "motion_contract": contract,
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
        auto_target = {
            "x_mm": float(target.get("x_mm", start_fk["x_mm"])),
            "y_mm": float(target.get("y_mm", start_fk["y_mm"])),
            "z_mm": float(target.get("z_mm", start_fk["z_mm"])),
            "phi_auto": True,
        }
        if target.get("preferred_phi_deg") is not None:
            auto_target["preferred_phi_deg"] = float(target["preferred_phi_deg"])
        final_ik = inverse_kinematics(
            auto_target,
            links,
            joints,
            start_deg,
            branch,
            selection_policy=ik_selection_policy(settings),
        )
        if not final_ik["ok"] or not final_ik["selected"]:
            failure_reason = final_ik.get("failure_reason") or "linear target has no reachable auto-phi solution"
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
                "errors": [failure_reason],
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
    phi_distance_deg = angle_distance_deg(end_pose["phi_deg"], start_pose["phi_deg"])
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
    if distance_mm <= 1e-9 and phi_distance_deg <= 1e-9:
        path_deltas = [0.0 for _ in start_deg]
        limit_summary = _motion_limit_summary(
            path_mode="linear",
            target_type="cartesian_pose",
            joints=joints,
            deltas_deg=path_deltas,
            speed_limits_deg_s=speed_limits,
            accel_limits_deg_s2=accel_limits,
            profile=profile,
            ramp_fraction=ramp_fraction,
            settings=settings,
        )
        contract = _motion_contract(
            path_mode="linear",
            target_type="cartesian_pose",
            profile=profile,
            duration_s=0.0,
            waypoint_count=1,
            limit_summary=limit_summary,
        )
        return {
            "ok": True,
            "mode": "linear",
            "profile": profile,
            "duration_s": 0.0,
            "waypoint_count": 1,
            "waypoints": [[float(value) for value in start_deg]],
            "cartesian_waypoints": [start_pose],
            "segment_durations_s": [0.0],
            "time_from_start_s": [0.0],
            "speed_limits_deg_s": speed_limits,
            "accel_limits_deg_s2": accel_limits,
            "limit_summary": limit_summary,
            "motion_contract": contract,
            "ik_results": [],
            "errors": [],
        }
    max_step_mm = _limit_value(settings.get("cartesian_step_mm"), 10.0)
    steps = max(2, int(ceil(distance_mm / max_step_mm)) + 1)
    if distance_mm <= 1e-9 and phi_distance_deg > 1e-9:
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
        ik = inverse_kinematics(
            pose,
            links,
            joints,
            previous,
            ik_branch,
            selection_policy=ik_selection_policy(settings, linear=True),
        )
        selected = _select_continuous_ik_candidate(ik, previous, ik_branch)
        ik_results.append(
            {
                "index": index,
                "ok": selected is not None,
                "selected_branch": selected["branch"] if selected else ik["selected_branch"],
                "selected_posture": selected.get("posture") if selected else None,
                "radial_reach_mm": selected.get("radial_reach_mm") if selected else None,
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

    path_deltas = _joint_path_deltas(waypoints)
    duration_floor_s = (len(waypoints) - 1) / waypoint_rate_hz
    duration_s = _joint_delta_duration_s(path_deltas, speed_limits, accel_limits, profile, ramp_fraction)
    duration_s = max(duration_s, duration_floor_s)
    limit_summary = _motion_limit_summary(
        path_mode="linear",
        target_type="cartesian_pose",
        joints=joints,
        deltas_deg=path_deltas,
        speed_limits_deg_s=speed_limits,
        accel_limits_deg_s2=accel_limits,
        profile=profile,
        ramp_fraction=ramp_fraction,
        settings=settings,
        duration_floor_s=duration_floor_s,
        duration_floor_reason="waypoint_rate",
    )
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

    contract = _motion_contract(
        path_mode="linear",
        target_type="cartesian_pose",
        profile=profile,
        duration_s=duration_s,
        waypoint_count=len(waypoints),
        limit_summary=limit_summary,
    )
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
        "limit_summary": limit_summary,
        "motion_contract": contract,
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


def _waypoint_label(waypoint: dict[str, Any], index: int) -> str:
    label = waypoint.get("label") or waypoint.get("name") or waypoint.get("kind")
    return str(label) if label else f"waypoint {index + 1}"


def _format_cartesian_target(target: dict[str, Any]) -> str:
    parts = [
        f"x {float(target.get('x_mm', target.get('x', 0.0))):.1f}",
        f"y {float(target.get('y_mm', target.get('y', 0.0))):.1f}",
        f"z {float(target.get('z_mm', target.get('z', 0.0))):.1f}",
    ]
    if target.get("preferred_phi_deg") is not None:
        parts.append(f"preferred phi {float(target.get('preferred_phi_deg')):.1f}")
    elif target.get("phi_auto"):
        parts.append("phi auto")
    elif target.get("phi_deg") is not None or target.get("phi") is not None:
        parts.append(f"phi {float(target.get('phi_deg', target.get('phi'))):.1f}")
    return ", ".join(parts)


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
        return {
            "ok": False,
            "mode": "program",
            "step_count": 0,
            "move_count": 0,
            "waypoints": [],
            "step_results": [],
            "errors": ["program has no waypoints"],
        }

    current = [float(value) for value in start_deg]
    selected_branch = branch
    combined_waypoints: list[list[float]] = []
    combined_durations: list[float] = []
    segment_summaries: list[dict[str, Any]] = []
    cartesian_waypoints: list[dict[str, float]] = []
    step_results: list[dict[str, Any]] = []
    enabled_count = sum(1 for waypoint in waypoints if waypoint.get("enabled", True) is not False)
    move_count = sum(
        1
        for waypoint in waypoints
        if waypoint.get("enabled", True) is not False
        and str(waypoint.get("type") or waypoint.get("kind") or "cartesian").lower() in {"joint", "cartesian"}
    )
    action_count = enabled_count - move_count
    execution_steps: list[dict[str, Any]] = []
    if enabled_count == 0:
        return {
            "ok": False,
            "mode": "program",
            "step_count": len(waypoints),
            "move_count": 0,
            "action_count": 0,
            "waypoints": [],
            "step_results": [
                {
                    "index": index,
                    "label": _waypoint_label(waypoint, index),
                    "type": str(waypoint.get("type") or waypoint.get("kind") or "cartesian").lower(),
                    "mode": str(waypoint.get("mode") or "joint").lower(),
                    "enabled": False,
                    "status": "disabled",
                    "duration_s": 0.0,
                    "waypoint_count": 0,
                    "errors": [],
                }
                for index, waypoint in enumerate(waypoints)
            ],
            "errors": ["program has no enabled steps"],
        }

    for index, waypoint in enumerate(waypoints):
        step_started_s = sum(combined_durations)
        waypoint_label = _waypoint_label(waypoint, index)
        waypoint_settings = _settings_for_waypoint(settings, waypoint)
        kind = str(waypoint.get("type") or waypoint.get("kind") or "cartesian").lower()
        mode = str(waypoint.get("mode") or ("linear" if kind == "cartesian" else "joint")).lower()
        if waypoint.get("enabled", True) is False:
            step_results.append(
                {
                    "index": index,
                    "label": waypoint_label,
                    "type": kind,
                    "mode": mode,
                    "enabled": False,
                    "status": "disabled",
                    "duration_s": 0.0,
                    "start_time_s": step_started_s,
                    "end_time_s": step_started_s,
                    "waypoint_count": 0,
                    "errors": [],
                }
            )
            continue
        if kind == "tool":
            action = str(waypoint.get("action") or "").strip().lower()
            if action not in {"open", "close", "set", "on", "off"}:
                error = f"unsupported end-effector action {action or '(empty)'}"
                step_results.append(
                    {
                        "index": index,
                        "label": waypoint_label,
                        "type": kind,
                        "mode": "tool",
                        "enabled": True,
                        "status": "invalid",
                        "duration_s": 0.0,
                        "start_time_s": step_started_s,
                        "end_time_s": step_started_s,
                        "waypoint_count": 0,
                        "errors": [error],
                    }
                )
                return {
                    "ok": False,
                    "mode": "program",
                    "step_count": len(waypoints),
                    "move_count": move_count,
                    "action_count": action_count,
                    "waypoints": combined_waypoints,
                    "errors": [f"program step {index + 1} ({waypoint_label}): {error}"],
                    "segments": segment_summaries,
                    "step_results": step_results,
                    "execution_steps": execution_steps,
                }
            value = waypoint.get("value")
            if action == "set":
                try:
                    value = float(value)
                except (TypeError, ValueError):
                    value = None
                if value is None or not 0.0 <= value <= 1.0:
                    error = "set action value must be between 0 and 1"
                    step_results.append(
                        {
                            "index": index,
                            "label": waypoint_label,
                            "type": kind,
                            "mode": "tool",
                            "enabled": True,
                            "status": "invalid",
                            "duration_s": 0.0,
                            "start_time_s": step_started_s,
                            "end_time_s": step_started_s,
                            "waypoint_count": 0,
                            "errors": [error],
                        }
                    )
                    return {
                        "ok": False,
                        "mode": "program",
                        "step_count": len(waypoints),
                        "move_count": move_count,
                        "action_count": action_count,
                        "waypoints": combined_waypoints,
                        "errors": [f"program step {index + 1} ({waypoint_label}): {error}"],
                        "segments": segment_summaries,
                        "step_results": step_results,
                        "execution_steps": execution_steps,
                    }
            try:
                settle_ms = float(waypoint.get("settle_ms", 150.0))
            except (TypeError, ValueError):
                settle_ms = -1.0
            if settle_ms < 0.0:
                error = "settle_ms must be zero or greater"
                step_results.append(
                    {
                        "index": index,
                        "label": waypoint_label,
                        "type": kind,
                        "mode": "tool",
                        "enabled": True,
                        "status": "invalid",
                        "duration_s": 0.0,
                        "start_time_s": step_started_s,
                        "end_time_s": step_started_s,
                        "waypoint_count": 0,
                        "errors": [error],
                    }
                )
                return {
                    "ok": False,
                    "mode": "program",
                    "step_count": len(waypoints),
                    "move_count": move_count,
                    "action_count": action_count,
                    "waypoints": combined_waypoints,
                    "errors": [f"program step {index + 1} ({waypoint_label}): {error}"],
                    "segments": segment_summaries,
                    "step_results": step_results,
                    "execution_steps": execution_steps,
                }
            duration_s = settle_ms / 1000.0
            if not combined_waypoints:
                combined_waypoints.append(current.copy())
                combined_durations.append(0.0)
            combined_waypoints.append(current.copy())
            combined_durations.append(duration_s)
            summary = {
                "index": index,
                "label": waypoint_label,
                "type": kind,
                "mode": action,
                "duration_s": duration_s,
                "start_time_s": step_started_s,
                "end_time_s": step_started_s + duration_s,
                "waypoint_count": 0,
                "profile": "tool_action",
                "limit_summary": None,
                "motion_contract": None,
                "action": action,
                "tool": waypoint.get("tool"),
                "value": value,
            }
            segment_summaries.append(summary)
            step_results.append({**summary, "enabled": True, "status": "valid", "errors": []})
            execution_steps.append(
                {
                    "kind": "tool",
                    "index": index,
                    "label": waypoint_label,
                    "action": action,
                    "tool": waypoint.get("tool"),
                    "value": value,
                    "duration_s": duration_s,
                }
            )
            continue
        if kind not in {"joint", "cartesian"}:
            error = f"unsupported program step type {kind}"
            step_results.append(
                {
                    "index": index,
                    "label": waypoint_label,
                    "type": kind,
                    "mode": mode,
                    "enabled": True,
                    "status": "invalid",
                    "duration_s": 0.0,
                    "start_time_s": step_started_s,
                    "end_time_s": step_started_s,
                    "waypoint_count": 0,
                    "errors": [error],
                }
            )
            return {
                "ok": False,
                "mode": "program",
                "step_count": len(waypoints),
                "move_count": move_count,
                "action_count": action_count,
                "waypoints": combined_waypoints,
                "errors": [f"program step {index + 1} ({waypoint_label}): {error}"],
                "segments": segment_summaries,
                "step_results": step_results,
                "execution_steps": execution_steps,
            }
        explicit_waypoint_branch = waypoint.get("branch")
        if explicit_waypoint_branch is not None:
            waypoint_branch = str(explicit_waypoint_branch)
        elif mode == "linear":
            waypoint_branch = str(selected_branch or branch or "auto")
        else:
            # A joint-space transfer may legitimately move to another IK
            # branch. Preserve branch continuity only inside linear segments
            # unless the caller explicitly requests a branch.
            waypoint_branch = str(branch or "auto")

        if kind == "joint":
            target_angles = waypoint.get("angles_deg") or waypoint.get("joints_deg")
            if isinstance(waypoint.get("target"), dict):
                target_angles = target_angles or waypoint["target"].get("angles_deg")
            if not isinstance(target_angles, list):
                error = f"program waypoint {index + 1} ({waypoint_label}) missing joint angles"
                step_results.append(
                    {
                        "index": index,
                        "label": waypoint_label,
                        "type": kind,
                        "mode": "joint",
                        "enabled": True,
                        "status": "invalid",
                        "duration_s": 0.0,
                        "start_time_s": step_started_s,
                        "end_time_s": step_started_s,
                        "waypoint_count": 0,
                        "errors": ["missing joint angles"],
                    }
                )
                return {
                    "ok": False,
                    "mode": "program",
                    "step_count": len(waypoints),
                    "move_count": move_count,
                    "action_count": action_count,
                    "waypoints": combined_waypoints,
                    "errors": [error],
                    "segments": segment_summaries,
                    "step_results": step_results,
                    "execution_steps": execution_steps,
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
                if raw_target.get("preferred_phi_deg") is not None:
                    target["preferred_phi_deg"] = float(raw_target["preferred_phi_deg"])
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
                ik = inverse_kinematics(
                    target,
                    links,
                    joints,
                    current,
                    waypoint_branch,
                    selection_policy=ik_selection_policy(waypoint_settings),
                )
                if not ik["ok"] or not ik["selected"]:
                    error = ik.get("failure_reason") or f"no valid IK solution at {_format_cartesian_target(target)}"
                    step_results.append(
                        {
                            "index": index,
                            "label": waypoint_label,
                            "type": kind,
                            "mode": mode,
                            "enabled": True,
                            "status": "invalid",
                            "duration_s": 0.0,
                            "start_time_s": step_started_s,
                            "end_time_s": step_started_s,
                            "waypoint_count": 0,
                            "errors": [error],
                        }
                    )
                    return {
                        "ok": False,
                        "mode": "program",
                        "step_count": len(waypoints),
                        "move_count": move_count,
                        "action_count": action_count,
                        "waypoints": combined_waypoints,
                        "errors": [
                            f"program waypoint {index + 1} ({waypoint_label}) has {error}"
                        ],
                        "ik": ik,
                        "segments": segment_summaries,
                        "step_results": step_results,
                        "execution_steps": execution_steps,
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
            segment_errors = [str(error) for error in segment.get("errors", [])]
            step_results.append(
                {
                    "index": index,
                    "label": waypoint_label,
                    "type": kind,
                    "mode": mode,
                    "enabled": True,
                    "status": "invalid",
                    "duration_s": 0.0,
                    "start_time_s": step_started_s,
                    "end_time_s": step_started_s,
                    "waypoint_count": 0,
                    "errors": segment_errors,
                }
            )
            return {
                "ok": False,
                "mode": "program",
                "step_count": len(waypoints),
                "move_count": move_count,
                "action_count": action_count,
                "waypoints": combined_waypoints,
                "errors": [f"program waypoint {index + 1} ({waypoint_label}): {'; '.join(segment_errors)}"],
                "segments": segment_summaries,
                "step_results": step_results,
                "execution_steps": execution_steps,
            }

        _append_segment(combined_waypoints, combined_durations, segment)
        current = [float(value) for value in segment["waypoints"][-1]]
        summary = {
            "index": index,
            "label": waypoint_label,
            "type": kind,
            "mode": segment.get("mode", mode),
            "duration_s": segment.get("duration_s", 0.0),
            "start_time_s": step_started_s,
            "end_time_s": step_started_s + float(segment.get("duration_s", 0.0)),
            "waypoint_count": segment.get("waypoint_count", 0),
            "profile": segment.get("profile", _profile_name(waypoint_settings)),
            "limit_summary": segment.get("limit_summary"),
            "motion_contract": segment.get("motion_contract"),
            "settings": waypoint_settings,
        }
        segment_summaries.append(summary)
        step_results.append(
            {
                **summary,
                "enabled": True,
                "status": "valid",
                "errors": [],
            }
        )
        execution_steps.append(
            {
                "kind": "motion",
                "index": index,
                "label": waypoint_label,
                "settings": waypoint_settings,
                "trajectory": segment,
            }
        )

    duration_s = sum(combined_durations)
    limit_summary = _program_limit_summary(segment_summaries, duration_s, len(combined_waypoints))
    contract = _motion_contract(
        path_mode="program",
        target_type="program",
        profile=str(limit_summary.get("profile", _profile_name(settings))),
        duration_s=duration_s,
        waypoint_count=len(combined_waypoints),
        limit_summary=limit_summary,
    )
    return {
        "ok": True,
        "mode": "program",
        "step_count": len(waypoints),
        "move_count": move_count,
        "action_count": action_count,
        "profile": _profile_name(settings),
        "duration_s": duration_s,
        "waypoint_count": len(combined_waypoints),
        "waypoints": combined_waypoints,
        "segment_durations_s": combined_durations,
        "time_from_start_s": _cumulative_times(combined_durations),
        "segments": segment_summaries,
        "step_results": step_results,
        "execution_steps": execution_steps,
        "cartesian_waypoints": cartesian_waypoints,
        "limit_summary": limit_summary,
        "motion_contract": contract,
        "errors": [],
    }
