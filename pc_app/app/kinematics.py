from __future__ import annotations

from math import acos, atan2, cos, degrees, hypot, isfinite, radians, sin, sqrt
from typing import Any

import numpy as np

from .config import DHRowConfig, JointConfig, LinkConfig


COORDINATE_FRAME = """
Standard DH kinematics with the project robot frame mapped from the DH frame:
- The DH table uses theta, d, a, alpha in the standard convention.
- Joint values are UI/API degrees and are added to each row's theta_offset_deg.
- Robot coordinates are mapped as robot x = DH y, robot y = -DH x, robot z = DH z.
- At base/theta1 = 0 deg, positive planar reach points along global +Y.
- +Z points upward from the mounting plane.
- Working assumption for the measured L convention: d1=L1+L3, base side
  offset=L2, d2=s4*L4, a2=L5, d3=s6*L6, a3=L7, d4=s8*L8, a4=L9.
- tool_phi_deg is the first-pass pitch task angle theta2 + theta3 + theta4 after direction and zero offsets.
- The active tool TCP offset is applied after the final DH joint transform.
- Tool TCP +Z is treated as the tool-forward axis, which maps to local DH +X.
""".strip()

CARTESIAN_POSITION_PROGRESS_MIN_FRACTION = 0.20
CARTESIAN_DIRECTION_ALIGNMENT_MIN = 0.95


def _default_rows(links: LinkConfig) -> list[DHRowConfig]:
    return [
        DHRowConfig(0, 0.0, links.base_height_mm, 0.0, 90.0),
        DHRowConfig(1, 90.0, 0.0, links.upper_arm_mm, 0.0),
        DHRowConfig(2, 0.0, 0.0, links.forearm_mm, 0.0),
        DHRowConfig(3, 0.0, 0.0, links.wrist_mm + links.tool_mm, 0.0),
    ]


def _rows(links: LinkConfig) -> list[DHRowConfig]:
    return links.dh_rows if links.dh_rows else _default_rows(links)


def _normalize_deg(angle: float) -> float:
    normalized = (angle + 180.0) % 360.0 - 180.0
    if normalized == -180.0 and angle > 0:
        return 180.0
    return normalized


def angle_distance_deg(a: float, b: float) -> float:
    return abs(_normalize_deg(a - b))


def _signed_angle_error_deg(target: float, actual: float) -> float:
    return _normalize_deg(target - actual)


def _dh_matrix(theta_deg: float, d_mm: float, a_mm: float, alpha_deg: float) -> np.ndarray:
    return (
        _rotation_z(theta_deg)
        @ _translation(0.0, 0.0, d_mm)
        @ _translation(a_mm, 0.0, 0.0)
        @ _rotation_x(alpha_deg)
    )


def _rotation_z(theta_deg: float) -> np.ndarray:
    theta = radians(theta_deg)
    ct = cos(theta)
    st = sin(theta)
    return np.array(
        [
            [ct, -st, 0.0, 0.0],
            [st, ct, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _rotation_x(alpha_deg: float) -> np.ndarray:
    alpha = radians(alpha_deg)
    ca = cos(alpha)
    sa = sin(alpha)
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, ca, -sa, 0.0],
            [0.0, sa, ca, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _translation(x_mm: float, y_mm: float, z_mm: float) -> np.ndarray:
    transform = np.identity(4)
    transform[0, 3] = x_mm
    transform[1, 3] = y_mm
    transform[2, 3] = z_mm
    return transform


def _dh_step(
    transform: np.ndarray,
    theta_deg: float,
    d_mm: float,
    a_mm: float,
    alpha_deg: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    after_theta = transform @ _rotation_z(theta_deg)
    after_d = after_theta @ _translation(0.0, 0.0, d_mm)
    after_a = after_d @ _translation(a_mm, 0.0, 0.0)
    final = after_a @ _rotation_x(alpha_deg)
    return final, after_d, after_a


def _dh_step_with_side_offset(
    transform: np.ndarray,
    theta_deg: float,
    d_mm: float,
    a_mm: float,
    alpha_deg: float,
    side_offset_mm: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    after_theta = transform @ _rotation_z(theta_deg)
    after_d = after_theta @ _translation(0.0, 0.0, d_mm)
    after_side = after_d @ _translation(0.0, side_offset_mm, 0.0)
    after_a = after_side @ _translation(a_mm, 0.0, 0.0)
    final = after_a @ _rotation_x(alpha_deg)
    return final, after_d, after_side, after_a


def _robot_point_from_dh(vector: np.ndarray) -> dict[str, float]:
    return {"x_mm": float(vector[1]), "y_mm": float(-vector[0]), "z_mm": float(vector[2])}


def _row_theta(row: DHRowConfig, joint_angles_deg: list[float]) -> float:
    joint_angle = float(joint_angles_deg[row.joint_index])
    return joint_angle * row.direction_sign + row.zero_offset_deg + row.theta_offset_deg


def dh_transforms(joint_angles_deg: list[float], links: LinkConfig) -> list[np.ndarray]:
    if len(joint_angles_deg) != 4:
        raise ValueError("DH kinematics expects four joint angles")
    transforms: list[np.ndarray] = []
    transform = np.identity(4)
    transforms.append(transform.copy())
    for row_index, row in enumerate(_rows(links)):
        side_offset = links.base_side_offset_mm if row_index == 0 else 0.0
        transform, _, _, _ = _dh_step_with_side_offset(
            transform,
            _row_theta(row, joint_angles_deg),
            row.d_mm,
            row.a_mm,
            row.alpha_deg,
            side_offset,
        )
        transforms.append(transform.copy())
    return transforms


def joint_frame_points(joint_angles_deg: list[float], links: LinkConfig) -> list[dict[str, float]]:
    return [_robot_point_from_dh(transform[:3, 3]) for transform in dh_transforms(joint_angles_deg, links)]


def _segment_label(row_index: int, kind: str) -> str:
    labels = [
        {"d": "L1+L3", "side": "L2", "a": "a1"},
        {"d": "s4*L4", "a": "L5"},
        {"d": "s6*L6", "a": "L7"},
        {"d": "s8*L8", "a": "L9"},
    ]
    if row_index < len(labels):
        return labels[row_index].get(kind, f"{kind}{row_index + 1}")
    return f"{kind}{row_index + 1}"


def dh_segment_points(joint_angles_deg: list[float], links: LinkConfig) -> list[dict[str, Any]]:
    """Return Standard DH translation segments as separate d and a offsets."""
    if len(joint_angles_deg) != 4:
        raise ValueError("DH segment visualization expects four joint angles")
    transform = np.identity(4)
    segments: list[dict[str, Any]] = []
    for row_index, row in enumerate(_rows(links)):
        side_offset = links.base_side_offset_mm if row_index == 0 else 0.0
        final, after_d, after_side, after_a = _dh_step_with_side_offset(
            transform,
            _row_theta(row, joint_angles_deg),
            row.d_mm,
            row.a_mm,
            row.alpha_deg,
            side_offset,
        )
        start = _robot_point_from_dh(transform[:3, 3])
        d_end = _robot_point_from_dh(after_d[:3, 3])
        side_end = _robot_point_from_dh(after_side[:3, 3])
        a_end = _robot_point_from_dh(after_a[:3, 3])
        if abs(row.d_mm) > 1e-9:
            segments.append(
                {
                    "kind": "d",
                    "row": row_index + 1,
                    "joint": row.joint_index + 1,
                    "label": _segment_label(row_index, "d"),
                    "signed_length_mm": float(row.d_mm),
                    "length_mm": abs(float(row.d_mm)),
                    "start": start,
                    "end": d_end,
                }
            )
        if abs(side_offset) > 1e-9:
            segments.append(
                {
                    "kind": "side",
                    "row": row_index + 1,
                    "joint": row.joint_index + 1,
                    "label": _segment_label(row_index, "side"),
                    "signed_length_mm": float(side_offset),
                    "length_mm": abs(float(side_offset)),
                    "start": d_end,
                    "end": side_end,
                }
            )
        if abs(row.a_mm) > 1e-9:
            segments.append(
                {
                    "kind": "a",
                    "row": row_index + 1,
                    "joint": row.joint_index + 1,
                    "label": _segment_label(row_index, "a"),
                    "signed_length_mm": float(row.a_mm),
                    "length_mm": abs(float(row.a_mm)),
                    "start": side_end,
                    "end": a_end,
                }
            )
        transform = final
    return segments


def _tool_phi_from_angles(joint_angles_deg: list[float], links: LinkConfig) -> float:
    rows = _rows(links)
    pitch = 0.0
    for row in rows:
        if row.joint_index > 0:
            pitch += float(joint_angles_deg[row.joint_index]) * row.direction_sign + row.zero_offset_deg
    return _normalize_deg(pitch)


def _tool_tcp_offset_vector(links: LinkConfig) -> np.ndarray:
    offset = links.tool_tcp_offset_mm or {}
    tool_x = float(offset.get("x", offset.get("x_mm", 0.0)))
    tool_y = float(offset.get("y", offset.get("y_mm", 0.0)))
    tool_z = float(offset.get("z", offset.get("z_mm", 0.0)))
    # The UI/config uses tool +Z as the forward TCP axis. Standard DH link
    # extension uses local +X, so map tool-frame offsets into the final DH frame.
    return np.array(
        [
            tool_z,
            tool_x,
            tool_y,
            1.0,
        ],
        dtype=float,
    )


def forward_kinematics(joint_angles_deg: list[float], links: LinkConfig) -> dict[str, Any]:
    frames = dh_transforms(joint_angles_deg, links)
    wrist = _robot_point_from_dh(frames[-1][:3, 3])
    tcp_vector = frames[-1] @ _tool_tcp_offset_vector(links)
    tcp = _robot_point_from_dh(tcp_vector[:3])
    phi_deg = _tool_phi_from_angles(joint_angles_deg, links)
    tcp["radial_mm"] = hypot(tcp["x_mm"], tcp["y_mm"])
    tcp["tool_phi_deg"] = phi_deg
    tcp["tool_pitch_deg"] = phi_deg
    tcp["dh_frames"] = joint_frame_points(joint_angles_deg, links)
    tcp["dh_segments"] = dh_segment_points(joint_angles_deg, links)
    tcp["wrist_frame"] = wrist
    tcp["tool_tcp_offset_mm"] = dict(links.tool_tcp_offset_mm or {})
    return tcp


def _joint_limit_reasons(joints: list[JointConfig], angles_deg: list[float]) -> list[str]:
    reasons: list[str] = []
    for joint, angle in zip(joints, angles_deg, strict=True):
        if angle < joint.min_deg or angle > joint.max_deg:
            reasons.append(
                f"{joint.name} {angle:.2f} deg outside {joint.min_deg:.2f}..{joint.max_deg:.2f} deg"
            )
    return reasons


def _task_error(target: dict[str, float], angles_deg: list[float], links: LinkConfig) -> np.ndarray:
    fk = forward_kinematics(angles_deg, links)
    return np.array(
        [
            float(target["x_mm"]) - fk["x_mm"],
            float(target["y_mm"]) - fk["y_mm"],
            float(target["z_mm"]) - fk["z_mm"],
            _signed_angle_error_deg(float(target["phi_deg"]), fk["tool_phi_deg"]),
        ],
        dtype=float,
    )


def _numeric_jacobian(target: dict[str, float], angles_deg: list[float], links: LinkConfig) -> np.ndarray:
    del target
    base_fk = forward_kinematics(angles_deg, links)
    base_vector = np.array(
        [base_fk["x_mm"], base_fk["y_mm"], base_fk["z_mm"], base_fk["tool_phi_deg"]],
        dtype=float,
    )
    jacobian = np.zeros((4, len(angles_deg)), dtype=float)
    eps = 0.05
    for index in range(len(angles_deg)):
        shifted = angles_deg.copy()
        shifted[index] += eps
        shifted_fk = forward_kinematics(shifted, links)
        shifted_vector = np.array(
            [
                shifted_fk["x_mm"],
                shifted_fk["y_mm"],
                shifted_fk["z_mm"],
                shifted_fk["tool_phi_deg"],
            ],
            dtype=float,
        )
        diff = shifted_vector - base_vector
        diff[3] = _normalize_deg(shifted_fk["tool_phi_deg"] - base_fk["tool_phi_deg"])
        jacobian[:, index] = diff / eps
    return jacobian


def differential_ik_step(
    current_joints_deg: list[float],
    task_delta: dict[str, float] | list[float] | tuple[float, ...],
    links: LinkConfig,
    joints: list[JointConfig],
    damping: float = 0.35,
    max_joint_step_deg: float = 8.0,
) -> dict[str, Any]:
    """Solve one small Cartesian jog step with damped least-squares IK.

    The task vector is [x_mm, y_mm, z_mm, phi_deg]. This is intended for
    resolved-rate jogging; callers still need to apply velocity, acceleration,
    stale-input, and hardware safety policy around it.
    """
    if len(current_joints_deg) != len(joints):
        return {"ok": False, "error": f"expected {len(joints)} joint angles"}
    if isinstance(task_delta, dict):
        delta = np.array(
            [
                float(task_delta.get("x_mm", task_delta.get("x", 0.0))),
                float(task_delta.get("y_mm", task_delta.get("y", 0.0))),
                float(task_delta.get("z_mm", task_delta.get("z", 0.0))),
                float(task_delta.get("phi_deg", task_delta.get("phi", 0.0))),
            ],
            dtype=float,
        )
    else:
        if len(task_delta) != 4:
            return {"ok": False, "error": "task_delta must have four values"}
        delta = np.array([float(value) for value in task_delta], dtype=float)
    if not np.all(np.isfinite(delta)):
        return {"ok": False, "error": "task_delta contains a non-finite value"}

    current = _clamp_to_limits([float(value) for value in current_joints_deg], joints)
    jacobian = _numeric_jacobian({}, current, links)
    active_rows: list[int] = []
    if float(np.linalg.norm(delta[:3])) > 1e-9:
        active_rows.extend([0, 1, 2])
    if abs(float(delta[3])) > 1e-9:
        active_rows.append(3)
    if not active_rows:
        current_fk = forward_kinematics(current, links)
        return {
            "ok": True,
            "target_angles_deg": current,
            "joint_delta_deg": [0.0 for _ in current],
            "raw_joint_delta_deg": [0.0 for _ in current],
            "requested_delta": {"x_mm": 0.0, "y_mm": 0.0, "z_mm": 0.0, "phi_deg": 0.0},
            "achieved_delta": {"x_mm": 0.0, "y_mm": 0.0, "z_mm": 0.0, "phi_deg": 0.0},
            "candidate_achieved_delta": {"x_mm": 0.0, "y_mm": 0.0, "z_mm": 0.0, "phi_deg": 0.0},
            "position_progress_mm": 0.0,
            "position_lateral_mm": 0.0,
            "position_alignment": None,
            "predicted_fk": current_fk,
            "condition": 0.0,
            "singularity_warning": False,
            "blocked": False,
            "notes": [],
        }
    task_jacobian = jacobian[active_rows, :]
    task_delta = delta[active_rows]
    lhs = task_jacobian @ task_jacobian.T + (max(float(damping), 1e-6) ** 2) * np.identity(len(active_rows))
    try:
        condition = float(np.linalg.cond(lhs))
        joint_delta = task_jacobian.T @ np.linalg.solve(lhs, task_delta)
    except np.linalg.LinAlgError:
        return {"ok": False, "error": "differential IK solve failed"}

    raw_joint_delta = joint_delta.copy()
    max_step = max(0.001, float(max_joint_step_deg))
    max_requested_step = float(np.max(np.abs(joint_delta))) if joint_delta.size else 0.0
    joint_step_scale = 1.0
    if max_requested_step > max_step:
        joint_step_scale = max_step / max(max_requested_step, 1e-9)
        joint_delta = joint_delta * joint_step_scale
    unclamped_target = [angle + float(step) for angle, step in zip(current, joint_delta, strict=True)]
    candidate_target = _clamp_to_limits(unclamped_target, joints)
    limit_reasons = _joint_limit_reasons(joints, unclamped_target)
    candidate_fk = forward_kinematics(candidate_target, links)
    current_fk = forward_kinematics(current, links)
    candidate_delta = {
        "x_mm": float(candidate_fk["x_mm"] - current_fk["x_mm"]),
        "y_mm": float(candidate_fk["y_mm"] - current_fk["y_mm"]),
        "z_mm": float(candidate_fk["z_mm"] - current_fk["z_mm"]),
        "phi_deg": float(_signed_angle_error_deg(candidate_fk["tool_phi_deg"], current_fk["tool_phi_deg"])),
    }
    requested_norm = float(np.linalg.norm(delta[:3]))
    achieved_position = np.array([candidate_delta["x_mm"], candidate_delta["y_mm"], candidate_delta["z_mm"]])
    achieved_norm = float(np.linalg.norm(achieved_position))
    position_progress = 0.0
    position_lateral = 0.0
    position_alignment: float | None = None
    if requested_norm > 1e-6:
        requested_direction = delta[:3] / requested_norm
        position_progress = float(achieved_position @ requested_direction)
        lateral_vector = achieved_position - position_progress * requested_direction
        position_lateral = float(np.linalg.norm(lateral_vector))
        position_alignment = position_progress / max(achieved_norm, 1e-9)
    position_blocked = (
        requested_norm > 1e-6
        and position_progress < requested_norm * CARTESIAN_POSITION_PROGRESS_MIN_FRACTION
    )
    direction_blocked = (
        requested_norm > 1e-6
        and achieved_norm > requested_norm * CARTESIAN_POSITION_PROGRESS_MIN_FRACTION
        and position_alignment is not None
        and position_alignment < CARTESIAN_DIRECTION_ALIGNMENT_MIN
    )
    requested_phi = abs(float(delta[3]))
    achieved_phi = abs(float(candidate_delta["phi_deg"]))
    phi_blocked = requested_phi > 1e-6 and achieved_phi < requested_phi * 0.2
    notes: list[str] = []
    failure_code: str | None = None
    failure_reason: str | None = None
    if position_blocked:
        failure_code = "local_step_unreachable"
        failure_reason = "requested Cartesian step is locally unreachable from the current pose"
        notes.append(failure_reason)
    if direction_blocked:
        failure_code = failure_code or "excessive_lateral_drift"
        failure_reason = failure_reason or (
            "near singularity or a joint limit: the requested axis would cause excessive lateral TCP drift"
        )
        notes.append(
            "near singularity or a joint limit: the requested axis would cause excessive lateral TCP drift"
        )
    if phi_blocked:
        failure_code = failure_code or "phi_step_unreachable"
        failure_reason = failure_reason or "requested tool-angle step is locally unreachable from the current pose"
        notes.append("requested tool-angle step is locally unreachable from the current pose")
    if condition > 1e6:
        notes.append("near-singular Jacobian")
    if joint_step_scale < 1.0 - 1e-9:
        notes.append("joint step scaled")
    notes.extend(limit_reasons)
    if limit_reasons and failure_code is None:
        failure_code = "joint_limit"
        failure_reason = limit_reasons[0]
    hard_blocked = position_blocked or direction_blocked or phi_blocked
    target = current if hard_blocked else candidate_target
    predicted_fk = current_fk if hard_blocked else candidate_fk
    achieved_delta = (
        {"x_mm": 0.0, "y_mm": 0.0, "z_mm": 0.0, "phi_deg": 0.0}
        if hard_blocked
        else candidate_delta
    )
    blocked = bool(limit_reasons) or hard_blocked

    return {
        "ok": True,
        "target_angles_deg": target,
        "joint_delta_deg": [float(value) for value in (np.array(target) - np.array(current))],
        "raw_joint_delta_deg": [float(value) for value in raw_joint_delta],
        "requested_delta": {
            "x_mm": float(delta[0]),
            "y_mm": float(delta[1]),
            "z_mm": float(delta[2]),
            "phi_deg": float(delta[3]),
        },
        "achieved_delta": achieved_delta,
        "candidate_achieved_delta": candidate_delta,
        "position_progress_mm": position_progress,
        "position_lateral_mm": position_lateral,
        "position_alignment": position_alignment,
        "predicted_fk": predicted_fk,
        "condition": condition,
        "singularity_warning": condition > 1e6,
        "blocked": blocked,
        "failure_code": failure_code,
        "failure_reason": failure_reason,
        "notes": notes,
    }


def _clamp_to_limits(angles_deg: list[float], joints: list[JointConfig]) -> list[float]:
    return [max(joint.min_deg, min(joint.max_deg, angle)) for angle, joint in zip(angles_deg, joints, strict=True)]


def _solve_from_seed(
    target: dict[str, float],
    links: LinkConfig,
    joints: list[JointConfig],
    seed_deg: list[float],
    label: str,
    max_iterations: int = 160,
    damping: float = 0.18,
    position_tolerance_mm: float = 1.0,
    orientation_tolerance_deg: float = 1.0,
) -> dict[str, Any]:
    angles = _clamp_to_limits([float(value) for value in seed_deg], joints)
    notes: list[str] = []
    iterations = 0
    singular = False

    for iterations in range(1, max_iterations + 1):
        error = _task_error(target, angles, links)
        position_error = float(np.linalg.norm(error[:3]))
        orientation_error = abs(float(error[3]))
        if position_error <= position_tolerance_mm and orientation_error <= orientation_tolerance_deg:
            break

        jacobian = _numeric_jacobian(target, angles, links)
        condition = np.linalg.cond(jacobian @ jacobian.T + np.identity(4) * 1e-9)
        singular = singular or bool(condition > 1e8)
        lhs = jacobian @ jacobian.T + (damping**2) * np.identity(4)
        try:
            delta = jacobian.T @ np.linalg.solve(lhs, error)
        except np.linalg.LinAlgError:
            notes.append("jacobian solve failed")
            break
        delta = np.clip(delta, -8.0, 8.0)
        next_angles = _clamp_to_limits([angle + float(step) for angle, step in zip(angles, delta, strict=True)], joints)
        if all(abs(a - b) < 1e-6 for a, b in zip(next_angles, angles, strict=True)):
            notes.append("solver stalled at a joint limit")
            break
        angles = next_angles

    fk = forward_kinematics(angles, links)
    position_error = hypot(hypot(fk["x_mm"] - target["x_mm"], fk["y_mm"] - target["y_mm"]), fk["z_mm"] - target["z_mm"])
    phi_error = angle_distance_deg(fk["tool_phi_deg"], target["phi_deg"])
    reasons = _joint_limit_reasons(joints, angles)
    if position_error > position_tolerance_mm or phi_error > orientation_tolerance_deg:
        reasons.append(
            f"IK did not converge: position error {position_error:.2f} mm, phi error {phi_error:.2f} deg"
        )
    if singular:
        notes.append("near-singular Jacobian encountered")
    return {
        "branch": label,
        "angles_deg": angles,
        "valid": not reasons,
        "reasons": reasons,
        "fk": fk,
        "position_error_mm": position_error,
        "phi_error_deg": phi_error,
        "iterations": iterations,
        "singularity_warning": singular,
        "notes": notes,
    }


def _candidate_from_angles(
    target: dict[str, float],
    links: LinkConfig,
    joints: list[JointConfig],
    angles_deg: list[float],
    label: str,
    position_tolerance_mm: float = 1.0,
    orientation_tolerance_deg: float = 1.0,
) -> dict[str, Any]:
    angles = [float(value) for value in angles_deg]
    fk = forward_kinematics(angles, links)
    position_error = hypot(hypot(fk["x_mm"] - target["x_mm"], fk["y_mm"] - target["y_mm"]), fk["z_mm"] - target["z_mm"])
    phi_error = angle_distance_deg(fk["tool_phi_deg"], target["phi_deg"])
    reasons = _joint_limit_reasons(joints, angles)
    if position_error > position_tolerance_mm or phi_error > orientation_tolerance_deg:
        reasons.append(
            f"IK did not converge: position error {position_error:.2f} mm, phi error {phi_error:.2f} deg"
        )
    return {
        "branch": label,
        "angles_deg": angles,
        "valid": not reasons,
        "reasons": reasons,
        "fk": fk,
        "position_error_mm": position_error,
        "phi_error_deg": phi_error,
        "iterations": 0,
        "singularity_warning": False,
        "notes": [],
    }


def _joint_angle_from_row_theta(row: DHRowConfig, theta_deg: float) -> float:
    return (theta_deg - row.zero_offset_deg - row.theta_offset_deg) / row.direction_sign


def _analytic_seed_candidates(
    target: dict[str, float],
    links: LinkConfig,
    joints: list[JointConfig],
) -> tuple[list[tuple[str, list[float]]], list[str]]:
    rows = _rows(links)
    notes: list[str] = []
    if len(rows) != 4:
        return [], ["analytic seed skipped: DH model does not have four rows"]
    if abs(rows[0].alpha_deg - 90.0) > 1e-6 or any(abs(row.alpha_deg) > 1e-6 for row in rows[1:]):
        return [], ["analytic seed skipped: DH rows do not match the planar prototype shape"]

    d1 = float(rows[0].d_mm)
    lateral_mm = float(links.base_side_offset_mm) - sum(float(row.d_mm) for row in rows[1:])
    a2 = float(rows[1].a_mm)
    a3 = float(rows[2].a_mm)
    tool_offset = _tool_tcp_offset_vector(links)
    a4 = float(rows[3].a_mm) + float(tool_offset[0])
    phi = radians(float(target["phi_deg"]))

    dh_x = -float(target["y_mm"])
    dh_y = float(target["x_mm"])
    xy_radius = hypot(dh_x, dh_y)
    if xy_radius < abs(lateral_mm) - 1e-6:
        return [], [f"target is inside lateral offset radius {abs(lateral_mm):.2f} mm"]

    radial_abs = sqrt(max(0.0, xy_radius * xy_radius - lateral_mm * lateral_mm))
    radial_signs = [1.0]
    if radial_abs > 1e-6:
        radial_signs.append(-1.0)

    seeds: list[tuple[str, list[float]]] = []
    for radial_sign in radial_signs:
        radial_mm = radial_abs * radial_sign
        base_theta = degrees(atan2(dh_y, dh_x) - atan2(lateral_mm, radial_mm))
        wrist_r = radial_mm - a4 * cos(phi)
        wrist_z = float(target["z_mm"]) - d1 - a4 * sin(phi)
        denom = 2.0 * a2 * a3
        if abs(denom) <= 1e-9:
            notes.append("analytic seed skipped: upper-arm or forearm length is zero")
            continue
        elbow_cos = (wrist_r * wrist_r + wrist_z * wrist_z - a2 * a2 - a3 * a3) / denom
        if elbow_cos < -1.0 - 1e-6 or elbow_cos > 1.0 + 1e-6:
            notes.append(
                f"analytic seed radial {radial_mm:.2f} mm is outside 2-link reach"
            )
            continue
        elbow_cos = max(-1.0, min(1.0, elbow_cos))
        for elbow_sign, label in [(-1.0, "elbow_down"), (1.0, "elbow_up")]:
            theta3 = elbow_sign * degrees(acos(elbow_cos))
            theta3_rad = radians(theta3)
            theta2 = degrees(
                atan2(wrist_z, wrist_r)
                - atan2(a3 * sin(theta3_rad), a2 + a3 * cos(theta3_rad))
            )
            theta4 = float(target["phi_deg"]) - theta2 - theta3
            angles = [
                _normalize_deg(_joint_angle_from_row_theta(rows[0], base_theta)),
                _normalize_deg(_joint_angle_from_row_theta(rows[1], theta2)),
                _normalize_deg(_joint_angle_from_row_theta(rows[2], theta3)),
                _normalize_deg(_joint_angle_from_row_theta(rows[3], theta4)),
            ]
            limit_reasons = _joint_limit_reasons(joints, angles)
            if limit_reasons:
                notes.append(f"analytic {label} seed outside joint limits: {limit_reasons[0]}")
            seeds.append((label, angles))

    unique: list[tuple[str, list[float]]] = []
    seen: set[tuple[str, tuple[int, ...]]] = set()
    for label, angles in seeds:
        key = (label, tuple(round(angle * 1000) for angle in angles))
        if key not in seen:
            unique.append((label, angles))
            seen.add(key)
    if unique:
        notes.insert(0, "analytic_seed")
    return unique, notes


def _seed_candidates(
    target: dict[str, float],
    links: LinkConfig,
    joints: list[JointConfig],
    current: list[float],
) -> tuple[list[tuple[str, list[float]]], list[str]]:
    analytic, notes = _analytic_seed_candidates(target, links, joints)
    base_guess = degrees(atan2(-target["x_mm"], target["y_mm"])) if hypot(target["x_mm"], target["y_mm"]) > 1e-6 else current[0]
    home = [joint.home_deg for joint in joints]
    fallback = [
        ("current_seed", current),
        ("elbow_up", [base_guess, min(60.0, joints[1].max_deg), 35.0, -20.0]),
        ("elbow_down", [base_guess, 30.0, -55.0, 55.0]),
        ("home_seed", home),
    ]
    candidates: list[tuple[str, list[float]]] = [("current_seed", current)]
    candidates.extend(analytic)
    candidates.append(("home_seed", home))
    if not analytic:
        candidates = fallback
    return candidates, notes


def _fixed_phi_pose(target: dict[str, Any]) -> dict[str, float]:
    return {
        "x_mm": float(target.get("x_mm", 0.0)),
        "y_mm": float(target.get("y_mm", 0.0)),
        "z_mm": float(target.get("z_mm", 0.0)),
        "phi_deg": float(target.get("phi_deg", target.get("tool_phi_deg", 0.0))),
    }


def _target_requests_auto_phi(target: dict[str, Any]) -> bool:
    if bool(target.get("phi_auto", False)):
        return True
    return target.get("phi_deg") is None and target.get("tool_phi_deg") is None


def _auto_phi_values(current_phi_deg: float) -> list[float]:
    values: list[float] = []
    seen: set[int] = set()

    def add(value: float) -> None:
        normalized = _normalize_deg(value)
        key = round(normalized * 1000)
        if key not in seen:
            values.append(normalized)
            seen.add(key)

    for delta in [0, -10, 10, -20, 20, -30, 30, -45, 45, -60, 60, -90, 90, -120, 120, -150, 150, 180]:
        add(current_phi_deg + delta)
    for value in range(-180, 181, 15):
        add(float(value))
    return values


def _candidate_continuity_error(candidate: dict[str, Any], current: list[float]) -> float:
    return sum(
        angle_distance_deg(angle, current[index])
        for index, angle in enumerate(candidate["angles_deg"])
    )


def _select_candidate(
    valid_candidates: list[dict[str, Any]],
    current: list[float],
    requested_branch: str,
) -> tuple[dict[str, Any] | None, list[str]]:
    notes: list[str] = []
    candidates = valid_candidates
    if requested_branch != "auto":
        candidates = [candidate for candidate in candidates if candidate["branch"] == requested_branch]
        if not candidates:
            notes.append(f"requested branch {requested_branch} is not valid")
    if not candidates:
        return None, notes
    return min(
        candidates,
        key=lambda candidate: (
            _candidate_continuity_error(candidate, current),
            candidate["position_error_mm"],
            candidate.get("phi_error_deg", 0.0),
            0 if candidate["branch"] in {"current_seed", "elbow_down"} else 1,
        ),
    ), notes


def _inverse_kinematics_fixed_phi(
    pose: dict[str, float],
    links: LinkConfig,
    joints: list[JointConfig],
    current_joints_deg: list[float] | None = None,
    branch: str = "auto",
) -> dict[str, Any]:
    if not all(isfinite(value) for value in pose.values()):
        return {
            "ok": False,
            "target": pose,
            "candidates": [],
            "selected": None,
            "selected_branch": None,
            "notes": ["target contains a non-finite value"],
        }

    current = current_joints_deg or [joint.home_deg for joint in joints]
    seeds, seed_notes = _seed_candidates(pose, links, joints, [float(value) for value in current])
    requested_branch = branch if branch in {"elbow_up", "elbow_down", "current_seed", "home_seed"} else "auto"
    candidates = [
        _solve_from_seed(pose, links, joints, seed, label)
        for label, seed in seeds
    ]
    valid_candidates = [candidate for candidate in candidates if candidate["valid"]]
    notes: list[str] = seed_notes.copy()
    selected, selection_notes = _select_candidate(valid_candidates, [float(value) for value in current], requested_branch)
    notes.extend(selection_notes)

    if selected is None:
        notes.insert(0, "target is unreachable or did not converge in DH/Jacobian solver")

    return {
        "ok": selected is not None,
        "target": pose,
        "candidates": candidates,
        "selected": selected,
        "selected_branch": selected["branch"] if selected else None,
        "notes": notes,
    }


def _auto_phi_candidate_score(candidate: dict[str, Any], current: list[float], current_phi_deg: float) -> tuple[float, ...]:
    phi_delta = angle_distance_deg(candidate["fk"]["tool_phi_deg"], current_phi_deg)
    return (
        candidate["position_error_mm"],
        _candidate_continuity_error(candidate, current),
        phi_delta,
        0 if candidate["branch"] == "elbow_down" else 1,
    )


def _inverse_kinematics_auto_phi(
    target: dict[str, Any],
    links: LinkConfig,
    joints: list[JointConfig],
    current_joints_deg: list[float] | None = None,
    branch: str = "auto",
) -> dict[str, Any]:
    base_pose = {
        "x_mm": float(target.get("x_mm", 0.0)),
        "y_mm": float(target.get("y_mm", 0.0)),
        "z_mm": float(target.get("z_mm", 0.0)),
        "phi_auto": True,
    }
    if not all(isfinite(value) for value in [base_pose["x_mm"], base_pose["y_mm"], base_pose["z_mm"]]):
        return {
            "ok": False,
            "target": base_pose,
            "candidates": [],
            "selected": None,
            "selected_branch": None,
            "notes": ["target contains a non-finite value"],
        }

    current = [float(value) for value in (current_joints_deg or [joint.home_deg for joint in joints])]
    current_phi = float(forward_kinematics(current, links)["tool_phi_deg"])
    requested_branch = branch if branch in {"elbow_up", "elbow_down", "current_seed", "home_seed"} else "auto"
    phi_values = _auto_phi_values(current_phi)
    valid_candidates: list[dict[str, Any]] = []
    invalid_samples: list[dict[str, Any]] = []
    notes: list[str] = ["auto_phi", f"searched {len(phi_values)} phi values"]
    seed_notes: list[str] = []

    for phi in phi_values:
        pose = {
            "x_mm": base_pose["x_mm"],
            "y_mm": base_pose["y_mm"],
            "z_mm": base_pose["z_mm"],
            "phi_deg": phi,
        }
        analytic_seeds, analytic_notes = _analytic_seed_candidates(pose, links, joints)
        for note in analytic_notes:
            if len(seed_notes) < 5 and note not in seed_notes:
                seed_notes.append(note)
        seeds = analytic_seeds
        if not seeds and abs(angle_distance_deg(phi, current_phi)) <= 1e-9:
            seeds = [("current_seed", current)]
        for label, seed in seeds:
            if label in {"elbow_up", "elbow_down"}:
                candidate = _candidate_from_angles(pose, links, joints, seed, label)
            else:
                candidate = _solve_from_seed(
                    pose,
                    links,
                    joints,
                    seed,
                    label,
                    max_iterations=40,
                )
            candidate["target_phi_deg"] = phi
            candidate["auto_phi"] = True
            if candidate["valid"]:
                valid_candidates.append(candidate)
            elif len(invalid_samples) < 10:
                invalid_samples.append(candidate)

    selection_pool = valid_candidates
    if requested_branch != "auto":
        selection_pool = [candidate for candidate in valid_candidates if candidate["branch"] == requested_branch]
        if not selection_pool:
            notes.append(f"requested branch {requested_branch} is not valid")

    selected = min(
        selection_pool,
        key=lambda candidate: _auto_phi_candidate_score(candidate, current, current_phi),
        default=None,
    )
    if selected is None:
        notes.insert(0, "target is unreachable or did not converge in auto-phi search")
        notes.extend(seed_notes)

    visible_candidates = sorted(
        valid_candidates,
        key=lambda candidate: _auto_phi_candidate_score(candidate, current, current_phi),
    )[:12]
    if selected and all(candidate is not selected for candidate in visible_candidates):
        visible_candidates.insert(0, selected)
    if not visible_candidates:
        visible_candidates = invalid_samples

    resolved_target = base_pose.copy()
    if selected:
        resolved_target["phi_deg"] = float(selected["fk"]["tool_phi_deg"])
        resolved_target["selected_phi_deg"] = float(selected["fk"]["tool_phi_deg"])
    else:
        resolved_target["phi_deg"] = current_phi

    return {
        "ok": selected is not None,
        "target": resolved_target,
        "candidates": visible_candidates,
        "selected": selected,
        "selected_branch": selected["branch"] if selected else None,
        "notes": notes,
    }


def inverse_kinematics(
    target: dict[str, Any],
    links: LinkConfig,
    joints: list[JointConfig],
    current_joints_deg: list[float] | None = None,
    branch: str = "auto",
    tolerance: float = 1e-6,
) -> dict[str, Any]:
    del tolerance
    if len(joints) != 4:
        raise ValueError("inverse_kinematics expects four joint configs")
    if _target_requests_auto_phi(target):
        return _inverse_kinematics_auto_phi(target, links, joints, current_joints_deg, branch)
    return _inverse_kinematics_fixed_phi(_fixed_phi_pose(target), links, joints, current_joints_deg, branch)
