from __future__ import annotations

from dataclasses import dataclass

from .config import RobotConfig
from .robot_state import MotionState, RobotState


@dataclass(frozen=True)
class SafetyResult:
    ok: bool
    reason: str = ""


def validate_joint_targets(config: RobotConfig, targets: list[float]) -> SafetyResult:
    if len(targets) != len(config.joints):
        return SafetyResult(False, f"expected {len(config.joints)} joint targets")

    for target, joint in zip(targets, config.joints, strict=True):
        if target < joint.min_deg or target > joint.max_deg:
            return SafetyResult(
                False,
                f"{joint.name} target {target:.2f} deg outside "
                f"{joint.min_deg:.2f}..{joint.max_deg:.2f} deg",
            )

    return SafetyResult(True)


def validate_can_move(state: RobotState) -> SafetyResult:
    if state.motion_state == MotionState.ESTOP:
        return SafetyResult(False, "emergency stop is active")
    if state.motion_state == MotionState.FAULT:
        return SafetyResult(False, state.last_error or "robot fault is active")
    if not state.connected and not state.simulation:
        return SafetyResult(False, "not connected to hardware and simulation is disabled")
    if not state.simulation and not state.known_pose:
        return SafetyResult(False, "robot planning pose is unknown; use Set Pose while disarmed before motion")
    return SafetyResult(True)
