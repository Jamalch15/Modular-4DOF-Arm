from __future__ import annotations

from .motion import RateLimitedMotion, has_reached_target
from .robot_state import MotionState, RobotState


def apply_simulation_step(state: RobotState, limiter: RateLimitedMotion, dt_s: float) -> None:
    if state.motion_state in {MotionState.ESTOP, MotionState.FAULT, MotionState.STOPPED}:
        limiter.set_target(state.reported_angles_deg)
        return

    reported = limiter.step(dt_s)
    state.update_reported_pose(reported, source="simulation", known_pose=True)
    if has_reached_target(reported, state.target_angles_deg):
        state.motion_state = MotionState.IDLE
    else:
        state.motion_state = MotionState.MOVING
