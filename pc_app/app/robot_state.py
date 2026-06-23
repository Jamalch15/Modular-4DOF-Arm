from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from time import time
from typing import Any


class MotionState(StrEnum):
    IDLE = "idle"
    MOVING = "moving"
    STOPPED = "stopped"
    ESTOP = "estop"
    FAULT = "fault"


@dataclass
class RobotState:
    joint_names: list[str]
    target_angles_deg: list[float]
    reported_angles_deg: list[float]
    estimated_angles_deg: list[float] = field(default_factory=list)
    measured_angles_deg: list[float | None] = field(default_factory=list)
    measurement_valid_mask: str = ""
    joint_authority: list[str] = field(default_factory=list)
    encoder_evidence: list[dict[str, Any]] = field(default_factory=list)
    encoder_telemetry_revision: int = 0
    pose_revision: int = 0
    reported_at: float = field(default_factory=time)
    pose_known_mask: str = ""
    connected: bool = False
    simulation: bool = True
    serial_port: str | None = None
    motion_state: MotionState = MotionState.IDLE
    last_command: str = ""
    last_error: str = ""
    homed: bool = False
    hardware_armed: bool = False
    live_motion_enabled: bool = False
    hardware_mode: str = "simulated"
    hardware_enabled_axes: str = "0000"
    hardware_axis_states: list[str] = field(default_factory=lambda: ["simulated"] * 4)
    config_sync_status: str = "not_connected"
    config_sync_message: str = ""
    controller_capabilities: dict[str, Any] = field(default_factory=dict)
    known_pose: bool = True
    pose_source: str = "simulation"
    encoder_available: str = "0000"
    encoder_angles_deg: list[float | None] = field(default_factory=lambda: [None] * 4)
    encoder_errors_deg: list[float | None] = field(default_factory=lambda: [None] * 4)
    encoder_fault: bool = False
    encoder_mismatch: dict[str, Any] = field(default_factory=dict)
    closed_loop_mode: str = "off"
    correction_state: dict[str, Any] = field(default_factory=dict)
    active_tool: str = "gripper"
    tool_type: str = "servo_gripper"
    tool_state: str = "unknown"
    tool_value: float | None = None
    last_status_line: str = ""
    last_controller_response: str = ""
    motion_execution_state: str = "idle"
    motion_diagnostics: dict[str, Any] = field(default_factory=dict)
    pending_motion: dict[str, Any] = field(default_factory=dict)
    config_change: dict[str, Any] = field(default_factory=dict)
    task_execution: dict[str, Any] = field(default_factory=dict)
    fk: dict[str, Any] = field(default_factory=dict)
    updated_at: float = field(default_factory=time)

    def __post_init__(self) -> None:
        joint_count = len(self.joint_names)
        if not self.estimated_angles_deg:
            self.estimated_angles_deg = [float(value) for value in self.reported_angles_deg]
        if not self.measured_angles_deg:
            self.measured_angles_deg = [None] * joint_count
        if not self.measurement_valid_mask:
            self.measurement_valid_mask = "0" * joint_count
        if not self.joint_authority:
            authority = "simulation" if self.simulation else ("estimated" if self.known_pose else "unknown")
            self.joint_authority = [authority] * joint_count
        if not self.encoder_evidence:
            self.encoder_evidence = [
                {
                    "joint": index + 1,
                    "name": name,
                    "source": "none",
                    "valid": False,
                    "fresh": False,
                    "health": "unavailable",
                }
                for index, name in enumerate(self.joint_names)
            ]
        if not self.pose_known_mask:
            self.pose_known_mask = ("1" if self.known_pose else "0") * len(self.joint_names)

    def update_reported_pose(
        self,
        angles_deg: list[float],
        *,
        source: str | None = None,
        known_pose: bool | None = None,
        known_mask: str | None = None,
        force_revision: bool = False,
        tolerance_deg: float = 1e-9,
    ) -> bool:
        next_angles = [float(value) for value in angles_deg]
        angles_changed = len(next_angles) != len(self.reported_angles_deg) or any(
            abs(current - previous) > tolerance_deg
            for current, previous in zip(next_angles, self.reported_angles_deg)
        )
        source_changed = source is not None and source != self.pose_source
        known_changed = known_pose is not None and bool(known_pose) != self.known_pose
        next_known_mask = known_mask or (
            ("1" if (self.known_pose if known_pose is None else bool(known_pose)) else "0") * len(self.joint_names)
        )
        normalized_next_mask = "".join(
            "1" if bit == "1" else "0"
            for bit in next_known_mask[: len(self.joint_names)]
        ).ljust(len(self.joint_names), "0")
        mask_changed = normalized_next_mask != self.pose_known_mask

        self.reported_angles_deg = next_angles
        self.estimated_angles_deg = next_angles.copy()
        if source is not None:
            self.pose_source = source
        if known_pose is not None:
            self.known_pose = bool(known_pose)
        if known_mask is not None:
            self.pose_known_mask = normalized_next_mask
            self.known_pose = all(bit == "1" for bit in self.pose_known_mask)
        else:
            self.pose_known_mask = ("1" if self.known_pose else "0") * len(self.joint_names)
        if source == "simulation":
            self.joint_authority = ["simulation"] * len(self.joint_names)
        else:
            self.joint_authority = [
                "estimated" if bit == "1" else "unknown"
                for bit in self.pose_known_mask
            ]

        revised = force_revision or angles_changed or source_changed or known_changed or mask_changed
        if revised:
            self.pose_revision += 1
        now = time()
        self.reported_at = now
        self.updated_at = now
        return revised

    def update_encoder_evidence(self, evidence: list[dict[str, Any]]) -> None:
        joint_count = len(self.joint_names)
        normalized = [dict(item) for item in evidence[:joint_count]]
        while len(normalized) < joint_count:
            normalized.append(
                {
                    "joint": len(normalized) + 1,
                    "name": self.joint_names[len(normalized)],
                    "source": "none",
                    "valid": False,
                    "fresh": False,
                    "health": "unavailable",
                }
            )
        measured = [
            float(item["measured_angle_deg"])
            if item.get("fresh") and item.get("measured_angle_deg") is not None
            else None
            for item in normalized
        ]
        valid_mask = "".join("1" if value is not None else "0" for value in measured)
        if normalized != self.encoder_evidence or measured != self.measured_angles_deg:
            self.encoder_telemetry_revision += 1
        self.encoder_evidence = normalized
        self.measured_angles_deg = measured
        self.measurement_valid_mask = valid_mask
        self.updated_at = time()

    def to_dict(self) -> dict[str, Any]:
        return {
            "joint_names": self.joint_names,
            "target_angles_deg": self.target_angles_deg,
            "commanded_target_deg": self.target_angles_deg,
            "reported_angles_deg": self.reported_angles_deg,
            "planning_angles_deg": self.reported_angles_deg,
            "estimated_angles_deg": self.estimated_angles_deg,
            "measured_angles_deg": self.measured_angles_deg,
            "measurement_valid_mask": self.measurement_valid_mask,
            "joint_authority": self.joint_authority,
            "encoder_evidence": self.encoder_evidence,
            "encoder_telemetry_revision": self.encoder_telemetry_revision,
            "pose_revision": self.pose_revision,
            "reported_at": self.reported_at,
            "pose_known_mask": self.pose_known_mask,
            "connected": self.connected,
            "simulation": self.simulation,
            "serial_port": self.serial_port,
            "motion_state": self.motion_state.value,
            "last_command": self.last_command,
            "last_error": self.last_error,
            "homed": self.homed,
            "hardware_armed": self.hardware_armed,
            "live_motion_enabled": self.live_motion_enabled,
            "hardware_mode": self.hardware_mode,
            "hardware_enabled_axes": self.hardware_enabled_axes,
            "hardware_axis_states": self.hardware_axis_states,
            "config_sync_status": self.config_sync_status,
            "config_sync_message": self.config_sync_message,
            "controller_capabilities": self.controller_capabilities,
            "known_pose": self.known_pose,
            "pose_source": self.pose_source,
            "encoder_available": self.encoder_available,
            "encoder_angles_deg": self.encoder_angles_deg,
            "encoder_errors_deg": self.encoder_errors_deg,
            "encoder_fault": self.encoder_fault,
            "encoder_mismatch": self.encoder_mismatch,
            "closed_loop_mode": self.closed_loop_mode,
            "correction_state": self.correction_state,
            "active_tool": self.active_tool,
            "tool_type": self.tool_type,
            "tool_state": self.tool_state,
            "tool_value": self.tool_value,
            "last_status_line": self.last_status_line,
            "last_controller_response": self.last_controller_response,
            "motion_execution_state": self.motion_execution_state,
            "motion_diagnostics": self.motion_diagnostics,
            "pending_motion": self.pending_motion,
            "config_change": self.config_change,
            "task_execution": self.task_execution,
            "fk": self.fk,
            "updated_at": self.updated_at,
        }

    def set_error(self, message: str, fault: bool = False) -> None:
        self.last_error = message
        if fault:
            self.motion_state = MotionState.FAULT
        self.updated_at = time()

    def clear_error(self) -> None:
        self.last_error = ""
        self.updated_at = time()
