# Rough Architecture

## Important Note

This document describes early architecture ideas only.

It is not a final design document. The project is still being shaped, and several important technical decisions have not been made yet.

## High-Level Idea

The current concept is a robot system split into two broad sides:

- PC side
- Embedded controller side

The rough logic behind this split is:

- Run heavier computation on the PC
- Keep time-sensitive low-level control close to the hardware

This is a useful starting point, but it is still only a working model.

## PC Side Candidate Responsibilities

The PC may eventually handle some or all of the following:

- Camera input
- Vision pipelines
- YOLO inference or related object detection
- Coordinate extraction from detections
- Task logic
- Inverse kinematics
- Path planning
- GUI
- Logging and debugging tools
- Configuration and calibration tools

Not all of these responsibilities must stay on the PC. Some may move or be split further as the project becomes more concrete.

## Embedded Side Candidate Responsibilities

The embedded controller may eventually handle some or all of the following:

- Low-level actuator control
- Stepper and servo output generation
- Homing routines
- Safety-related stops or fault handling
- Execution of received movement commands
- Tool actuation for end effectors
- Hardware state reporting

The exact boundary between "high-level" and "low-level" control is still undecided.

## Early Data Flow

One possible flow is:

1. A camera observes the workspace
2. Vision software detects relevant objects or positions
3. Task logic decides what the robot should do
4. Motion logic converts that decision into joint targets or trajectories
5. Commands are sent to the embedded controller
6. The embedded controller drives actuators and reports state back

This flow is only a draft model. Future testing may show that some of these stages need to be merged, reordered, or simplified.

## Current Motion Mode Boundaries

These are current implementation boundaries, not final architecture decisions:

- Preview-only IK target editing changes the ghost arm and planned target. It does not command motion.
- Live joint jogging sends rate-limited joint targets.
- Live Cartesian jogging treats the X/Y/Z/Phi faders as velocity controls, solves one bounded local differential-IK step per update, and streams joint velocities with `JOGV`.
- Planned Cartesian or program execution builds a complete path and uses the timed `TRAJ` protocol.

A rejected Cartesian jog sample is not retained as a hidden endpoint target. The next smaller or reverse sample is solved from the last accepted jog seed, while releasing the control sends `JOG STOP` and clears the stream state.

## Task Structure

The robot is expected to support multiple tasks.

Current thinking suggests that each task may differ in:

- What objects it needs to detect
- What perception model or dataset it uses
- What success criteria it has
- What motion sequence it performs
- What end effector it uses

At the same time, tasks will likely share common foundations such as:

- Robot geometry
- Kinematics
- Communication
- Calibration data
- Basic motion APIs

This suggests a future architecture with shared core modules and task-specific modules on top, but that structure is still a proposal, not a commitment.

## Likely Core Software Areas

The project will probably need some version of these components:

- Perception
- Coordinate transforms
- Task planning or task sequencing
- Kinematics
- Trajectory or path generation
- Robot command interface
- Embedded actuator control
- Calibration
- Operator interface

The names, boundaries, and implementation details of these areas are all still open.

## Design Principles For Now

Until the project is more mature, the safest working principles are:

- Keep modules loosely coupled
- Keep assumptions visible
- Prefer replaceable interfaces over fixed hard-coded pipelines
- Separate task-specific logic from reusable robot infrastructure
- Avoid making architecture look more final than it really is

## Things That Are Not Settled Yet

- Whether each task should be its own program or part of one application
- How PC and embedded software should communicate
- Whether IK and path planning should stay on the PC
- How calibration should be stored and applied
- How coordinate frames should be defined
- What level of autonomy the GUI should expose
- How tool-changing or task-specific end effectors should be modeled

These unknowns are expected. The purpose of this document is to make them explicit.
