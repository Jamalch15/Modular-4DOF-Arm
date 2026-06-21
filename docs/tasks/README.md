# Task Notes

## Status

This folder is for early task descriptions.

Nothing about task structure is finalized yet. A "task" may later become:

- A separate program
- A module inside one larger application
- A configuration profile
- A combination of code and trained vision assets

## Current Idea

Each robot task may define:

- What the robot is trying to accomplish
- What objects or features the vision system must detect
- What outputs the perception stage must produce
- What motion sequence the robot should execute
- What end effector behavior is required
- What calibration or setup data is task-specific

## Current Color-Sorting Contract

The current implementation uses a preview-first operator flow:

1. Capture and inspect a detection snapshot.
2. Select detections, or leave the selection empty to use all eligible detections.
3. Resolve destinations only for the relevant detections in that snapshot.
4. Preview the complete generated sequence.
5. Execute the exact bound preview.

The preview is bound to the detection snapshot, robot pose, robot
configuration/model, task settings, and destination mappings. Any of those
changing requires a new preview.

Working assumption: tool commands do not confirm physical grip or release.
Abort reporting therefore distinguishes no object indicated, possibly held,
and release unconfirmed. Recovery targets are suggestions that still require
pose verification and a new motion preview before use.

## Possible Shared Pattern

A future task description may follow a structure like:

1. Input source
2. Detection targets
3. Required coordinate outputs
4. Motion objective
5. Tool action
6. Success condition
7. Failure handling

This is only a template idea, not a locked standard.

## Intention

The long-term goal is likely to keep task-specific details separate from reusable robot infrastructure such as:

- Kinematics
- Motion execution
- Communication
- Calibration
- Basic operator controls

Exactly how that separation should look is still undecided.
