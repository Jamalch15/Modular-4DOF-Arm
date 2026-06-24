# Component Master Plan

## Status

This document is a planning guide only. It describes possible software components for the 4DOF robot arm prototype. It does not mean the architecture, hardware split, interfaces, or task structure are final.

The current goal is to keep the project segmented so each part can be developed, tested, and replaced without letting the system become too complicated too early.

## Priority 1: Safety And Known State

This should be treated as the most important component because every other part depends on the robot knowing whether it is safe to move.

Responsibilities:

- Emergency stop behavior.
- Normal stop behavior.
- Homing or startup position handling.
- Refuse movement when the robot position is unknown.
- Joint angle limits.
- Workspace limits.
- Speed and acceleration limits.
- Basic fault states.

Example fault states:

```text
OK
NOT_HOMED
TARGET_UNREACHABLE
JOINT_LIMIT_EXCEEDED
COMMUNICATION_LOST
MOTION_ABORTED
HOMING_FAILED
VISION_NO_OBJECT_FOUND
```

Prototype note:

Start simple. It is enough if the robot can say "I am not homed", "I am stopped", or "this target is not allowed".

## Priority 2: Robot Model And Configuration

This component stores the physical assumptions about the robot.

Responsibilities:

- Joint names.
- Link lengths.
- Joint angle limits.
- Motor or servo assumptions.
- Gear ratios.
- Tool length or tool offset.
- Home position.
- Safe travel position.
- Motion limits.

Values that are likely to change should live in configuration files instead of being hardcoded.

Useful config groups:

```text
robot_geometry
joint_limits
motion_limits
workspace
camera_calibration
task_settings
```

Important assumptions to document:

- Length units, probably millimeters.
- Angle units, either degrees or radians.
- Positive joint directions.
- Robot base coordinate frame.
- Tool/end-effector coordinate frame.

## Priority 3: Kinematics

This component converts between robot joint angles and positions in space.

Responsibilities:

- Forward kinematics.
- Inverse kinematics.
- Joint limit checking.
- Reachability checking.

Forward kinematics:

```text
joint angles -> end-effector position
```

Inverse kinematics:

```text
target position -> joint angles
```

Recommended first interface:

```text
input:  x, y, z, tool_angle
output: j1, j2, j3, j4 or TARGET_UNREACHABLE
```

Prototype note:

Keep this independent from motors, vision, GUI, and serial communication. It should be possible to test the IK solver without the robot connected.

## Priority 4: Motion Control

This component decides how the robot moves from the current joint angles to target joint angles.

Responsibilities:

- Synchronized joint movement.
- Speed limits.
- Acceleration limits.
- Smooth start and stop.
- Motion progress tracking.
- Abort handling.

Recommended first version:

Use joint-space movement.

```text
current joint angles -> target joint angles -> synchronized smooth move
```

All joints should be scaled so they arrive at the target at the same time.

Recommended motion profile:

```text
accelerate -> constant speed -> decelerate
```

For short moves:

```text
accelerate -> decelerate
```

Prototype note:

Do not start with full Cartesian path planning. Add that later only if the end-effector path needs to be a straight line or avoid obstacles.

## Priority 5: Embedded Controller Firmware

This component runs on the microcontroller and handles timing-sensitive hardware behavior.

Likely responsibilities:

- Stepper pulse generation.
- Servo output.
- Homing routines.
- Limit switch handling, if available.
- Emergency stop.
- Executing motion commands.
- Reporting current state.
- Tool or gripper actuation.

Suggested command ideas:

```text
HOME
MOVEJ j1 j2 j3 j4 speed accel
STOP
ESTOP
STATUS
GRIPPER open
GRIPPER close
```

Prototype note:

The embedded controller should not need to understand vision or task logic. It should mostly receive movement/tool commands and execute them reliably.

## Priority 6: PC-To-Controller Communication

This component connects the PC software to the embedded controller.

Responsibilities:

- Send commands.
- Receive status.
- Parse errors.
- Detect disconnects or timeouts.
- Log communication.

Recommended first version:

Use a simple text-based serial protocol.

Example:

```text
PC -> controller:
MOVEJ 30.0 45.0 -70.0 10.0 35 80

controller -> PC:
OK
STATUS homed=1 moving=0 fault=OK
```

Prototype note:

Avoid sending raw step pulses from the PC. The PC should send targets; the controller should handle real-time motion.

## Priority 7: Manual Test Tool

This component lets the team test the robot without vision or task automation.

Responsibilities:

- Connect/disconnect controller.
- Home robot.
- Move individual joints.
- Move to Cartesian target through IK.
- Open/close gripper.
- Send raw command.
- Show status and errors.

Prototype note:

This can be a simple command-line tool or a small browser UI. It should exist before vision is connected.

## Priority 8: Calibration

This component helps convert rough hardware into repeatable behavior.

Calibration areas:

- Homing position.
- Joint zero offsets.
- Servo pulse ranges.
- Camera calibration.
- Camera-to-robot coordinate transform.
- Pickup height.
- Place height.
- Safe travel height.
- Gripper open/close settings.

Recommended output:

Calibration should save values into config files, not directly into code.

Prototype note:

Start with manual calibration. Automated calibration can come later.

## Priority 9: Vision

This component detects objects and returns their positions.

Responsibilities:

- Read camera image.
- Detect objects.
- Estimate object center.
- Convert image coordinates to robot coordinates.
- Report confidence or detection failure.

Recommended first version:

Use simple color/blob detection or manually selected points.

Possible later version:

Use YOLO or another model if simple detection is not enough for the task.

Prototype note:

The first useful vision output can be very simple:

```text
object found at x, y, z in robot coordinates
```

## Priority 10: Task Logic

This component decides what the robot should do with detected or manually entered object positions.

Responsibilities:

- Choose task mode.
- Choose target object.
- Decide pickup and drop-off positions.
- Build motion sequence.
- Handle task-level failures.

Example first task:

```text
pick object at detected position
move to fixed drop-off point
release object
return safe
```

Possible later tasks:

- Sorting by color.
- Sorting by shape.
- Object-specific handling.
- Different end effectors.

Prototype note:

Keep task logic separate from vision and low-level robot movement.

## Priority 11: Gripper Or End Effector

This component controls whatever tool is attached to the robot.

Responsibilities:

- Open gripper.
- Close gripper.
- Turn electromagnet on/off, if used.
- Store tool-specific settings.
- Optionally verify pickup success.

Prototype note:

Treat the tool as replaceable. A gripper, magnet, or other tool should not require rewriting IK or motion control.

## Priority 12: GUI Or Operator Interface

This component should make testing and operation easier, but it should not be the first dependency for everything else.

Useful views:

- Manual control.
- Serial/status console.
- Camera feed.
- Detected objects.
- Calibration.
- Task launcher.
- Emergency stop.

Prototype note:

Build the GUI around working modules. Avoid making the GUI responsible for the core robot logic.

## Priority 13: Logging And Debugging

This component makes failures understandable.

Useful logs:

- Commands sent to controller.
- Controller responses.
- IK inputs and outputs.
- Motion targets.
- Vision detections.
- Calibration values.
- Faults and aborted moves.

Prototype note:

Plain text logs are enough at first.

## Suggested Development Order

1. Document robot geometry and coordinate assumptions.
2. Build and test forward kinematics.
3. Build and test inverse kinematics.
4. Test one motor/joint at a time.
5. Add synchronized joint motion.
6. Add homing and basic safety state.
7. Add serial command protocol.
8. Build manual test tool.
9. Add calibration files and workflow.
10. Add simple vision or manual object input.
11. Build pick-and-place task sequence.
12. Add GUI features around the working pieces.
13. Add more advanced task logic or path planning only when needed.

## Suggested AI Workflow

Use AI for one small segment at a time.

Good request:

```text
Create a simple IK solver for this 4DOF geometry.
Keep it independent from hardware.
Include 3 tests.
```

Better sequence:

```text
1. Write FK only.
2. Add FK tests.
3. Write IK only.
4. Add IK tests.
5. Add joint limits.
6. Add motion profile.
```

Avoid requests like:

```text
Build the whole robot software with vision, GUI, firmware, calibration, and task logic.
```

## Main Design Rule

Each layer should work by itself before connecting it to the next layer.

Useful data flow:

```text
camera image
-> vision detection
-> robot-frame coordinates
-> task logic
-> IK
-> motion planner
-> controller command
-> actuator movement
-> status/error feedback
```

This keeps the project understandable and leaves room to change hardware, vision, communication, or GUI choices later.
