# Accelerated Exam Demo Plan

## Status

This document is a fast, presentation-focused development plan for the robot arm project.

It intentionally focuses on the parts that are visible from the outside during an exam or demo. The goal is not to build every possible subsystem. The goal is to build the most convincing prototype: a robot that looks coordinated, vision-guided, calibrated, safe, and understandable.

The older broader component plan is kept in `component_master_plan.md`.

## Exam Goal

The ideal exam demo should show this complete flow:

```text
1. Start the operator interface.
2. Connect to the robot controller.
3. Run homing or confirm known start position.
4. Show camera feed with detected object.
5. Convert detected object into robot coordinates.
6. Show planned pickup and drop-off sequence.
7. Move the robot smoothly with synchronized joints.
8. Pick the object.
9. Place or sort the object.
10. Show status, logs, and emergency stop behavior.
```

This gives the project a clear story:

```text
vision -> coordinates -> decision -> IK -> smooth motion -> physical action
```

## What To Prioritize

### 1. Smooth Coordinated Movement

Visible value:

- The robot looks controlled instead of improvised.
- Joints start and stop smoothly.
- Joints arrive at the target together.
- The audience can see that the motion is planned, not just manually driven.

What to build:

- Joint-space movement.
- Synchronized arrival time.
- Acceleration and deceleration ramping.
- A safe travel pose above the workspace.
- A few named positions: `home`, `safe`, `pickup`, `dropoff`.

What to avoid for now:

- Advanced obstacle avoidance.
- Full Cartesian path planning.
- Complex PID unless the hardware requires it.

Exam explanation:

> The IK decides where the joints should end. The motion controller decides how they move there smoothly.

### 2. Vision-Guided Object Position

Visible value:

- The camera feed makes the robot feel intelligent.
- Detected object positions can be shown on screen.
- The robot reacts to something in the workspace instead of only playing a fixed script.

What to build:

- Camera view in the operator interface.
- Simple object detection first, such as color/blob detection.
- Draw a marker around the detected object.
- Show object pixel coordinates.
- Convert object position to robot coordinates.

What to avoid for now:

- Training a custom YOLO model unless simple detection is not enough.
- Multi-object recognition unless sorting needs it.
- Complex 3D pose estimation.

Exam explanation:

> The vision system does not control the motors directly. It only returns object coordinates. Task logic and motion control handle the rest.

### 3. Calibration Sequence

Visible value:

- Calibration makes the project look engineered rather than guessed.
- It explains how camera coordinates become robot coordinates.
- It gives the examiners a concrete workflow to understand.

What to build:

- A calibration page or calibration script.
- A simple camera-to-robot mapping.
- A visible calibration target or reference point.
- Saved values for:
  - camera origin
  - scale from pixels to millimeters
  - pickup height
  - drop-off height
  - safe travel height

What to avoid for now:

- Fully automatic calibration.
- Overly mathematical calibration that cannot be demonstrated clearly.

Exam explanation:

> Calibration connects the software coordinate system to the physical robot and workspace.

### 4. Pick-And-Place Task

Visible value:

- This is the clearest complete robot behavior.
- It uses almost every important subsystem.
- It can be repeated during the exam.

What to build:

- Detect or manually select one object.
- Move above object.
- Move down to pickup height.
- Close gripper or activate tool.
- Lift object.
- Move above drop-off point.
- Move down.
- Release object.
- Return to safe pose.

Preferred first sequence:

```text
safe
open gripper
move above object
move to pickup
close gripper
lift
move above dropoff
move to dropoff
open gripper
lift
safe
```

Exam explanation:

> The task layer converts a goal, such as "move this object", into a reusable sequence of robot actions.

### 5. Sorting Mode

Visible value:

- Sorting is more impressive than simple pick-and-place.
- It shows task-specific logic.
- It makes vision feel useful.

What to build after basic pick-and-place works:

- Detect object color or category.
- Map each category to a drop-off zone.
- Use the same pick-and-place sequence with different destination coordinates.

Example:

```text
red object -> bin A
blue object -> bin B
green object -> bin C
```

What to avoid for now:

- Too many categories.
- Hard object classes that require unreliable detection.
- Complex task planning.

Exam explanation:

> Sorting uses the same shared robot infrastructure, but changes the task decision layer.

### 6. Operator Interface

Visible value:

- The interface makes the system understandable.
- It shows status, detected objects, and commands.
- It gives the examiners something concrete to inspect.

Useful visible sections:

- Connection status.
- Camera feed.
- Detected object coordinates.
- Robot status.
- Current task mode.
- Manual joint controls.
- Named position buttons.
- Calibration controls.
- Serial/log console.
- Emergency stop button.

What to avoid for now:

- A landing page.
- Decorative pages that do not control the robot.
- A large GUI before the robot behavior works.

Exam explanation:

> The GUI is an operator tool. It exposes the system state and triggers tested robot functions.

### 7. Live Status And Logging

Visible value:

- Logs make debugging and demonstration easier.
- Status messages show that the system is structured.
- It helps explain failures during the exam.

What to show:

- Current state: `idle`, `homing`, `moving`, `fault`.
- Current target.
- Last command sent.
- Last controller response.
- IK result.
- Vision result.
- Calibration values loaded.

Example log:

```text
[vision] object red at pixel=(341, 216)
[calibration] robot=(28.5, 162.0, 20.0) mm
[ik] target reachable, joints=(12.4, 44.8, -83.2, 28.4)
[motion] synchronized move duration=2.1 s
[controller] MOVEJ accepted
```

Exam explanation:

> Logs make each stage of the robot pipeline visible and testable.

### 8. Safety Demo

Visible value:

- Safety is important in mechatronics.
- A visible emergency stop is easy to understand.
- Refusing unsafe moves shows engineering discipline.

What to build:

- Big `ESTOP` button.
- `STOP` button.
- Refuse movement before homing.
- Refuse unreachable targets.
- Refuse joint limit violations.
- Show fault message in UI.

Exam explanation:

> The robot is not allowed to move blindly. It needs a known state and checked targets.

## Recommended Project Structure For The Exam

This is a presentation-friendly structure. The exact folder names can change later.

```text
pc_app/
  operator_ui/
    camera_view
    manual_controls
    calibration_panel
    task_launcher
    status_log

  vision/
    object_detector
    detection_overlay

  calibration/
    camera_to_robot_transform
    saved_calibration_values

  robot_model/
    geometry
    joint_limits
    workspace_limits

  kinematics/
    forward_kinematics
    inverse_kinematics

  tasks/
    pick_and_place
    sorting

  communication/
    serial_client
    command_parser
    status_parser

controller_firmware/
  command_receiver
  safety_state
  homing
  joint_motion
  stepper_output
  servo_output
  tool_control
```

For the exam, explain it as two sides:

```text
PC side:
vision, calibration, IK, task logic, GUI, logs

Controller side:
real-time motor control, safety stops, homing, actuator output
```

## Accelerated Build Phases

### Phase 1: Make Motion Look Good

Goal:

```text
The robot moves between named positions smoothly and safely.
```

Build:

- Manual joint control.
- Named positions.
- Joint limits.
- Smooth acceleration.
- Synchronized movement.
- Emergency stop.

Demo:

```text
home -> safe -> pickup preview -> dropoff preview -> safe
```

Minimum success:

- Robot motion is repeatable.
- The arm does not jerk aggressively.
- The robot can be stopped.

### Phase 2: Add Kinematics And Target Preview

Goal:

```text
The operator can enter a target position and see the calculated joint angles.
```

Build:

- Forward kinematics.
- Inverse kinematics.
- Reachability check.
- Workspace check.
- UI fields for `x, y, z, tool angle`.
- Preview of target joint angles before moving.

Demo:

```text
enter object coordinate -> calculate IK -> move to target
```

Minimum success:

- Valid targets produce joint angles.
- Invalid targets are rejected.
- The result is visible in the UI/log.

### Phase 3: Add Vision Position Input

Goal:

```text
The camera finds an object and sends its position into the robot pipeline.
```

Build:

- Camera feed.
- Simple object detection.
- Detection marker overlay.
- Pixel coordinate display.
- Pixel-to-robot coordinate transform.

Demo:

```text
place object -> camera detects object -> UI shows robot coordinate
```

Minimum success:

- The object marker follows the object.
- The detected coordinate is stable enough for pickup testing.

### Phase 4: Add Pick-And-Place

Goal:

```text
The robot completes one full object transfer.
```

Build:

- Pick-and-place sequence.
- Gripper/tool control.
- Safe approach height.
- Pickup height.
- Drop-off position.
- Abort handling.

Demo:

```text
detect object -> pick -> place -> return safe
```

Minimum success:

- The sequence works at least with one reliable object type.
- Each step is visible in the log.

### Phase 5: Add Sorting

Goal:

```text
The robot chooses a destination based on the object.
```

Build:

- Detect color or category.
- Map category to destination.
- Reuse pick-and-place sequence.
- Show selected bin in UI.

Demo:

```text
red object -> bin A
blue object -> bin B
```

Minimum success:

- At least two categories work reliably.
- The UI shows the detected class and chosen destination.

## Recommended Exam Demo Script

Use a short, controlled script instead of improvising everything live.

### Step 1: Show Architecture

Say:

> The system is split into PC-side intelligence and embedded real-time control.

Show:

```text
camera -> vision -> calibration -> IK -> task sequence -> controller -> motors
```

### Step 2: Show Manual Motion

Show:

- Connect to controller.
- Home or confirm known start.
- Move to safe position.
- Move to one named target.
- Trigger stop or show emergency stop button.

Why it matters:

This proves the robot can move safely before autonomy is added.

### Step 3: Show IK

Show:

- Enter target coordinates.
- Display calculated joint angles.
- Reject one unreachable target.
- Move to a reachable target.

Why it matters:

This proves the robot is using geometry, not only fixed positions.

### Step 4: Show Vision

Show:

- Camera feed.
- Object marker.
- Pixel coordinates.
- Robot coordinates.

Why it matters:

This proves the system can sense the workspace.

### Step 5: Show Pick-And-Place

Show:

- Place object in workspace.
- Run task.
- Robot picks and places object.
- Logs update during sequence.

Why it matters:

This is the complete integrated behavior.

### Step 6: Show Sorting If Ready

Show:

- Two object colors or categories.
- Robot chooses different drop-off positions.

Why it matters:

This shows task logic on top of the same shared robot system.

## What To Build Only If Time Allows

These are impressive, but should not block the main demo:

- YOLO model.
- Multiple simultaneous objects.
- Automatic calibration.
- Cartesian straight-line interpolation.
- Obstacle avoidance.
- 3D pose estimation.
- Complex GUI styling.
- Advanced PID tuning.

## What Not To Spend Too Much Time On

These are useful internally but weak exam demos by themselves:

- Perfect folder structure.
- Very abstract plugin systems.
- Large configuration framework.
- Complex logging backend.
- Overengineered communication protocol.
- A beautiful UI with no reliable robot motion.

## Exam-Focused Success Criteria

The project is successful if the examiners can see:

- The robot has a known safe state.
- The robot moves smoothly.
- The camera detects something meaningful.
- Coordinates are transformed into robot targets.
- IK produces joint commands.
- A visible task sequence runs from start to finish.
- The interface shows status and errors.
- The project structure separates vision, task logic, kinematics, communication, and firmware.

## Best Single Demo To Aim For

If time is limited, aim for this:

```text
One colored object is detected by camera.
The UI shows the object position.
The robot moves smoothly above it.
The robot picks it up.
The robot places it in one fixed drop-off zone.
The robot returns to safe position.
The log shows every step.
```

This is enough to demonstrate:

- vision
- calibration
- IK
- motion planning
- embedded control
- tool control
- task logic
- GUI
- safety

## AI Development Strategy

Use AI in small visible increments.

Recommended prompt sequence:

```text
1. Make a simple UI panel for manual robot status and commands.
2. Make a small FK module with tests.
3. Make a small IK module with tests.
4. Make a joint-space trajectory generator with synchronized arrival.
5. Make a simple camera blob detector and overlay.
6. Make a calibration transform from pixel to robot coordinates.
7. Make a pick-and-place task sequence generator.
8. Connect the pieces through a dry-run mode.
9. Only then connect to real hardware.
```

Rule:

```text
No AI request should build more than one subsystem at a time.
```

## Short Version

Build the project around what an examiner can see:

```text
smooth robot motion
camera detection
coordinate conversion
IK target solving
pick-and-place sequence
sorting if time allows
operator UI
logs and safety state
```

Keep the internal architecture simple enough that each visible feature can be explained clearly.
