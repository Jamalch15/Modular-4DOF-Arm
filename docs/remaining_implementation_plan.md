# Remaining Implementation Plan By Workpiece

This document replaces the earlier scattered roadmap with a subsystem-based plan. The goal is to make it easy to request implementation in useful chunks, for example:

```text
Implement TOOL-01 and TOOL-02.
```

```text
Implement KIN-01 through KIN-03 only.
```

The project is still in an early stage. Treat the structure below as the current working plan, not a fixed final architecture.

## Current Reality

Some earlier roadmap items have been partially implemented, but several are still placeholder-level.

Currently present at a rough level:

- Main tabs have been renamed to `Control`, `Tasks`, `Kinematics`, `Program`, and `Settings`.
- The old permanent COM text box has been replaced with a serial modal.
- The Control tab has a gripper/magnet tool selector and shows only the matching control group for the selected tool.
- The backend has tool-type-aware `TOOL` commands, a `tools` config section, and active tool validation.
- The backend has a Standard DH data model and Jacobian IK plumbing.
- The app has an editable MATLAB prototype geometry preset with `L_1..L_9`, `s4`, `s6`, and `s8`.
- The Settings view has a derived DH table, but it is currently read-only.
- The 3D view has DH-based rendering and object marker plumbing.
- The backend has basic color/blob vision helpers and task sequence builders.
- Firmware/backend protocol parsing includes some newer status fields.
- Firmware/backend now have a timed `TRAJ` upload/start path for multi-waypoint motion, replacing repeated hardware `MOVEJ` waypoint streaming for linear/program paths.
- The viewport faders have a first-pass Cartesian live jog mode using differential IK and `JOGV` firmware velocity updates.
- Encoder readback fields exist in software, but are not a real hardware workflow yet.
- Analytic IK seed generation exists as a backend first pass and has regression tests.
- Backend unit tests currently pass, and the protocol stub plus full-arm controller firmware targets build.

Recent verification from the 2026-06-17 audit and the 2026-06-18 calibration continuation:

- `python -m pytest -q` in `pc_app`: 92 passed, with two FastAPI `on_event` deprecation warnings.
- Python compilation, frontend ES-module syntax checks, API/static route smoke checks, and the documented AprilTag CLI `--help` invocation passed.
- `pio run -e esp32-s3-arm-protocol-stub -e esp32-s3-arm-controller`: both firmware targets built successfully.
- Headless Chrome rendered UI check through DevTools Protocol: selecting `Gripper` shows only gripper controls and the slider; selecting `Magnet` shows only magnet controls; no browser console errors were reported.
- No real hardware movement or real tool IO was verified during this audit.

Still missing or not production-ready:

- Control preview/live jog behavior needs to be proven stable.
- Tool controls switch by selected tool, and unsupported tool commands are rejected in software; real unsafe/hardware states still need physical validation.
- Tool TCP offsets drive the backend active TCP model as a first pass; visible 3D tool geometry and physical calibration still need work.
- Tool pins can be edited from Settings. Encoder pins cannot be properly edited from Settings yet.
- Camera preview should move out of the side panel into a movable popup.
- DH editing needs an editable table workflow; the current table is derived/read-only.
- FK and analytic seeding have been partially reconciled with the MATLAB prototype, but the Jacobian implementation still needs to be reconciled with the DH-frame cross-product model.
- The MATLAB motion prototype has useful ideas, but it lacks motor velocity/acceleration limits for real execution.
- Calibration is not yet a guided workflow.
- AprilTag-based camera/world calibration is implemented as a first pass with a fixed four-tag workspace layout, multi-frame accumulation, pose-quality checks, persistence, and verification.
- The viewport now shows configured AprilTags and a solved camera frustum in calibration/frame mode. Projecting the live camera image onto the workspace is still missing.
- Planned, estimated, and actual executed TCP paths are not clearly separated in the viewport.
- Cartesian waypoint execution does not yet guarantee blended continuous motion through waypoints.
- End-effector speed/acceleration limits in `mm/s` and `mm/s^2` are not first-class motion settings yet.
- Settings should show encoders only for base and shoulder. Elbow and wrist are servos for now.
- AS5048A encoder configuration is not truly PC-app driven yet.
- Vision and task workflows are placeholders, not operator-ready workflows.
- Diagnostics and tests need to be organized around real failure modes.

## MATLAB Prototype Summary

Prototype reviewed: `jacobian_ik_robotarm_analytic_seed.m`.

The MATLAB file implements a useful working prototype for robot geometry, DH forward kinematics, analytic IK seeding, Jacobian movement, visualization, and motion diagnostics.

### Physical Model In The Prototype

Working assumption: these are current measured/prototype values, not final calibration values.

```text
L_1 = 93.45 mm
L_2 = 23.20 mm
L_3 = 64.50 mm
L_4 = 42.69 mm
L_5 = 160.15 mm
L_6 = 41.39 mm
L_7 = 142.55 mm
L_8 = 49.20 mm
L_9 = 15.00 mm

s4 = -1
s6 = -1
s8 =  1
```

Joint limits in the prototype:

```text
theta1: -180 to 180 deg
theta2:  -90 to 160 deg
theta3: -160 to 160 deg
theta4: -180 to 180 deg
```

Starting pose:

```text
[0, 0, -70, -20] deg
```

Movement tuning in the prototype:

```text
end-effector speed: 25 mm/s
dt: 0.02 s
max move time: 60 s
position tolerance: 1.0 mm
phi tolerance: 2 deg
Jacobian damping lambda: 0.1
phi convergence gain: 1.5
prefer elbow down: true
```

### DH And FK Implemented In The Prototype

The prototype uses Standard DH. The active DH table is:

```text
joint  theta   alpha   a     d          extra measured offset
1      th1     pi/2    0     L1 + L3   L2 side offset after d1
2      th2     0       L5    s4 * L4
3      th3     0       L7    s6 * L6
4      th4     0       L9    s8 * L8
```

It computes:

- Full transform chain.
- Joint points.
- End-effector position.
- Pitch-like tool angle `phi = theta2 + theta3 + theta4`.
- Linear Jacobian from DH frame axes using cross products.
- Full Jacobian including angular rows.
- Segment data for visualizing DH `d` and `a` offsets separately.

### Analytic Seed IK Implemented In The Prototype

The prototype does not rely on Jacobian IK from a random seed. It first computes an analytic seed using the robot geometry.

Main ideas:

- Combine lateral offsets into `B = s4*L4 + s6*L6 + s8*L8`.
- Solve base angle from the XY target while accounting for the lateral offset.
- Solve the shoulder/elbow planar 2-link part using law of cosines.
- Compute wrist angle from `theta4 = phiTarget - theta2 - theta3`.
- Try multiple candidate branches.
- Reject candidates outside joint limits.
- Score candidates using:
  - FK position error,
  - phi error,
  - elbow-down preference,
  - continuity from the current pose.

This is important. It gives the numerical Jacobian solver a realistic starting point and reduces weird solutions near joint limits.

### Jacobian Movement Implemented In The Prototype

The movement function simulates Cartesian end-effector velocity control.

At each time step it:

- Computes current FK and Jacobian.
- Computes position error and phi error.
- Commands a linear end-effector velocity toward the target.
- Commands phi convergence with a bounded phi rate.
- Builds a task Jacobian:

```text
J_task = [Jv; J_phi]
J_phi = [0, 1, 1, 1]
```

- Solves damped least squares:

```text
thetaDot = J_task' * inv(J_task * J_task' + lambda^2 * I) * xDotTask
```

- Adds a weak attraction toward the analytic seed.
- Wraps and clamps joint angles to limits.
- Stops when position and phi tolerances are reached.

Important limitation: the function name includes `NoMotorLimit`. It does not properly enforce real motor velocity, acceleration, step timing, or synchronized arrival constraints. It is good as a preview/simulation model, but it should not be treated as real execution control until those constraints are added.

### Visualization And Diagnostics Implemented In The Prototype

The prototype includes:

- 3D animation of the arm path.
- Separate drawing of DH `d` and `a` segments.
- End-effector path trace.
- Target marker.
- Phi direction arrow.
- Plots for desired vs actual end-effector speed.
- Joint velocity plots.
- Position error and phi error plots.
- Warnings when final joints are near limits.

## What From The MATLAB Prototype Belongs In This Roadmap

These ideas should be added to the implementation plan:

- Use the measured `L_1..L_9` and sign-offset model as a geometry preset or calibration starting point.
- Make measured geometry the editable source of truth, with Standard DH derived from the MATLAB dimensions as the first known-good physical model.
- Add an analytic seed step before numerical Jacobian IK.
- Use DH frame axes to compute the Jacobian, not ad hoc or inconsistent derivatives.
- Preserve the `phi = theta2 + theta3 + theta4` task orientation model for the first pass.
- Add elbow-down and continuity preferences to IK solution scoring.
- Add Cartesian velocity preview using damped least squares.
- Add movement diagnostics: estimated duration, EE speed, joint velocities, position error, phi error, and limit warnings.
- Add DH segment visualization to the 3D view so measured geometry is understandable.
- Add tests that compare Python FK/IK behavior against known MATLAB prototype targets.

These ideas should not be ported directly:

- The MATLAB command-window loop.
- MATLAB figures as the app UI model.
- The no-motor-limit movement loop as real robot execution.
- Any assumption that the prototype dimensions are final calibration.
- Any full closed-loop control behavior.

## Recommended Implementation Order

This order keeps useful operator fixes first, then builds the kinematics and movement foundation before adding heavier vision/task behavior.

1. `SHELL-01` through `SHELL-04`: fix the operator shell, preview stability, camera popup, HUD/fader, and serial modal polish.
2. `TOOL-01` through `TOOL-04`: finish end-effector selection, settings, active TCP, and firmware IO behavior.
3. `KIN-01` through `KIN-06`: reconcile the app with the MATLAB DH/IK model.
4. `MOVE-01` through `MOVE-08`: build movement preview, constraints, continuous Cartesian control, progress, abort, and diagnostics.
5. `CAL-01` through `CAL-04`: add guided arm/tool calibration and validation.
6. `ENC-01` through `ENC-05`: implement AS5048A readback, known pose, verification, and bounded correction.
7. `VISION-01` through `VISION-07`: build the camera popup, detections, profiles, AprilTag calibration, camera pose, and projected camera plane.
8. `TASK-01` through `TASK-04`: rebuild task workflows around preview-first operation.
9. `VIEW-01` through `VIEW-05`: polish 3D visualization, object markers, DH segment overlays, camera pose, and projected camera image layers.
10. `DIAG-01`, `TEST-01`, `TEST-02`, and `TEST-03`: harden diagnostics and regression tests.

## Workpiece: Operator Shell And Control UI

### SHELL-01: Control Preview And Live Jog Stability

Status: implemented as a source-level first pass; needs browser and hardware verification.

Reality note: frontend state now separates draft, commanded, pending, and reported angles well enough on source review. This still needs rendered UI testing during live jog and real hardware feedback.

Work:

- Keep preview visible after pressing Apply.
- Clear preview only when target is reached, Reset is pressed, or a newer target replaces it.
- Separate frontend state into:
  - draft target,
  - commanded target,
  - reported pose,
  - target reached.
- Stop websocket pose updates from recreating or clearing the preview during live jog.
- Stabilize Apply and Reset buttons so state changes do not shift layout.

Acceptance:

- Apply does not immediately remove the preview.
- Live jog preview does not twitch.
- Apply/Reset buttons do not visually twitch.

### SHELL-02: Camera Popup Instead Of Side-Panel Preview

Status: missing.

Reality note: source review still shows the camera frame rendered inline in the Tasks side panel.

Work:

- Remove the large inline camera preview from the Tasks side panel.
- Add a `View Camera` button.
- Open a movable, resizable camera popup over the viewport.
- Keep annotations in the popup.
- Let detections continue when the popup is closed or minimized.

Acceptance:

- Tasks panel stays compact.
- Camera can be opened, moved, resized, and closed.
- Detection state remains usable without the popup open.

### SHELL-03: HUD And Fader Chrome Polish

Status: partial; placement and arrows still need UI polish.

Reality note: HUD and fader widgets have collapse buttons and CSS rotation, but the placement and arrow direction have not been browser-verified and still match the user-reported problem area.

Work:

- Reposition HUD and fader widgets so they do not fight the rail, left panel, or viewport.
- Fix collapse arrow direction.
- Prefer clear icon buttons instead of ambiguous text arrows.
- Verify desktop and smaller viewport layouts.

Acceptance:

- HUD/fader controls do not overlap important controls.
- Closed/open arrows point correctly.
- Collapsed controls remain usable.

### SHELL-04: Serial Modal Polish

Status: implemented as a first pass; stale-port behavior still needs hardware verification.

Reality note: serial connection is modal, lists detected ports with descriptions/HWID, supports baud selection, and saves the last selected port after a successful connection.

Work:

- Keep the serial picker as a modal, not a permanent large widget.
- Show available COM ports with descriptions.
- Keep baud rate secondary but configurable.
- Save last selected port to local config.
- Handle unavailable/stale ports clearly.

Acceptance:

- User can connect without typing COM names manually.
- Stale/disconnected ports fail clearly.

## Workpiece: End Effector And Tooling

### TOOL-01: Selected Tool Controls Only

Status: implemented as a first pass; needs real hardware validation.

Reality note: the Control tab now hides gripper controls for magnet and hides magnet controls for gripper, including the CSS `[hidden]` override fix that caused both groups to appear at once. Frontend command buttons are gated by connection/armed/fault state, and backend tool actions reject commands unsupported by the active tool type. A headless Chrome rendered check verified the gripper and magnet selection behavior; real hardware behavior still needs validation.

Work:

- Show gripper controls only when active tool is `gripper`.
- Show magnet controls only when active tool is `magnet`.
- Gripper controls:
  - open,
  - close,
  - proportional slider `0.0..1.0`.
- Magnet controls:
  - on,
  - off.
- Disable irrelevant commands when disconnected, unsupported, or unsafe.

Acceptance:

- Selecting `Gripper` shows only gripper controls.
- Selecting `Magnet` shows only magnet controls.
- Tool UI state follows backend state.

### TOOL-02: End-Effector Settings And IO Editor

Status: implemented as a first pass; needs hardware values and operator review.

Reality note: Settings now has editable tool preset cards for type, display label, TCP offset, gripper servo IO/range/open-close values, and magnet GPIO/polarity. Saves go through `robot.local.yaml` calibration updates with validation before persistence. The real pin numbers, pulse ranges, and TCP offsets still need to be confirmed on the physical build.

Work:

- Add editable tool presets in Settings:
  - tool type,
  - display name,
  - TCP offset x/y/z,
  - gripper PWM pin,
  - gripper pulse min/max,
  - gripper open/closed values,
  - magnet GPIO pin,
  - magnet active polarity.
- Save values to `robot.local.yaml`.
- Validate pins, ranges, and units before saving.

Acceptance:

- Gripper dimensions and servo IO can be edited from the UI.
- Magnet dimensions and GPIO IO can be edited from the UI.
- Invalid values do not silently save.

### TOOL-03: Active Tool TCP Integration

Status: implemented as a backend first pass; 3D geometry and physical calibration still incomplete.

Reality note: the backend loads the active tool TCP offset and applies it through the active link/tool model. The UI editor can update the offsets, active tool geometry changes mark tool calibration stale, and pick/place task tool steps now follow the active tool type. The 3D view does not yet visibly change the rendered tool geometry, and the physical TCP offsets are still unvalidated.

Work:

- Apply the active tool TCP offset in FK.
- Use active tool TCP in IK.
- Use active tool TCP in pickup/dropoff task targets.
- Update the 3D view so gripper/magnet length changes the visible TCP.
- Mark calibration stale when active tool geometry changes.

Acceptance:

- Switching tool type changes the TCP used by FK/IK.
- Task targets use the selected tool geometry.
- Backend and 3D view report the same TCP position.

### TOOL-04: Real Firmware Tool IO

Status: implemented as a firmware first pass; real IO untested.

Reality note: the PC config sync now emits active `CONFIG TOOL` lines, the protocol stub accepts them, and `arm_controller.cpp` parses the active tool config. The controller drives a configured gripper PWM output for `OPEN`, `CLOSE`, and `SET`, drives a configured magnet GPIO for `ON` and `OFF`, rejects unsupported commands for the active tool type, and sets the tool safe on boot/stop/fault/disable. This has compiled for the ESP32-S3 targets, but it has not been tested against the real servo, transistor, magnet, or power wiring.

Work:

- Implement `TOOL OPEN`, `TOOL CLOSE`, and `TOOL SET value=...` for the gripper.
- Implement `TOOL ON` and `TOOL OFF` for the magnet.
- Drive a 180 degree microservo from configured PWM and pulse range.
- Drive electromagnet output from configured GPIO and polarity.
- Set safe tool output on boot, stop, fault, and disconnect.

Acceptance:

- Gripper commands move the configured servo output.
- Magnet commands switch the configured output.
- Tool outputs fail safe on stop/fault/reset.

## Workpiece: Robot Geometry, DH, FK, And IK

### KIN-01: Import MATLAB Physical Geometry As A Preset

Status: implemented as a first pass.

Reality note: the MATLAB values are stored as a `working_assumption` preset, not as final calibration. Applying the preset fills a DH draft; it does not silently prove the physical arm is calibrated.

Work:

- Add a named geometry preset based on the MATLAB prototype.
- Store values as measured link dimensions, not hardcoded solver constants.
- Include `L_1..L_9` and sign values `s4`, `s6`, `s8`.
- Keep `L_2` in the active measured model as a base side offset, not as
  first-row `a1`.
- Label all units as `mm` and `deg`.

Acceptance:

- The app can load the MATLAB prototype geometry as a starting model.
- The user can see and edit measured dimensions before applying them.

### KIN-02: Professional DH Table Editor

Status: partial.

Reality note: the Settings UI now renders the derived DH rows as a table and validates the derived draft, but the table is read-only. Direct DH-row editing in a proper table is still missing.

Work:

- Replace generic DH inputs with a table editor.
- Columns:
  - joint,
  - theta offset deg,
  - d mm,
  - a mm,
  - alpha deg,
  - min deg,
  - max deg,
  - zero offset deg,
  - direction sign.
- Add row-level validation.
- Show FK preview before saving.
- Save to `robot.local.yaml`.

Acceptance:

- Measured geometry is edited in one place.
- Derived DH values are shown read-only and validated visibly.
- FK preview updates predictably from the derived model.

### KIN-03: DH Forward Kinematics Aligned With MATLAB

Status: implemented as a first pass.

Reality note: Python FK is tested against selected measured-prototype DH poses. The coordinate-frame convention is still a project working assumption until measured against the physical arm.

Work:

- Derive Standard DH from the active measured-geometry preset.
- Verify the app uses the same transform order as the MATLAB prototype.
- Support the measured-prototype table:

```text
d1 = L1 + L3
base side offset = L2
d2 = s4 * L4
d3 = s6 * L6
d4 = s8 * L8
a2 = L5
a3 = L7
a4 = L9
alpha1 = 90 deg
alpha2..4 = 0 deg
```

- Apply active tool TCP after the final joint transform.
- Add FK tests against known measured-prototype poses.

Acceptance:

- Python FK matches the MATLAB model for selected test poses.
- The 3D view and backend agree on joint frames and TCP position.

### KIN-04: Analytic Seed Before Jacobian IK

Status: implemented as a backend first pass.

Reality note: analytic seed candidate generation exists, runs before the numerical solver for the measured-prototype DH shape, and is covered by regression tests. It still needs hardware validation and clearer UI diagnostics for rejected seeds and selected branches.

Work:

- Port the MATLAB analytic seed concept into backend kinematics.
- Use lateral offset `B = d2 + d3 + d4`.
- Solve base angle while accounting for offset.
- Solve shoulder/elbow with 2-link planar IK.
- Compute wrist angle from target phi.
- Generate candidate branches.
- Reject candidates outside joint limits.
- Score with:
  - FK position error,
  - phi error,
  - elbow-down preference,
  - continuity from current pose.

Acceptance:

- Numerical IK starts from a realistic seed.
- Common targets converge more reliably.
- Solver chooses less surprising joint configurations.

### KIN-05: DH-Based Jacobian IK Diagnostics

Status: partial.

Reality note: IK reports candidates, selected branch, convergence errors, iterations, notes, and singularity warnings. The current solver still uses a finite-difference numeric Jacobian, not the DH-frame axis cross-product Jacobian from the MATLAB prototype.

Work:

- Compute linear Jacobian from DH frame axes using cross products.
- Use damped least squares.
- Keep the first-pass orientation task as `phi = theta2 + theta3 + theta4`.
- Report:
  - success/failure,
  - final position error,
  - final phi/orientation error,
  - iteration count,
  - limit warnings,
  - singularity or near-singularity warnings,
  - selected seed source.

Acceptance:

- Reachable targets converge in tests.
- Unreachable targets fail clearly.
- The UI explains why IK failed.

### KIN-06: Workspace And Reachability Checks

Status: missing.

Work:

- Add a fast reachability precheck before expensive IK.
- Check joint limits, workspace bounds, and target approach height.
- Show whether target failure is due to reach, joint limits, or invalid orientation.

Acceptance:

- Invalid targets fail before execution.
- Operator sees a useful reason, not just `IK failed`.

## Workpiece: Movement, Trajectory, And Execution

### MOVE-01: Motion State And Preview Reliability

Status: implemented as a first pass after audit.

Reality note: draft, commanded, reported, and preview state are separated well
enough for the current UI. Preview, target, object, planned path, and actual
path now use separate view layers. This should still be verified during real
hardware jogging.

Work:

- Keep draft, commanded, reported, and reached states separate.
- Ensure preview/path/object markers are independent layers.
- Clear preview only from explicit state transitions.
- Keep Home and Stop available in the main view.

Acceptance:

- Preview behavior is stable across Apply, Reset, live jog, websocket updates, and task execution.

### MOVE-02: MATLAB-Style Cartesian Velocity Preview

Status: partially covered by current trajectory preview.

Reality note: the app now has joint, linear Cartesian, and program trajectory
preview paths with duration estimates. This is enough for MOVE-04/MOVE-06.
The exact MATLAB-style damped-least-squares velocity loop remains a future
preview refinement and is still not a hardware execution controller.

Work:

- Add a preview-only movement simulation based on the MATLAB loop:
  - fixed end-effector speed,
  - fixed `dt`,
  - damped least squares,
  - phi convergence,
  - analytic seed attraction,
  - joint wrapping and clamping.
- Show estimated path, duration, speed, and final error.
- Keep this marked as preview/simulation until motor limits are added.

Acceptance:

- User can preview a smooth Cartesian move before execution.
- Preview reports estimated duration and final error.
- Preview does not pretend to be a guaranteed hardware execution path.

### MOVE-03: Real Joint Limits, Velocity Limits, And Acceleration Limits

Status: implemented as a first pass for joint-space previews and execution.

Reality note: joint targets are checked against limits, per-joint speed and
acceleration limits feed the generated waypoint profile, and direct joint
execution uses configured speed/acceleration. Cartesian previews still depend
on generated IK waypoints and should not be treated as firmware-level blended
Cartesian control.

Work:

- Add per-joint velocity and acceleration limits.
- Add stepper/servo command-rate constraints.
- Add synchronized joint arrival for joint-space moves.
- Add bounded Cartesian preview that respects motor limits.
- Make units explicit.

Acceptance:

- Generated moves respect configured joint limits and speed limits.
- Estimated duration is credible for real hardware.

### MOVE-04: Execution Progress And Abort Behavior

Status: implemented as a first pass.

Reality note: motion diagnostics now track a run id, execution state, current
waypoint, total waypoints, progress ratio, active task step label, expected
duration, actual duration, final joint error, controller response, and stop or
failure result. Stop cancels path, live, and task motions through the same
diagnostic finish path. Stale cancelled motion runs cannot overwrite a newer
active run.

Work:

- Show active move, waypoint, or task step.
- Show progress through generated path.
- Make Stop behavior consistent across:
  - direct joint movement,
  - live jog,
  - path execution,
  - tasks,
  - serial motion.
- Keep internal emergency/fault handling, but do not expose a large E-stop UI unless needed.

Acceptance:

- User can tell what the robot is currently doing.
- Stop reliably aborts motion/task/live jog.

### MOVE-05: Motion Diagnostics

Status: partial.

Reality note: backend motion diagnostics track run id, execution state, waypoint progress, expected/actual duration, final error, controller response, and actual TCP samples. The UI diagnostics are still not organized into the full motion/IK plots and failure-mode views described below.

Work:

- Add diagnostics inspired by MATLAB plots:
  - desired vs actual EE speed,
  - joint velocity history,
  - position error,
  - phi error,
  - near-limit warnings,
  - estimated vs actual move duration.
- Keep this in Diagnostics, not normal operator UI.

Acceptance:

- Movement behavior can be debugged after a bad move without cluttering normal operation.

### MOVE-06: Planned, Estimated, And Actual Path Layers

Status: implemented as a first pass.

Reality note: the viewport now separates the planned/estimated preview TCP path
from the actual reported TCP trail. Actual TCP points are sampled from
`reported_angles_deg` plus FK during simulation and serial status updates. The
path summary labels joint moves as joint-space TCP estimates instead of
guaranteed Cartesian paths.

Problem: the current blue path is the preview waypoint TCP trace. In joint mode, execution may send only the final joint endpoint, so the blue path is not necessarily the path the physical arm will take. This is confusing and should be fixed before trusting Cartesian workflows.

Work:

- Split path visualization into separate layers:
  - planned TCP path,
  - estimated execution TCP path,
  - actual reported TCP trail.
- Record actual reported TCP points during every motion from `reported_angles_deg` plus FK.
- Display actual trail with a different color/style from the planned path.
- Label the preview path type:
  - joint-space TCP trace,
  - linear Cartesian waypoint path,
  - live Cartesian jog path,
  - recorded actual path.
- Keep old preview behavior available but make it explicit when it is only an estimate.
- Add diagnostics comparing:
  - planned endpoint,
  - reported endpoint,
  - max path deviation,
  - final TCP error,
  - expected vs actual duration.

Acceptance:

- The user can tell whether a line is planned, estimated, or actually executed.
- Joint-mode preview no longer implies that hardware will trace a guaranteed Cartesian path.
- Actual path recording works in simulation and hardware mode.

### MOVE-07: End-Effector Speed And Acceleration Limits

Status: missing.

Working assumption: this starts with Cartesian preview and software-generated waypoint streaming. Firmware remains a target follower until a proper trajectory queue/blending mode exists.

Work:

- Add motion settings for Cartesian/TCP moves:
  - max TCP speed `mm/s`,
  - max TCP acceleration `mm/s^2`,
  - control/update period,
  - Cartesian tolerance,
  - orientation/phi behavior.
- Generate time-parameterized Cartesian profiles:
  - trapezoid or S-curve scalar progress along a Cartesian segment,
  - bounded TCP velocity,
  - bounded TCP acceleration.
- Convert each time step to joint targets through IK/Jacobian.
- Check and report joint velocity/acceleration limit violations.
- Slow the Cartesian profile down when joint limits would be exceeded.
- Show TCP speed/acceleration estimates in diagnostics.
- Keep joint-space speed/accel controls separate from TCP speed/accel controls.

Acceptance:

- A linear Cartesian move can be requested as "max 100 mm/s, max N mm/s^2".
- The preview reports whether the requested TCP speed is feasible under joint limits.
- Generated paths do not silently exceed configured joint limits.

### MOVE-08: Continuous Cartesian Live Jog And Plane Drawing

Status: implemented as a first pass for live Cartesian jog; plane drawing remains missing.

Goal: dragging X/Y/Z/Phi controls should make the TCP move smoothly along Cartesian directions without running a full endpoint IK preview/planning cycle for every tiny UI change. Later this can support constrained plane drawing.

Working assumption: this should be implemented as resolved-rate Cartesian jogging / differential IK. The current full IK preview path is still useful for "go to this point", but it is too heavy and too stop-start for live slider control.

Reality note: the viewport faders now have an explicit `Cart Jog` mode with TCP and Phi speed limits. With that mode off they are preview-only; `Live Real` alone does not turn an IK target preview into Cartesian jogging. The backend exposes `/api/cartesian-jog` and `/api/cartesian-jog/stop`, computes one bounded local resolved-rate step per velocity sample with damped least-squares differential IK, seeds simulation jogs from the reported/current pose, vector-scales joint steps/velocity limits so the joint direction is not distorted, and uses `JOGV`/`JOG STOP` for hardware. Rejected samples are not accumulated as a hidden Cartesian endpoint, so a smaller or reverse command is evaluated immediately from the last accepted seed. Frontend jog and stop requests are serialized so an old response cannot overwrite a newer drag. Translation-only jogs do not silently constrain Phi; Phi is included when the Phi jog command is non-zero. Firmware and the no-motor stub accept `JOGV`; the full controller integrates streamed joint velocities with acceleration limiting and has a watchdog that ramps velocity toward zero if updates stop. Locally constrained or near-singular directions report whether the local step is unreachable, would cause excessive lateral drift, or hit a joint limit. A repeatable simulation/debug harness exists at `pc_app/tools/debug_cartesian_jog.py` to sweep known poses and classify paths before testing on hardware. This has unit-test coverage, but live hardware smoothness still needs physical validation.

Work:

- Add a live Cartesian jog mode separate from joint live jog. Status: first pass implemented.
- Add operator controls for:
  - X jog,
  - Y jog,
  - Z jog,
  - Phi jog or fixed/auto phi behavior,
  - TCP jog speed,
  - live-enable/deadman state for hardware.
  Status: first pass implemented through viewport faders, `Cart Jog`, TCP/Phi speed fields, and existing Live Real/Arm gating.
- Treat X/Y/Z/Phi sliders or faders as Cartesian velocity or small-delta commands, not as one-off IK endpoint commands.
- Prefer "hold/drag to jog" controls over absolute sliders for the first hardware version so accidental large target jumps are harder to create.
- Maintain a live Cartesian goal state:
  - current TCP,
  - desired TCP,
  - selected constraint plane or free XYZ,
  - phi fixed or auto,
  - TCP speed/accel limits.
- Run a continuous PC-side control loop:
  - throttle browser input to a fixed command rate, for example 20-30 Hz,
  - sample UI target or commanded Cartesian velocity at a fixed rate,
  - generate a bounded Cartesian step,
  - compute joint deltas with a Jacobian pseudo-inverse or damped least-squares differential IK,
  - seed each update from the previous joint solution/current reported pose, not a fresh global solve,
  - clamp joint velocity, joint acceleration, joint limits, TCP speed, and TCP acceleration,
  - stream short joint targets or mini timed `TRAJ` updates at a safe fixed rate,
  - stop cleanly when input stops or target is reached.
  Status: implemented with `/api/cartesian-jog`, differential IK, command throttling, speed/accel clamping, and `JOGV`.
- Add singularity and limit handling:
  - damped least-squares fallback near singular Jacobians,
  - reject or slow commands that require excessive joint velocity,
  - show blocked directions when workspace, joint limit, or singularity limits prevent motion,
  - avoid branch flips by keeping continuity from the previous solution.
  Status: implemented for live jog with direction-authority checks and regression tests. The current policy intentionally blocks some near-singular local directions rather than allowing a visible sideways TCP drift.
- Add backend API shape for Cartesian jog:
  - start/stop live Cartesian jog,
  - update commanded Cartesian velocity/delta,
  - return current TCP, accepted command, clamped command, and block reason,
  - enforce stale-input timeout.
- Add firmware/protocol support if needed after the PC-side prototype:
  - a live jog/servo mode with watchdog timeout,
  - queued mini-trajectories that replace smoothly,
  - no uncontrolled motion if PC updates stop.
  Status: `JOGV` live jog with acceleration ramping and watchdog deceleration is implemented. Queued mini-trajectory replacement remains a possible refinement if hardware tests show velocity jog is still not smooth enough.
- Add optional plane constraints:
  - XY plane at fixed Z,
  - XZ plane at fixed Y,
  - YZ plane at fixed X,
  - custom work plane later.
- Add drawing mode:
  - record the commanded TCP path,
  - record the actual reported TCP path,
  - preview replay before execution,
  - optionally simplify/smooth recorded points.
- Add safety:
  - workspace bounds,
  - joint limits,
  - singularity warnings,
  - deadman/live-enable requirement for real hardware,
  - stop on stale UI input or communication dropout.

Acceptance:

- Moving an X/Y/Z/Phi control produces smooth TCP motion in the intended Cartesian direction.
- The first usable version feels responsive without solving full endpoint IK for every slider event.
- Near singularities or joint limits, the UI shows that a direction is blocked or slowed instead of twitching or jumping.
- The viewport shows the estimated path and the actual executed TCP trail.
- Plane drawing can constrain motion to a selected plane and record the result.
- Stopping input stops motion without leaving stale targets streaming.

## Workpiece: Arm And Tool Calibration

### CAL-01: Manual Arm Geometry Calibration

Status: missing.

Work:

- Add a guided workflow for:
  - measured link dimensions,
  - DH rows,
  - joint zero offsets,
  - direction signs,
  - joint limits,
  - home pose,
  - safe pose,
  - movement tolerance.
- Save to `robot.local.yaml`.
- Treat MATLAB dimensions as an editable starting preset.

Acceptance:

- User can calibrate without editing YAML manually.
- App clearly separates example defaults, measured local values, and active values.

### CAL-02: Tool Calibration

Status: missing.

Work:

- Calibrate gripper TCP dimensions.
- Calibrate magnet TCP dimensions.
- Track which tool was active during calibration.
- Warn when tool dimensions change after calibration.

Acceptance:

- Tool geometry is not mixed accidentally between gripper and magnet.
- Calibration status is visible per tool.

### CAL-03: Calibration Validation

Status: missing.

Work:

- Validate FK at home pose.
- Validate named target reachability.
- Compare measured points against FK.
- Show likely causes when validation fails:
  - wrong tool length,
  - wrong zero offset,
  - wrong direction sign,
  - bad DH dimension.

Acceptance:

- User can run a calibration validation pass.
- Failures point to plausible causes.

### CAL-04: Named Positions

Status: partial.

Reality note: default named positions and backend validation exist, and the UI can preview/move them. Editable named-position management and save/delete workflows are still missing.

Work:

- Add editable named positions:
  - home,
  - safe,
  - pickup test,
  - dropoff A,
  - dropoff B,
  - user-defined saved positions.
- Validate named positions against joint limits and IK reachability.
- Store in `robot.local.yaml`.

Acceptance:

- Named positions are usable in manual control and tasks.
- Invalid saved poses are rejected or clearly marked.

## Workpiece: Encoders And Known Pose

### ENC-01: Base/Shoulder Encoder Settings UI

Status: missing.

Reality note: Settings currently shows encoder summary rows for every configured axis, including disabled elbow/wrist entries. Normal operator settings should expose only base/shoulder encoder setup for the current hardware plan.

Work:

- Show only base and shoulder encoder setup in normal Settings.
- Do not show elbow/wrist encoders as active concepts for now.
- Add editable values:
  - enabled,
  - CS pin,
  - zero offset,
  - direction sign,
  - readback tolerance,
  - fault tolerance.

Acceptance:

- Settings matches the actual hardware plan.
- User can save base/shoulder encoder settings.

### ENC-02: Config-Driven AS5048A Readback

Status: partial.

Reality note: `arm_controller.cpp` has optional compile-time AS5048A readback for base/shoulder using fixed build-time CS pins. It is not yet driven from PC-app encoder settings, and zero offset/direction handling is not yet applied in firmware status.

Work:

- Decide which SPI pins are compile-time versus runtime config.
- Make base/shoulder CS pins configurable.
- Read AS5048A values over SPI.
- Report raw and calibrated values.
- Apply zero offset and direction sign.
- Report valid/error state.

Protocol fields:

```text
enc=1100
e1=<deg>
e2=<deg>
```

Acceptance:

- UI shows live base/shoulder encoder angles.
- Bad/missing encoder is visible and not trusted silently.

### ENC-03: Encoder Known Pose And Homing

Status: missing.

Work:

- Add `Use Encoder Pose` or `Set Home From Encoders`.
- Mark pose source as:
  - manual,
  - setpose,
  - encoder,
  - mixed.
- Use encoders to establish known pose for base/shoulder.
- Store offsets in local config.

Acceptance:

- Robot can enter known-pose state from valid encoder readback.
- UI clearly shows pose source.

### ENC-04: Encoder Verification And Fault Detection

Status: partial backend plumbing.

Reality note: the backend can compare reported encoder angles to commanded targets and set an encoder fault, but this is not yet a guided workflow and has not been proven with real AS5048A hardware.

Work:

- After motion, compare commanded angle vs encoder angle.
- Warn above tolerance.
- Fault above hard threshold.
- Stop trusting pose after large mismatch.
- Add diagnostics:
  - target,
  - encoder,
  - error,
  - tolerance,
  - fault state.

Acceptance:

- Step loss or mismatch is detected.
- Robot does not continue pretending pose is accurate after large mismatch.

### ENC-05: Bounded Stepper Settle Correction

Status: deferred until readback and verification work on hardware.

Work:

- After move completion, compare encoder angle to target.
- Issue small correction move if error exceeds settle tolerance.
- Limit correction attempts.
- Fault after repeated failure.

Acceptance:

- Small final errors can be corrected.
- Large disagreement becomes a fault, not endless correction.

### ENC-06: Experimental Full Closed-Loop Stepper Control

Status: explicitly deferred.

Do not start this until `ENC-01` through `ENC-05` work on real hardware.

Future work:

- Real-time correction during motion.
- Wraparound handling.
- Backlash handling.
- Filtering.
- PID or equivalent control.
- Max correction rate.
- Stall detection.
- Explicit experimental config flag.

## Workpiece: Vision And Camera

### VISION-01: Live Annotated Camera Popup

Status: missing.

Reality note: detection refresh and annotated frame plumbing exist, but the UI still renders the camera inline in the Tasks side panel.

Work:

- Use the movable popup from `SHELL-02`.
- Show live USB camera feed.
- Overlay detected blobs.
- Show:
  - color label,
  - image coordinate,
  - calibrated robot coordinate when available.
- Add refresh/live mode.

Acceptance:

- Camera view is useful without crowding the side panel.
- Annotation and detection list match.

### VISION-02: Color Profile Editor

Status: partial or missing.

Work:

- Add editable color profiles.
- Use HSV thresholds for first pass.
- Save profiles to `robot.local.yaml`.
- Support enabling/disabling colors per task.
- Add minimum blob area and filtering settings.

Acceptance:

- Sorting colors are configurable from UI.
- Detection is not hardcoded.

### VISION-03: Camera-To-Robot Calibration

Status: backend helper exists, UI missing.

Work:

- Add 4-point planar calibration workflow.
- User clicks or enters 4 image points.
- User enters corresponding robot XY points.
- Save transform to `robot.local.yaml`.
- Show transformed robot coordinates for detections.
- Treat this as the simple fallback path if AprilTag pose calibration is not ready.

Acceptance:

- Detections can be mapped to robot-frame coordinates.
- Calibration can be completed from the UI.

### VISION-04: Detection State Contract

Status: missing.

Work:

- Define a clean detection object format:
  - id,
  - label,
  - confidence or quality score,
  - image x/y,
  - robot x/y/z when calibrated,
  - area,
  - timestamp.
- Keep vision output independent from task logic.

Acceptance:

- Task code consumes detections without depending on OpenCV internals.

### VISION-05: AprilTag World Anchors

Status: implemented as a first pass for the fixed workspace.

Working assumption: `DICT_APRILTAG_36H11` tags 0-3 are 40 mm squares placed inside the workspace. The configured coordinates are the four 478 x 315 mm workspace corners, not tag centers. Tag 0's bottom-left, tag 1's bottom-right, tag 2's top-right, and tag 3's top-left corner coincide with those workspace corners. Each printed top edge points toward robot +Y. The Logitech C270 currently uses a clearly labeled 55-degree-diagonal-FOV intrinsic estimate; per-camera checkerboard calibration remains required for accurate distortion and millimeter projection.

Reality note: `app/apriltag_calibration.py` now owns tag detection, world-corner generation, robust PnP solving/refinement, planar homography fallback, camera-pose inversion, reprojection/tilt/inlier/confidence metrics, frame accumulation, distortion-aware workspace-plane ray projection, and invalidation when camera pixel geometry changes. FastAPI exposes reset, capture/accumulate, status, save, and verify endpoints. Settings provides intrinsics entry, collection, save, verification, annotated frames, and metrics. `tools/calibrate_apriltags.py` provides the same fixed-camera workflow outside the GUI. Saving requires every configured tag to reach the minimum observation count and an accepted 6-DoF pose.

Work:

- Add an AprilTag calibration model:
  - tag id,
  - tag size mm,
  - tag pose in robot/world frame,
  - optional tag board/group id,
  - confidence/quality threshold.
- Detect AprilTags in the camera image.
- Estimate camera pose from one or more known tags.
- Save tag definitions and calibration settings to `robot.local.yaml`.
- Validate pose quality:
  - reprojection error,
  - number of visible tags,
  - tag viewing angle,
  - timestamp/freshness.
- Keep AprilTag calibration separate from color/object detection logic.

Acceptance:

- The system can report camera pose in robot/world coordinates from visible AprilTags.
- Bad or ambiguous tag pose estimates are rejected or clearly marked stale.
- The calibration state says which tags were used and how good the estimate was.

### VISION-06: Camera-Space To Robot-Space Object Mapping

Status: implemented as a first pass for objects on workspace Z=0.

Reality note: color-blob centers now use a saved accepted AprilTag camera pose to cast a camera ray onto workspace Z=0. Detection output records the camera-pose ID and coordinate source. The old four-point homography remains the fallback when no accepted pose exists. Bounding/quality checks against the physical workspace and non-planar object support remain open.

Work:

- Use the estimated camera pose plus camera intrinsics to map image detections into robot/world coordinates.
- For objects on the table/workspace plane, intersect camera rays with the calibrated plane.
- Store detections with both image-space and world-space fields:
  - pixel center,
  - projected robot x/y/z,
  - source camera pose id/timestamp,
  - projection quality.
- Keep the old homography/simple planar transform as a fallback.
- Add diagnostics for projection failures:
  - no valid camera pose,
  - no workspace plane,
  - ray does not intersect expected plane,
  - projection outside workspace bounds.

Acceptance:

- Detected objects appear in the robot frame at positions consistent with the camera pose.
- The UI can explain why an object has image coordinates but no trusted robot coordinates.

### VISION-07: Projected Camera Image In Robot View

Status: new idea, missing.

This is the "camera view lies flat in the viewport" idea. It should be treated as a visualization/calibration aid first, not as a control authority.

Work:

- Use camera intrinsics, camera pose, and workspace plane to texture-map the live camera image into the 3D robot viewport.
- Render the camera image as a plane/mesh in robot/world coordinates, aligned with the real table/workspace.
- Add controls to show/hide:
  - raw camera popup,
  - projected camera plane,
  - detected objects,
  - AprilTags,
  - camera frustum.
- Handle stale frames by fading or labeling the projected plane.
- Keep projection optional so normal robot control stays uncluttered.

Acceptance:

- The projected camera image visually lines up with AprilTag markers and detected objects in the 3D viewport.
- Stale or uncalibrated projection is clearly indicated.
- Turning the projection on/off does not affect task logic or robot motion.

## Workpiece: Task Workflow

### TASK-01: Dedicated Task Panel

Status: placeholder-level.

Reality note: task sequence builders and preview/execute endpoints exist, but the operator workflow still mixes task settings, inline camera, detection summaries, and execution controls in a prototype layout.

Work:

- Rebuild Tasks around:
  - task selector,
  - task-specific settings,
  - camera button,
  - detection list,
  - preview,
  - confirmation,
  - execute,
  - compact status/progress.
- Keep task logic separate from vision, IK, motion execution, and firmware transport.

Acceptance:

- Operator can choose a task and understand what will happen before execution.

### TASK-02: Pick-And-Place Template

Status: partial.

Work:

- Generate reusable sequence:
  - safe pose,
  - move above object,
  - descend,
  - close/enable tool,
  - lift,
  - move above drop zone,
  - descend,
  - open/disable tool,
  - lift,
  - return safe.
- Validate approach height, tool type, and target reachability.

Acceptance:

- Pick-and-place can be previewed and executed from a clear workflow.
- Invalid targets fail at preview time.

### TASK-03: Batch Color Sorting

Status: placeholder-level.

Work:

- Detect all visible colored objects.
- Group by selected color profiles.
- Map each color to a configured drop zone.
- Generate full pick-and-place sequence.
- Show object list and preview path.
- Require confirmation before execution.

Acceptance:

- User can choose which colors to sort.
- Sorting is configurable, not hardcoded.
- Full batch sequence is visible before motion.

### TASK-04: Task Abort And Recovery

Status: missing.

Work:

- Define what happens when a task stops mid-sequence.
- Preserve enough state to tell the user:
  - current step,
  - last completed step,
  - whether tool is holding an object,
  - safe recovery options.

Acceptance:

- Stopping a task does not leave the UI confused about robot/tool state.

## Workpiece: 3D View And Spatial Feedback

### VIEW-01: DH Segment Visualization

Status: implemented as a first pass.

Reality note: the 3D view now renders Standard DH `d` and `a` translations as
separate visible segments and can show labels/frame axes through the Frames
toggle. `L_2` is part of the active measured base side offset and is also shown
in the measured base support sketch.

Work:

- Use measured-prototype segment data to show DH `d` and `a` offsets.
- Add optional labels in diagnostics/calibration mode.
- Keep normal operator view less cluttered.

Acceptance:

- Calibration/debug mode makes the robot geometry understandable.
- Operator mode remains clean.

### VIEW-02: Object Markers In Robot Frame

Status: plumbing exists, workflow incomplete.

Work:

- Display calibrated detections in the 3D robot view.
- Use colored markers and labels.
- Keep object markers independent from preview/path clearing.
- Add a visibility toggle.
- When camera pose is available, show whether marker position came from:
  - simple planar homography,
  - AprilTag camera-pose projection,
  - manual/test input.

Acceptance:

- Detected objects appear in correct robot-frame locations after camera calibration.
- Preview clearing does not remove object markers.

### VIEW-03: Path And Target Layers

Status: partial.

Work:

- Separate layers for:
  - current arm,
  - draft preview,
  - commanded target,
  - task path,
  - object detections,
  - calibration markers.

Acceptance:

- Updating one visual layer does not accidentally erase unrelated information.

### VIEW-04: AprilTag And Camera Pose Overlay

Status: implemented as a first-pass calibration overlay.

Reality note: the viewport renders the configured tag squares and IDs on workspace Z=0 plus a camera body/frustum colored by accepted/rejected pose state. The overlay follows the existing Frames visibility toggle. Detailed stale/unknown-tag states and a dedicated overlay toggle remain open.

Work:

- Render configured AprilTags in the 3D robot/world view at their known poses.
- Render detected AprilTags with quality/status indication:
  - matched known tag,
  - unknown tag,
  - stale tag,
  - rejected low-quality estimate.
- Render the estimated camera body and camera frustum in the viewport.
- Show camera pose metadata in diagnostics:
  - position,
  - orientation,
  - tags used,
  - reprojection error,
  - age/staleness.
- Keep this as a calibration/debug layer by default, not always-on operator clutter.

Acceptance:

- The 3D viewport can show where the camera is relative to the robot.
- The user can visually confirm that AprilTags, camera pose, and robot frame agree.
- Pose quality problems are visible without reading logs.

### VIEW-05: Projected Camera Plane Layer

Status: new idea, missing.

Work:

- Add a viewport layer that displays the live camera frame projected onto the workspace plane.
- Align the projection using camera intrinsics, estimated camera pose, and workspace plane definition.
- Render detected objects on top of the projected image in the same robot/world coordinate frame.
- Add opacity and visibility controls.
- Clearly label states:
  - uncalibrated,
  - stale camera pose,
  - stale image frame,
  - projection valid.
- Keep the projected image visually subordinate to the robot/path layers so it does not hide motion preview.

Acceptance:

- The camera image appears flat in the 3D scene where the real workspace is.
- Detected object markers line up with the projected objects.
- The operator can use the view to understand where the camera sees objects relative to the robot.

## Workpiece: Firmware And Protocol

### FW-01: TB6600-Oriented Hardware Config

Status: partial.

Reality note: the UI/protocol no longer exposes software microstep pins, and sync is guarded while motion/live/task execution is active. Physical TB6600 timing, wiring, and config-sync behavior still need hardware validation.

Work:

- Remove software microstep pin fields from the UI.
- Keep microstep value in config for steps-per-degree math.
- Default driver model to `TB6600`.
- Allow config sync while armed only when:
  - controller is idle,
  - no path is running,
  - live jog is off,
  - no task is executing.
- Firmware must reject config sync during motion.

Acceptance:

- Settings match TB6600 physical DIP switch reality.
- Syncing config while armed is possible when idle, but blocked during motion.

### FW-02: Protocol Status Extensions

Status: implemented as a first pass.

Reality note: backend parsing and firmware status output include known pose, pose source, encoder availability, encoder angles, closed-loop mode, tool state, and tool value fields while keeping older status lines parseable.

Work:

- Keep old firmware compatibility.
- Support optional status fields:

```text
known=0|1
pose_source=<manual|setpose|encoder|mixed|unknown>
enc=1100
e1=<deg>
e2=<deg>
tool=<open|closed|moving|on|off|unknown>
```

Acceptance:

- Old status lines still parse.
- New status lines expose known pose, encoder state, and tool state.

### FW-03: Safe Stop And Fault Semantics

Status: partial.

Reality note: PC and firmware have Stop/ESTOP paths and motion cancellation. Tool-output fail-safe behavior cannot be complete until real tool IO exists.

Work:

- Keep internal emergency/fault handling.
- Use one normal visible Stop button in the UI unless later testing proves a visible E-stop is needed.
- Define firmware responses for stop, fault, clear fault, and reset.
- Ensure tool outputs go safe on stop/fault.

Acceptance:

- Stop behavior is understandable and consistent.
- Safety internals remain available even if the UI is simpler.

### FW-04: Queued And Blended Trajectory Following

Status: partial.

Problem: sending many independent `MOVEJ` commands can create stop-start behavior if the controller treats each waypoint as its own target. Smooth Cartesian motion needs a queued path or a higher-rate target-following mode with clear timing.

Reality note: the dashboard now uploads multi-waypoint hardware paths with `TRAJ BEGIN`, sequential `TRAJ POINT` lines containing `time_from_start_s`, and `TRAJ START`. The full-arm controller stores the timed queue, interpolates between points with a clamped cubic Hermite segment, and clears the queue on stop, E-stop, home, set-pose, disarm, config changes, or a new `MOVEJ`. The no-motor protocol stub accepts the same commands for safe integration testing. This has compiled, but real hardware smoothness still needs validation.

Work:

- Validate the queued timestamp protocol on real hardware.
- Tune controller-side interpolation/blending so the arm does not fully decelerate at every intermediate waypoint unless commanded.
- Support synchronized joint arrival across axes.
- Apply speed and acceleration limits consistently for steppers and servos.
- Add queue progress reporting if the UI needs it.
- Add stale-stream safety:
  - timeout if the upload is started but not completed,
  - controlled stop,
  - fault or stopped state if queue underruns unexpectedly.

Acceptance:

- A multi-waypoint path can execute smoothly without visible stop-start at every point.
- The controller reports active waypoint/progress.
- Stop/abort clears the queued trajectory safely.

## Workpiece: Diagnostics And Tests

### DIAG-01: Hidden Diagnostics Drawer

Status: partial.

Work:

- Keep event logs out of normal Tasks UI.
- Organize diagnostics into:
  - serial,
  - motion,
  - IK,
  - encoder,
  - config sync,
  - tool,
  - vision.
- Add copy/export if useful.

Acceptance:

- Debug information is available without cluttering normal operation.

### TEST-01: Backend Unit Tests

Status: partial but currently green.

Reality note: the 2026-06-18 continuation ran `python -m pytest -q`: 92 tests passed. Coverage is still backend-heavy and does not replace real hardware tests.

Work:

- Test config precedence and saving to `robot.local.yaml`.
- Test tool config validation.
- Test DH config load/save.
- Test FK against measured-prototype known poses.
- Test analytic seed candidate behavior.
- Test Jacobian IK success/failure.
- Test encoder parsing.
- Test armed idle-only config sync.

Acceptance:

- Core math/config/protocol behavior is covered by repeatable tests.

### TEST-02: UI Regression Tests

Status: missing.

Work:

- Test:
  - preview persistence,
  - live jog stability,
  - tool switching,
  - serial modal,
  - camera popup,
  - DH table save,
  - diagnostics drawer.

Acceptance:

- Common UI regressions are caught automatically.

### TEST-03: Firmware Build And Protocol Tests

Status: partial.

Reality note: the 2026-06-17 audit built `esp32-s3-arm-protocol-stub` and `esp32-s3-arm-controller` successfully with PlatformIO. There are still no automated firmware protocol tests running against the compiled binaries or hardware.

Work:

- Build protocol stub and controller firmware.
- Test:
  - `TOOL ON/OFF`,
  - `TOOL OPEN/CLOSE/SET`,
  - encoder status fields,
  - config while armed but idle,
  - config rejection during motion,
  - safe tool state on stop/fault.

Acceptance:

- Firmware/protocol changes do not silently break the dashboard.

## Open Questions Before Hardware-Heavy Work

These should be answered before implementing hardware-heavy packages.

- Are the MATLAB `L_1..L_9` values the newest measurements?
- Is the current `L_2` sign/direction correct after measuring against the physical arm?
- Are `s4 = -1`, `s6 = -1`, and `s8 = 1` final for the current build?
- Does the app coordinate frame match the MATLAB coordinate frame?
- Are the MATLAB joint limits final and mechanically safe?
- Should elbow-down be the default for all demo tasks?
- What are the gripper servo pin, pulse min/max, open value, and close value?
- What are the electromagnet transistor GPIO pin and active polarity?
- What is the safe default magnet output state?
- What are the real gripper and magnet TCP offsets?
- Are AS5048A encoders mounted at the base and shoulder joint outputs?
- What SPI pins and CS pins are actually wired?
- Are encoder zero positions mechanically repeatable?
- What camera source index, resolution, and mount height will be used?
- Which AprilTag family should be used?
- What AprilTag physical size will be printed or mounted?
- Where will AprilTags be mounted relative to the robot base and workspace plane?
- Is the camera fixed to the frame/table, mounted on the robot, or movable?
- Do we need full camera intrinsics calibration, or is a simple approximate focal model enough for the first demo?
- What is the exact workspace plane height relative to the robot base?
- What are the first demo object colors and drop zones?
- What speed and acceleration values are safe for the physical arm?
- What TCP speed and TCP acceleration should be considered safe for first Cartesian live jog tests?
- Should live Cartesian jogging be allowed on hardware immediately, or simulation-only until the queued `TRAJ` follower has been validated on the real arm?
- What fixed update rate is realistic for PC-to-controller target streaming over the chosen serial/Bluetooth link?
- Which plane drawing mode matters first: XY table plane, vertical XZ/YZ plane, or arbitrary custom plane?

## Suggested Next Implementation Requests

Good focused requests:

```text
Implement SHELL-01 through SHELL-03 only.
```

```text
Implement TOOL-01 and TOOL-02 only. Do not change firmware yet.
```

```text
Implement KIN-01 through KIN-03 and add tests against the MATLAB geometry.
```

```text
Finish KIN-05 by replacing the numeric Jacobian with the DH-frame cross-product Jacobian and surfacing IK diagnostics in the UI.
```

```text
Implement MOVE-02 as preview-only. Do not use it for hardware execution yet.
```

```text
Implement MOVE-06 first so planned and actual TCP paths are drawn separately.
```

```text
Implement MOVE-07 in simulation only: TCP speed/accel-limited Cartesian preview.
```

```text
Implement MOVE-08 in simulation only: live Cartesian drag/jog with X/Y/Z/Phi controls using differential IK.
```

Avoid broad requests like:

```text
Implement the whole remaining plan.
```

The project will move faster if each workpiece is implemented, tested, and reviewed separately.
