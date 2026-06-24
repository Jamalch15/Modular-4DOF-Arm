# ARES-4 PC Control App

This folder contains the PC-side dashboard and planning backend for the ARES-4
robot arm prototype. It runs locally in a browser and handles the operator
interface, robot model, kinematics, trajectory preview, task sequencing,
calibration workflows, vision integration, and serial communication with the
ESP32-S3 controller.

For the full repository overview, see [../README.md](../README.md).

The operator-facing Cartesian TCP calibration workflow is documented in
[../docs/kinematics_calibration.md](../docs/kinematics_calibration.md). Shoulder
encoder authority and correction semantics are documented in
[../docs/shoulder_encoder_integration.md](../docs/shoulder_encoder_integration.md).

Current scope:

- Python backend with FastAPI and WebSockets
- Browser dashboard on localhost
- Manual Control tab for four rotary joints and active tool controls
- Standard DH forward kinematics and Jacobian IK sandbox
- Cartesian target preview with ghost arm, target marker, and path line
- Task panel with a movable live camera popup, multi-object workspace color detection, and pick/place or color-sorting preview
- Persistent staged program builder with per-step preview and adaptive demo templates
- Simulation mode by default
- Serial transport abstraction for later ESP32-S3 control
- Persistent Hardware IO settings for pins, TB6600 microstep value, gear ratios, servo pulse mapping, tools, and encoders
- ESP hardware-config sync on serial connect and Settings save
- Safety checks for joint limits, known pose, stop, armed hardware mode, live-motion gating, and rate-limited motion
- Position Library, task-destination, task, tool, vision, diagnostics, and encoder-readback APIs for the demo path
- AprilTag workspace calibration with multi-frame accumulation, camera-pose quality metrics, saved planar fallback, and 3D camera/tag overlays
- Working inverted `DICT_4X4_50` ArUco homography integration from `vision_robot_project.zip`
- Detector-neutral vision contract plus `/api/vision/project` for future YOLO/AI detections

Not included in this iteration:

- Real homing switches
- Full closed-loop encoder correction

## Working Assumptions

These are assumptions, not final design decisions.

- Joint 1, base: stepper motor
- Joint 2, shoulder: stepper motor
- Joint 3, elbow: servo motor
- Joint 4, wrist: servo motor
- Initial PC-to-controller transport: USB serial
- Bluetooth may be added later as another transport implementation
- Exact link lengths, gear ratios, zero offsets, pin assignments, servo pulse ranges, and homing hardware are not decided
- Hardware feedback is open-loop for now: steppers report commanded step counts and servos report commanded angles
- Shoulder AS5048A readback is calibrated per-joint evidence. It does not establish the full pose; bounded post-move correction remains disabled until locally validated.

## Quick Start After Restart

`localhost` only works while the FastAPI server is running. After restarting the PC, the old server process is gone, so start it again from PowerShell:

```powershell
cd "C:\Users\chark\Desktop\DTU\4 Semester\Mechatronics\Mechatronics Project Files\pc_app"
.\.venv\Scripts\Activate.ps1
uvicorn app.main:app --reload --host 127.0.0.1 --port 8000
```

Then open:

```text
http://127.0.0.1:8000
```

If the virtual environment does not exist yet, run the full setup below first.

## First-Time Setup

From this folder:

```powershell
cd pc_app
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Run the app whenever you want to use the dashboard:

```powershell
uvicorn app.main:app --reload --host 127.0.0.1 --port 8000
```

Open:

```text
http://127.0.0.1:8000
```

Simulation mode is enabled by default in `config/robot.example.yaml`, so the dashboard should work without an ESP32-S3 connected. Machine-specific values are saved to `config/robot.local.yaml` when present.

General robot poses are stored in the Position Library. Color-sorting mappings
select Position Library records directly in Tasks. The backend still writes
lightweight `task_destinations` references and legacy `named_positions` /
`drop_zones` mirrors so existing local configurations remain usable during
migration.

## Startup Troubleshooting

If `http://127.0.0.1:8000` does not load:

1. Make sure the PowerShell window running `uvicorn` is still open.
2. Check that you are in the `pc_app` folder before running the command.
3. Activate the venv with `.\.venv\Scripts\Activate.ps1`.
4. If PowerShell blocks activation, run:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

5. If port 8000 is already in use, use another port:

```powershell
uvicorn app.main:app --reload --host 127.0.0.1 --port 8001
```

Then open `http://127.0.0.1:8001`.

6. If dependencies are missing, reinstall them:

```powershell
python -m pip install -r requirements.txt
```

## Stop, Restart, And Reset

### Stop The Server

In the PowerShell window running Uvicorn, press `Ctrl+C`. Closing that window
also stops localhost.

### Restart The Server

Stop it with `Ctrl+C`, then run:

```powershell
.\.venv\Scripts\Activate.ps1
uvicorn app.main:app --reload --host 127.0.0.1 --port 8000
```

Use `Ctrl+F5` in the browser if it still displays old frontend files.

### Clear A Stuck Port

If Uvicorn reports that port 8000 is already in use:

```powershell
$processId = (Get-NetTCPConnection -LocalPort 8000 -State Listen).OwningProcess
Stop-Process -Id $processId
```

Start Uvicorn again afterward. Only stop the process after confirming it is
the old localhost server.

### Reset Saved App Settings

The app stores machine-specific settings and calibration in
`config/robot.local.yaml`. To return to the tracked simulation-safe defaults,
stop Uvicorn and rename that file:

```powershell
Move-Item .\config\robot.local.yaml .\config\robot.local.backup.yaml
```

Restart Uvicorn. The app will load `config/robot.example.yaml`. This resets
saved hardware IO, geometry/calibration, tools, camera settings, and other
local changes. To restore the old settings, stop Uvicorn and rename the backup
to `robot.local.yaml`.

## User Guide

The dashboard is a sandbox for testing the arm model and motion behavior before trusting real hardware.

The bottom-left build indicator verifies the running localhost instance every
15 seconds:

- `Current localhost <commit>` means the browser assets match the current
  HTML/JS/CSS files, the running Python backend matches the Python files on
  disk, and the loaded robot settings match the config file.
- `Current localhost <commit> - remote differs` means localhost is fresh, but
  local `HEAD` does not match `origin/main`.
- `Browser outdated` means HTML/JS/CSS changed; click the indicator to reload
  the tab with build-keyed asset URLs.
- `Backend outdated` means Python files changed without the server process
  reloading. With the documented `uvicorn --reload` command this should clear
  automatically; otherwise restart localhost.
- `Settings file changed` means the config file changed outside the loaded
  runtime config; restart localhost before trusting the displayed settings.

The frontend and backend checks are intentionally separate. FastAPI serves
static files directly from disk, while imported Python code remains in the
running process until Uvicorn reloads it. Uvicorn's default reload filter
watches Python files, not HTML/JS/CSS, so frontend edits are handled by browser
refresh and backend edits by the Uvicorn reloader.

### Layout

- Left panel: operation tabs and settings.
- Right viewport: live 3D robot view, preview ghost arm, target marker, and path line.
- Top-right rail: simulation, serial picker, disconnect, home, stop, diagnostics, and armed toggle.
- View widgets: HUD, target faders, preview/path/frame toggles, reset view, and Live Real.

### Control Tab

Use this for direct joint-angle experiments.

- Move joint sliders or type angles to update the preview ghost.
- `Apply` sends the current joint preview through the backend motion path.
- The preview stays visible until the reported pose reaches the commanded target, a newer target replaces it, or you press `Reset`.
- `Live joint jog` sends rate-limited joint updates while editing.
- The Tool panel switches between gripper controls and magnet controls from saved tool config.
- Keep movements small when the physical arm is connected.

### Tasks Tab

Use this for demo workflows.

- Select `Color Sorting` or `Pick and Place`.
- Refresh the camera panel to run current color/blob detection.
- Preview builds the pick/place sequence before execution.
- Detected objects can be shown in the 3D view when camera calibration is available.

### Kinematics Tab

Use this for Cartesian target experiments.

- Set `x`, `y`, `z` in millimeters and `phi` in degrees.
- With `Cart Jog` off, the sliders and viewport faders edit an IK target and only update the preview.
- With `Cart Jog` on, the viewport faders become Cartesian velocity controls. Simulation can run directly; hardware also requires `Live Real`, `Armed`, a known pose, and valid synced configuration.
- Releasing the last active fader sends `JOG STOP` and clears the live jog stream. It does not execute the previewed endpoint.
- `Mode` selects joint-space or Cartesian linear path generation.
- `Branch` selects the numerical IK seed preference.
- `Preview` builds a path and updates the ghost arm, target marker, and path line.
- `Execute` runs the accepted preview.
- The active model is edited from Settings. Kinematics uses the saved derived DH rows.

### Program Tab

Use the staged `Library -> Build -> Preview -> Run` workflow for reusable
motion sequences.

- `Library` creates, saves, loads, duplicates, and deletes local user programs.
- Built-in read-only demos include an adaptive air square, a 24-segment air
  circle, and a conservative joint-space kinematic showcase.
- Copy a built-in before editing it. Built-ins adapt their coordinates or
  joint amplitudes to the active robot geometry and limits.
- `Build` supports manually entered joint angles and Cartesian targets,
  Position Library records, the current reported pose, the current IK target,
  and available vision/task targets.
- Every Cartesian step explicitly selects joint-space or linear TCP motion.
- The selected Build step can be previewed directly from the current reported
  robot pose. `Go to target` only unlocks for that exact unchanged preview and
  executes the displayed path through the normal safety gates.
- `Preview selected step` plans through all preceding enabled steps so the
  selected step is checked in sequence context.
- Planning a saved user program persists its compiled trajectory. Loading the
  program restores that plan automatically when the program, planner code,
  robot configuration/model, and starting joint pose still match.
- If a run finishes back at the saved starting pose, the same compiled plan is
  restored automatically for another run. Otherwise the program must be
  planned again from the new pose.
- User programs persist in ignored `config/programs.local.json`; the file uses
  a versioned JSON schema and also stores the optional compiled plan. It is not
  committed with machine-specific data.

### Settings Tab

Use this as the shared robot model for both FK and IK.

- Edit measured geometry, joint limits, home pose, zero offsets, direction signs, and motion defaults.
- Derived link values and Standard DH rows are shown read-only so the saved model can be inspected without creating a second dimension input path.
- Edit Hardware IO placeholders for stepper pins, TB6600 microstep value, gear ratios, servo pulse range, and enabled axes.
- Draft edits do not affect FK/IK until you press `Save`.
- After saving, the backend reloads config, refreshes the dashboard from the saved model, and tries to resync the ESP if serial hardware is connected.

### Workspace Camera Calibration

The Settings tab uses one planar calibration path for object coordinates and
the projected camera layer. It detects square tags 0-3 using the configured
ArUco/AprilTag dictionary candidates and does not require camera intrinsics.

- Robot `X` is sideways, robot `Y` is forward, and the work plate is `Z=0`.
- Each sample records the tag center for layout diagnostics and the physical
  outer corner for the coordinate fit. Printed marker rotation is ignored.
- Twelve observations per required center and outer corner are median-filtered.
- `Calibrate workspace` collects, validates, and saves the map in one action.
- Saving is blocked when outer-corner fit or tag-center layout exceeds the
  configured millimeter limits.
- Verification projects a fresh tag frame through the saved map and reports
  center/corner errors in millimeters.
- Normal operation never recalibrates from live tags.
- The live camera texture uses a dedicated fast saved-map endpoint and is
  placed on the matching robot-coordinate polygon in the 3D view.
- The Settings preview updates only after calibration or verification. Color
  detection stays in Tasks.

Camera intrinsics and the separate 6-DoF pose implementation remain optional
developer functionality. They are not used by the operator workspace workflow.

### Hardware IO

The Settings tab is the single source of truth for physical pin and actuator mapping.

- All axes start disabled with unknown pins set to `-1`.
- Base and shoulder are currently modeled as steppers.
- Base and shoulder stepper drivers are currently modeled as TB6600 drivers; microstepping is set on physical DIP switches but still stored as a numeric value for steps-per-degree math.
- Elbow and wrist are currently modeled as 270 degree servos.
- Disabled axes are simulated by the controller and shown as `simulated`.
- Enabled valid axes are shown as `hardware`.
- A mix of physical and simulated axes is shown as `mixed`.
- Enabled axes with missing or invalid pins are shown as `invalid` and block hardware arming.
- `Sync Hardware` sends the saved config to the ESP and checks for acknowledgement.

### Live Real And Safety

Working assumptions:

- Simulation can preview and execute without hardware.
- Real hardware movement requires the backend to be connected to the controller.
- Real hardware movement requires hardware config sync to be `synced`.
- `Armed` is required before hardware execution or Live Real hardware movement.
- `Live Real` is always off after page reload and disconnect.
- `Stop` cancels active movement.

Use `Preview` first, check the ghost/path visually, then arm and execute only when the physical workspace is clear.

If hardware reports `known=0` or asks for `SETPOSE`, leave hardware disarmed, move the joint sliders to match the real arm's current physical pose, and click `Set Pose` in the top robot-control rail. This tells the controller and PC that the current pose is known; then you can arm and run commands.

## Tests

```powershell
cd pc_app
pytest
```

The first tests cover config loading, joint limit validation, emergency-stop behavior, smoothing/rate limiting, FK, and protocol parsing/formatting.

For live Cartesian jog debugging, run the repeatable simulator sweep:

```powershell
cd pc_app
python tools/debug_cartesian_jog.py
```

It checks known joint poses against X/Y/Z jog commands and reports progress, lateral drift, alignment, and blocked steps. Near singularities, blocked directions are intentional: the solver rejects a local step that would be unreachable or drift sideways. Rejected samples are not accumulated, so a valid reverse command can proceed immediately.

## Configuration

Use `config/robot.example.yaml` as the tracked template. The app prefers `config/robot.local.yaml` when it exists and saves measured calibration there so private hardware values do not have to be committed.

Important fields:

- `geometry`: measured dimensions and signs used as the editable robot model source
- `links_mm`: derived compatibility link dimensions in millimeters
- `joints[].limits_deg`: conservative joint limits in degrees
- `joints[].home_deg`: configured home/reset pose
- `joints[].max_speed_deg_s`: per-joint speed limit
- `motion.smoothing_alpha`: smoothing applied to target changes
- `motion.acceleration_deg_s2`: simple acceleration limit
- `motion.allow_sudden_jumps`: keep `false` unless intentionally testing jumps
- `serial.port` and `serial.baud_rate`: initial USB serial settings
- `kinematics.dh_rows`: derived Standard DH rows used for FK and Jacobian IK
- `joints[].hardware.stepper`: step/dir/enable pins, driver model, full steps per rev, microsteps, and motor-to-joint gear ratio
- `joints[].hardware.servo`: PWM pin, pulse min/max, PWM frequency, servo range, neutral angle, and servo-to-joint gear ratio
- `tools`: active gripper or magnet dimensions and IO settings
- `encoders`: staged AS5048A readback and verification settings
- `camera.calibration.workspace_aruco`: planar tag layout, saved pixel
  references, workspace polygon, and millimeter quality thresholds
- `camera.intrinsics`: optional camera matrix for developer-only 3D pose work

## Coordinate Frame And Kinematics

The editable model starts from measured geometry. The app derives compatibility link dimensions and Standard DH rows from that geometry so FK, IK, and the 3D view use one coherent model.

Working assumption:

- Origin is the center of the base rotation axis on the mounting plane
- +Z points upward
- The arm points along global +Y when the base joint is 0 deg
- +X is horizontal sideways after base rotation
- Shoulder, elbow, and wrist are pitch joints in the vertical radial plane
- Shoulder 0 deg points the upper arm vertically upward
- Positive shoulder, elbow, and wrist angles bend the chain toward the local horizontal reach direction
- `phi` is the current tool angle target used by the numerical IK workflow.
  Auto-phi globally prioritizes reachable angles below `-90 deg`, with
  `-100 deg` as the default downward-forward preference.
- The measured prototype geometry currently derives `d1 = L1 + L3`, base side
  offset `L2`, `d2 = s4*L4`, `a2 = L5`, `d3 = s6*L6`, `a3 = L7`,
  `d4 = s8*L8`, and `a4 = L9`

Lengths are in millimeters internally. Trigonometry uses radians internally. UI and config use degrees for joint angles.

## Calibration Notes

Before using real hardware, measure and update:

- Link lengths as center-to-center joint distances
- Base height from mounting plane to shoulder axis
- Tool/end-effector offset from wrist axis
- Joint zero angles
- Positive rotation direction for each joint
- Conservative software joint limits
- Stepper driver type, microstepping, gear ratio, and steps per revolution
- Stepper degrees per step at the joint after gearing
- Servo pulse min/max values for safe mechanical range
- Servo angle mapping and any gear ratio between servo and joint
- Homing switch or hard-limit availability

Recommended calibration workflow:

1. Set all joint limits narrower than the physical range.
2. Define a repeatable zero pose for each joint.
3. Verify positive direction with small manual moves.
4. Measure several physical end-effector positions.
5. Compare measured positions against FK.
6. Adjust link lengths and zero offsets.
7. Only widen limits after repeated safe tests.

## Serial And Hardware Mode

There are currently three ESP-side firmware choices in `../controller_firmware/platformio`:

- `main.cpp`: preserved single-axis stepper/servo test firmware.
- `protocol_stub.cpp`: no-motor safe protocol parser.
- `arm_controller.cpp`: full-arm open-loop controller that accepts dashboard config and moves only enabled valid axes.

The current line-based protocol is documented in `../controller_firmware/protocol_stub.md`.

Core commands:

```text
HELLO
STATUS
CONFIG BEGIN / CONFIG JOINT / CONFIG END
CONFIG ENCODER_BUS / CONFIG ENCODER / CONFIG ENCODER_POLICY
ARM 0|1
SETPOSE j1 j2 j3 j4
MOVEJ j1 j2 j3 j4 speed accel
JOGJ j1 j2 j3 j4 speed accel
JOGV v1 v2 v3 v4 accel
SERVOJ j1 j2 j3 j4 duration_s
JOG STOP
TRAJ BEGIN count=N duration=seconds speed=deg_per_s accel=deg_per_s2
TRAJ POINT index=i t=seconds j1=deg j2=deg j3=deg j4=deg
TRAJ START
TRAJ CLEAR
CORRECTJ joint=2 delta=deg speed=deg_per_s accel=deg_per_s2 id=transaction
STOP
ESTOP
HOME
TOOL OPEN|CLOSE
TOOL SET value=0.000
TOOL ON|OFF
```

Current status response:

```text
STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=open_loop_estimate armed=1 hw=mixed enabled=1100 enc=0100 enc_valid=0100 er2=8192 ea2=180.0 em2=20.0 eage2=40 enoise2=0.08 evalidn2=4 ef2=none j1=0.0 j2=20.0 j3=20.0 j4=0.0 closed_loop=diagnostic correction=idle correction_id=none correction_delta=0 correction_steps=0 correction_attempts=0 cb1=0 cb2=0 cb3=0 cb4=0 tool_type=generic tool=unknown tool_value=0.000 fault=OK
```

Protocol v4 keeps `j1..j4` as open-loop controller estimates. `er2`, `ea2`, and `em2` are raw count, raw angle, and calibrated shoulder angle. `enc_valid`, consecutive-valid count, sample age, noise, and flags determine whether the measurement has authority. Diagnostic encoder fields do not make the complete robot pose known. If **Use encoder as shoulder pose while idle** is enabled, fresh calibrated shoulder evidence can update only the shoulder planning/estimated angle while the robot is idle/stopped.

See `../docs/shoulder_encoder_integration.md` for the state contract, calibration workflow, fault semantics, and disabled-by-default bounded correction rules.

Operator workflow for the shoulder encoder:

1. In Settings, enable encoder readback and the shoulder AS5048A, use the recommended ESP32-S3 pins unless your wiring differs, save, and sync while disarmed.
2. Confirm Diagnostics shows a stable raw AS5048A angle. This raw angle is magnet degrees, not a calibrated shoulder angle.
3. Put the shoulder at one physically known angle, enter that angle, and use **Set Pose to known angle** if the planner does not already match the real mark.
4. Disarm and use **Quick calibrate**. This stores the current raw AS5048A angle as the calibrated reference for that known shoulder angle. The fast path assumes joint-output mounting and one sensor turn per shoulder turn.
5. Keep **Use encoder as shoulder pose while idle** enabled for the normal workflow. When the arm is idle/stopped and the sample is fresh/stable, the shoulder value in the Control tab and 3D view follows the calibrated encoder.
6. Before hardware motion, the controller must also be rebased to that encoder-tracked shoulder. If you are disarmed, turning **Armed** on sends `SETPOSE` first, then arms. If you are already armed and the controller step position is stale, normal motion is blocked; disarm and arm again before moving.
7. Use normal Preview / Execute for motion. Small encoder-tracked shoulder drift between Preview and Execute is accepted and the first trajectory point is rebased to the current shoulder. Larger drift still asks for a new preview.
8. Arm and use **Run backlash check** if you want to measure reversal error. The app approaches the same shoulder angle from below and above and reports the output-side branch separation.
9. Enable post-move correction only with **Validate + enable post-move correction** after the encoder is stable and the current mismatch is within the configured correction limit.
10. Use the optional assisted sweep only when you need range validation or a `piecewise_linear` sensor map.

Post-move correction is not continuous closed-loop control. It only runs after eligible manual joint endpoint moves, Go Home, or the explicit **Align** button, within configured limits. `deadband_deg` is the small-error zone where no correction is attempted. `max_delta_deg` is the automatic post-move correction cap. `align_max_delta_deg` is the larger cap for the manual **Align** action: the first alignment move may correct the full measured shoulder error up to this cap, then the app rechecks and performs smaller cleanup corrections if needed. `fault_tolerance_deg` controls warning/fault behavior only; raising it does not increase the correction movement range. Go Home remains a planned move to the configured home pose followed by optional shoulder verification/correction. A single shoulder encoder can make the shoulder angle known/tracked, but it still does not make base, elbow, wrist, or the full TCP pose measured.

Working assumption: the PC remains the planner. Previewed joint-space,
Cartesian, and program moves upload a timed `TRAJ` queue so the controller
follows the planned waypoint timestamps. `MOVEJ` remains a low-level endpoint
protocol command, but the normal dashboard execution path does not collapse a
previewed joint trajectory to one endpoint. Live Cartesian jog runs one
fixed-rate PC servo loop: it ramps TCP velocity, solves a direction-preserving
bounded differential IK problem, and streams synchronized short-duration joint
position segments with `SERVOJ`, followed by `JOG STOP` on release. The firmware
does not apply a second independent joint-velocity ramp to these segments.
Preview-only IK target editing does not send motion. `JOGJ` and `JOGV` remain
compatibility commands. This is still open-loop target following for axes
without encoders, not final measured closed-loop Cartesian control.

## Bluetooth Notes

Bluetooth is intentionally not implemented yet.

Later, add a Bluetooth transport that implements the same high-level interface as the serial transport. The robot state, safety checks, FK, motion smoothing, and UI should not need to change when the transport changes.
