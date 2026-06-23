# Revised Implementation Plan From `comments.md`

Status: proposed source of truth for implementation order after review  
Prepared: 2026-06-20  
Scope: PC dashboard, motion/kinematics integration, controller protocol, task workflows, and hardware-feedback staging

## 1. Executive Summary

### What the project currently is

The repository contains a simulation-first robot-arm control stack with:

- a FastAPI backend and WebSocket state stream;
- a large vanilla-JavaScript operator dashboard with Control, Tasks,
  Kinematics, Program, and Settings tabs;
- a Three.js robot/workspace view;
- Standard-DH FK, numerical endpoint IK with analytic seeds, and a geometric
  Jacobian used by live Cartesian servoing;
- joint, Cartesian-linear, program, and task trajectory generation;
- a line-based serial protocol and ESP32-S3 open-loop controller;
- planar camera calibration, color detections, projected camera imagery, and
  color-sorting task generation;
- tool configuration and first-pass gripper/magnet firmware IO;
- optional, incomplete AS5048A readback plumbing;
- a sizeable backend test suite.

This is no longer only a placeholder application. Many requested features
exist as first passes. The main problem is that the features do not yet share
one sufficiently explicit state, calibration, and motion contract.

### What the app is trying to become

The target is a reliable operator console where:

- the current robot pose and its confidence/source are unambiguous;
- previews are tied to the pose and configuration they were planned from;
- FK, IK, tool TCP, physical calibration, and visualisation use one declared
  transform chain;
- every movement source uses the same safety and motion constraints;
- general position presets, task drop zones, programs, and detections are
  separate reusable concepts;
- simulation remains useful without pretending that open-loop estimates are
  measured physical truth;
- hardware feedback is added incrementally and cannot cause unbounded
  correction.

### Main architectural risk

The primary risk is ambiguous authority.

The backend owns `target_angles_deg` and `reported_angles_deg`, while the
frontend independently owns `draftAngles`, `pendingAngles`,
`commandedAngles`, `previewAngles`, IK target state, program state, task
previews, and viewport layers. A path preview records neither the reported
pose revision nor a start-pose tolerance. `/api/path/execute` can therefore
execute a trajectory planned from an older pose after another tab or command
has moved the arm.

This is a plausible root cause for mixed previews, stale controls, first-home
display problems, and aggressive corrective jumps.

There are also separate but related risks:

- `POST /api/home` now routes through normal planned trajectory execution and
  does not send the firmware `HOME` command.
- Firmware marks `homed=true` when a home target is accepted, not when a
  physical homing procedure or completed move proves the pose.
- There are no real homing switches in the current documented scope, so
  "Home" is currently "move to configured home", not pose-establishing homing.
- Hardware actuator zero/sign settings and DH model zero/sign settings use
  similar names but have different roles that are not clearly documented.
- A hardware config sync can reinterpret step counts and servo mapping at the
  current logical pose. Calibration-changing sync should not be treated like a
  harmless settings refresh.
- Optional encoder code can treat one available base/shoulder encoder as
  enough to make the whole 4DOF pose known, and raw encoder angles are not yet
  configured by the PC-side encoder settings.

### Recommended implementation philosophy

1. Make state transitions explicit before adding operator features.
2. Reject stale plans instead of silently trying to recover from them.
3. Separate estimated, commanded, and measured state.
4. Treat calibration as a transform chain with named frames and units.
5. Route all motion through one planner/executor contract.
6. Keep hardware-changing configuration conservative and disarmed.
7. Build diagnostics before closed-loop correction.
8. Preserve simulation, but label its stronger state assumptions clearly.
9. Prefer small schemas and compatibility migrations over another parallel
   set of settings.

### Immediate safety findings

These should be addressed before broader feature work:

1. **Stale previews are executable.** `build_preview()` starts from
   `state.reported_angles_deg`, but stored entries in `path_previews` do not
   capture a pose revision, and `execute_path()` does not compare the current
   pose with the preview start.
2. **Home is double-dispatched.** `home()` calls `set_targets()` and then sends
   `format_home()` on hardware. This creates two command paths for one operator
   action.
3. **"Homed" is overstated.** `handleHome()` in firmware sets `homed = true`
   immediately after setting a target. No switch-based homing exists yet.
4. **Config sync can change actuator interpretation.** `CONFIG END` calls
   `syncRuntimeFromCurrentPose()` after replacing zero offsets, signs, gearing,
   and IO mappings. Known-pose validity must be reconsidered when these fields
   change.
5. **Partial encoder availability can overstate pose knowledge.**
   `knownPose = homed || encoderAvailable[0] || encoderAvailable[1]` is not
   sufficient for a four-joint known pose.
6. **The real arm cannot be guaranteed to equal the simulation while it is
   open loop.** The UI can guarantee that it shows the controller's best known
   estimate and clearly labels its source. Exact physical agreement requires
   validated feedback.

## 2. Current Architecture Overview

### Frontend structure

The frontend is a single-page, framework-free dashboard:

- `pc_app/app/static/index.html`
  - defines the five main tabs;
  - includes the robot control rail, HUD, Cartesian faders, camera popup, and
    diagnostics drawer;
  - already includes a staged task workflow and a multi-section Program
    builder.
- `pc_app/app/static/app.js`
  - owns almost all UI state and API/WebSocket interaction;
  - contains Control, Tasks, Kinematics, Program, Settings, camera calibration,
    TCP calibration, tool, and serial workflows in one approximately
    5,700-line module;
  - stores local intent in `draftAngles`, `pendingAngles`,
    `commandedAngles`, `previewAngles`, `ikUserEdited`,
    `programWaypoints`, task drafts, and preview IDs;
  - uses `jointControlAngles()` precedence:
    draft → pending → preview → commanded → backend target → reported pose.
- `pc_app/app/static/robot_view.js`
  - independently reimplements the DH transform chain for rendering;
  - renders the current arm, ghost arm, planned path, actual path, objects,
    task markers, camera projection, and optional calibration overlays;
  - applies the active tool TCP offset;
  - currently draws the TCP offset as a link and point, while DH frame axes are
    drawn only at DH frames.

Architectural concern: `app.js` is both a UI controller and a second state
machine. Its state is not versioned against backend pose/config changes.

### Backend structure

`pc_app/app/main.py` is the integration layer and currently contains:

- FastAPI request models and all API routes;
- global runtime state and preview caches;
- serial transport orchestration;
- hardware sync and arming gates;
- path/task execution coroutines;
- motion diagnostics;
- camera/workspace calibration endpoints;
- TCP command-correction calibration endpoints;
- WebSocket broadcasting.

Important supporting modules:

- `robot_state.py`: compact backend `RobotState`.
- `motion.py`: joint, linear Cartesian, and program trajectory builders plus
  simulation rate limiting.
- `kinematics.py`: DH FK, TCP application, analytic IK seeds, endpoint IK,
  numerical Jacobian, and geometric task Jacobian.
- `cartesian_servo.py`: fixed-rate live Cartesian servo with task-space and
  joint-space limits.
- `cartesian_calibration.py`: tool-specific Cartesian command correction.
- `tasks.py`: pick/place and color-sorting settings, filtering, assignment,
  and sequence generation.
- `vision.py`, `workspace_calibration.py`, and
  `apriltag_calibration.py`: camera capture, planar projection, detection
  normalisation, and calibration.
- `demo_settings.py`: compatibility/default layers for named positions, drop
  zones, tools, tasks, encoders, and geometry.
- `protocol.py` and `serial_client.py`: protocol formatting/parsing and serial
  IO.

Architectural concern: `main.py` is the only real execution coordinator.
Preview validity, motion ownership, state transitions, and protocol handling
should be extracted behind explicit interfaces before more workflows are
added.

### Firmware and protocol structure

The active full-arm path is:

- `controller_firmware/platformio/src/arm_controller.cpp`
  - parses `CONFIG`, `ARM`, `SETPOSE`, `MOVEJ`, `SERVOJ`, `JOGV`, `TRAJ`,
    `HOME`, `STOP`, `ESTOP`, and `TOOL`;
  - performs open-loop stepper pulse generation and servo PWM updates;
  - stores timed trajectory points and interpolates them with clamped cubic
    Hermite segments;
  - supports optional compile-time AS5048A readback;
  - reports estimated joint angles, known-pose fields, tool state, and encoder
    fields.
- `protocol_stub.cpp`
  - no-motor parser used for integration testing.
- `protocol_stub.md`
  - current protocol documentation.

The PC remains the planner:

- previewed joint-space moves and multi-waypoint paths use `TRAJ`;
- `MOVEJ` remains available as a low-level endpoint command;
- live Cartesian jog uses short synchronized `SERVOJ` segments;
- the firmware enforces joint range and arming, but most high-level safety and
  planning remain on the PC.

### Configuration and settings model

Tracked defaults live in `pc_app/config/robot.example.yaml`. Runtime prefers
the ignored `pc_app/config/robot.local.yaml`.

Important sections:

- `geometry` and `kinematics.dh_rows`;
- `joints[]` including limits, home, speed/acceleration, actuator zero/sign,
  and hardware mapping;
- `motion` and `path_defaults`;
- `tools.presets.*.tcp_offset_mm` and tool IO;
- `named_positions`;
- `drop_zones`, `color_profiles`, `task_defaults`, and
  `tasks.color_sorting`;
- `camera.calibration.workspace_aruco`;
- `encoders`;
- `kinematics_calibration`.

The current local configuration is materially different from the example.
Examples include different geometry/sign values, a 42 mm final DH link, an
active magnet TCP Z offset of 158 mm, different joint homes, and physical
actuator zero/direction values. This confirms that tests and diagnostics must
explicitly choose example versus runtime configuration.

### Motion planning paths

| Operator path | Frontend/API | Backend path | Current constraint behavior |
|---|---|---|---|
| Joint Apply | `/api/joints` | `start_joint_target_trajectory()` → `build_joint_trajectory()` → `execute_joint_endpoint_move()` | Planned with global/per-joint limits, but hardware endpoint receives only global speed/acceleration plus firmware-configured per-joint caps |
| Live joint edit | WebSocket or `/api/live-target` | Repeated endpoint trajectory replacement | Uses the same planner but has separate browser pending/draft state |
| Kinematics Execute | `/api/path/preview`, `/api/path/execute` | `build_preview()` then endpoint or waypoint executor | Joint or Cartesian-linear, but preview is not tied to a pose revision |
| Cartesian faders, preview mode | `/api/path/preview` | Same as Kinematics | Edits a frontend IK target |
| Cartesian faders, Cart Jog | `/api/cartesian-jog` | `CartesianServo` → `SERVOJ` | Has dedicated TCP/Phi speed fields and acceleration limits |
| Named-position Move | Program-shaped `/api/path/preview` then execute | `namedPositionWaypoint()` → program trajectory | Existing Preview and Move buttons; no editor |
| Task | `/api/task/preview`, `/api/task/execute` | task sequence → program trajectory | Preview/config freshness checks exist, but no start-pose freshness check |
| Program | `/api/path/preview`, `/api/path/execute` | `build_program_trajectory()` | Manual joint/Cartesian editing already exists; persistence does not |
| Home | `/api/home` | `set_targets()` plus separate `HOME` | Bypasses the normal preview contract and double-dispatches on hardware |
| TCP calibration move | normal path preview/execute | `build_preview()` with calibration enabled/disabled | Uses standard path execution |

### Kinematics and calibration paths

The model chain is currently:

```text
reported logical joint angles
→ Standard DH rows
→ final wrist/flange transform
→ active tool TCP offset
→ FK TCP
```

Cartesian command correction is separate:

```text
requested physical TCP
→ optional tool-specific correction
→ model target sent to IK
→ joint trajectory
```

The correction is documented in `docs/kinematics_calibration.md` and is
correctly described there as a command-layer compensation, not a replacement
for DH, joint-zero, or tool calibration.

Local calibration samples are especially useful evidence:

- intended Z was 70 mm;
- FK from reported joints was approximately 70 mm;
- measured physical Z was approximately 38–43 mm;
- IK/FK target residual was generally below 0.7 mm;
- physical model/landing residual was approximately -27 to -32 mm in Z.

This strongly suggests the persistent Z error is not primarily numerical IK
convergence. Tool length/reference, actuator-to-joint calibration, geometry,
or the physical Z measurement convention must be audited first.

### Vision and task paths

The current vision pipeline is:

```text
camera frame
→ saved workspace homography and workspace polygon
→ color/blob or external detections
→ normalized detection objects
→ task filtering and drop-zone assignment
→ program-shaped pick/place sequence
→ motion preview and execution
```

The app already has:

- a raw annotated camera popup;
- a live projected camera texture on the work plate;
- object markers in robot coordinates;
- task preview markers;
- a setup/detect/plan/run task workflow;
- batch and recapture-per-object execution strategies.

The requested clean semantic workspace view is still missing. It should be a
separate vector/diagram layer, not a replacement for the existing raw camera
projection.

### State synchronisation paths

Backend:

- `RobotState.reported_angles_deg` is updated by simulation or parsed
  `STATUS`;
- `target_angles_deg` is updated by accepted motion commands;
- `fk` is recalculated from reported angles;
- WebSocket broadcasts repeatedly send `state.to_dict()`;
- `align_target_to_reported()` is used on selected stop/connect/set-pose
  paths.

Frontend:

- WebSocket state updates call `renderState()`;
- `renderState()` always sets the solid arm from reported angles;
- joint inputs may continue to show higher-priority local draft, pending,
  preview, or commanded angles;
- IK input follows FK only while `ikUserEdited` is false;
- previews live in independent frontend and backend caches;
- no shared pose/config revision links these objects.

The solid 3D arm is therefore intended to represent reported state, but other
controls and the ghost/path layers can remain based on older intent.

### Tests and verification scripts

Current evidence from this audit:

- `python -m pytest` in `pc_app`: **194 passed**, two FastAPI lifespan
  deprecation warnings.
- The same suite in a clean local Git clone without `robot.local.yaml`:
  **194 passed**, the same two warnings.
- `pio run -e esp32-s3-arm-protocol-stub -e esp32-s3-arm-controller`:
  both targets built successfully.
- JavaScript module syntax checks passed when checked as ES modules.
- The in-app browser was unavailable, so rendered interaction was not claimed.

Test coverage is strong for backend math and APIs but weak for:

- browser state transitions;
- stale-preview rejection;
- first-home behaviour;
- cross-tab synchronisation;
- real controller timing and reported-vs-physical pose;
- hardware calibration and tool/encoder electrical behaviour.

## 3. Documentation Reconciliation

### What `comments.md` adds

`comments.md` adds current operator evidence and changes the priority:

- pose-state disagreement is a safety and architecture issue, not UI polish;
- the physical Z error is measured and repeatable;
- motion settings appear inconsistent across actual operator paths;
- presets and task drop zones are conceptually mixed;
- Program, Tasks, and Kinematics need workflow refinement;
- smart gripper current sensing and a semantic workspace view are desired;
- encoders should begin as diagnostics, not closed-loop control.

These inputs should override the old roadmap order.

### What `docs/remaining_implementation_plan.md` already covers

The older roadmap provides useful subsystem inventory and records many first
passes:

- tool configuration and IO;
- DH/FK/IK foundations;
- analytic IK seeds;
- live Cartesian jog;
- queued trajectories;
- known-pose gates;
- planar workspace calibration;
- detection contracts;
- task sequence builders;
- encoder protocol fields;
- diagnostics and test categories.

It should remain as historical design context, but its priority order and some
status labels are stale.

Examples of stale statements:

- it records 92 tests; the current suite has 194;
- it describes Tasks as placeholder-level, but the current UI has a staged
  setup/detect/plan/run workflow and closed-loop execution state;
- it describes Program as a basic current-pose/IK-target sandbox, while the
  current code already supports editable manual joint and Cartesian steps;
- it says the 3D tool geometry does not reflect TCP, while the current renderer
  does draw a wrist-to-TCP link and marker, though not a proper TCP frame;
- its general Jacobian statement does not distinguish endpoint IK's numerical
  Jacobian from live Cartesian servoing's geometric DH Jacobian.

### What `docs/kinematics_calibration.md` already defines

That document accurately defines the current Cartesian command-correction
layer:

- tool-specific profiles;
- fit and validation samples;
- `constant_xyz` and `affine_xy_z_offset` models;
- residual definitions;
- quality thresholds;
- workspace-signature checks;
- limitations of compensation.

It must not be reinterpreted as proof that DH, actuator zeros, or TCP geometry
are correct. A correction may compensate a repeatable error while hiding its
physical source.

### Contradictions, duplicate plans, and missing links

1. **Roadmap state separation is descriptive, not enforced.** The old roadmap
   names draft/commanded/pending/reported concepts, but execution previews are
   not versioned against reported pose.
2. **Calibration naming is overloaded.** `joints[].zero_offset_deg` and
   `kinematics.dh_rows[].zero_offset_deg` may intentionally represent
   actuator mapping versus model mapping, but the names and UI do not explain
   that distinction.
3. **Motion settings overpromise.**
   - `motion.smoothing_alpha` applies to simulation rate limiting.
   - the Settings "Smoothing" field writes `jerk_percent`;
   - `_trapezoid_ramp_fraction()` prefers `blend_percent` whenever present,
     so `jerk_percent` can be ineffective;
   - `blend_percent` is not true waypoint blending;
   - S-curve progress does not use either percentage.
4. **Home semantics are inconsistent with hardware reality.** Documentation
   says real homing switches are absent, while UI/protocol state calls a
   commanded home move "homed".
5. **Named positions and drop zones leak into each other.**
   `drop_zones()` derives `dropoff_a` and `dropoff_b` from named positions
   before applying explicit drop zones.
6. **Encoder defaults imply more than runtime supports.**
   Example/runtime config can say `settle_correction`, while the normal
   firmware build does not enable AS5048A and the implemented optional mode is
   readback only.
7. **Task mapping rules differ by strategy.** Batch planning only requires
   drop zones for relevant detections, but closed-loop pre-validation can
   reject missing mappings for any enabled color before capture.

### Source-of-truth recommendation

After review:

- this document should be the source of truth for implementation order,
  dependencies, and work-package acceptance criteria;
- `docs/kinematics_calibration.md` should remain the source of truth for the
  current command-correction algorithm until it is revised;
- `pc_app/README.md` should remain the operator/run guide and be updated as
  packages land;
- `docs/remaining_implementation_plan.md` should be retained as historical
  subsystem context, with a short pointer to this plan;
- `comments.md` should remain raw operator input, not an architecture spec.

## 4. Root-Cause Grouping

### A. Pose-state authority and stale intent

Likely shared symptoms:

- first Home does not appear to update the simulated arm correctly;
- Control, Kinematics, faders, presets, Programs, and Tasks disagree;
- previews mix current and previous positions;
- aggressive jumps occur after another workflow changed pose;
- calibration moves appear to leave later actions in a confusing state.

Root causes:

- local browser intent outranks new reported state in control inputs;
- previews lack start-pose revisions;
- backend state has no explicit current/commanded/pending/preview contract;
- movement ownership is spread across several global tasks;
- Home, Stop, Set Pose, config reload, and task completion do not all use one
  state-transition helper.

### B. DH, TCP, calibration, and coordinate truth

Likely shared symptoms:

- physical Z is 20–30 mm low;
- "tool calibration" is unclear;
- TCP versus flange/wrist coordinates are unclear;
- tool visualisation is vague;
- calibration correction may appear to fix one path but not another.

Root causes:

- physical transform chain is not presented as one auditable sequence;
- actuator mapping and kinematic mapping use ambiguous duplicate terminology;
- active tool geometry is large and hardware-unvalidated;
- command correction is separate from the physical model but presented in the
  same Settings area;
- no measured known-pose table ties physical joint/TCP measurements to model
  parameters.

### C. Motion planner and executor inconsistency

Likely shared symptoms:

- acceleration/smoothing changes appear ineffective;
- settings behave differently in Home, joint Apply, Cartesian paths, tasks,
  and live jog;
- linear versus joint-space behaviour is not always obvious;
- estimated duration may not match hardware.

Root causes:

- different command families have different controllers;
- some settings are planner-only, some simulation-only, some firmware-only;
- endpoint hardware commands do not carry per-preview per-joint overrides;
- firmware trajectory timing and actuator lag have not been physically
  validated;
- "smoothing", "jerk", and "blend" labels do not match implemented semantics.

### D. Preset/task abstraction leak

Likely shared symptoms:

- custom preset names cannot be entered;
- new items appear stuck as draft;
- presets feel task-owned;
- color mappings live in Settings;
- workspace boundaries appear to restrict general robot positions.

Root causes:

- general `named_positions` are read-only in the UI;
- editable "presets" are actually task `drop_zones`;
- new color profiles intentionally carry a draft flag;
- default drop zones are partly derived from named positions;
- general positions, task destinations, and color assignments lack separate
  schemas.

### E. Program builder workflow

The complaint is partly stale.

Already present:

- manual joint input;
- manual Cartesian input;
- per-step joint versus linear mode;
- branch selection;
- editing, reordering, disabling, duplication;
- full-program preview with stale-revision protection.

Still missing:

- per-step preview;
- save/load/delete;
- read-only built-in templates and copy-to-user flow;
- persistent program IDs/schema;
- optional tool/wait steps;
- robust demo generation against active geometry and limits.

### F. Camera/workspace visualisation

Likely shared symptoms:

- workspace appears cut off;
- the raw projection is visually noisy;
- simulated detections/tags are not understandable as a workspace model.

Root causes:

- detection mask, calibrated workspace, projected work plate, and semantic
  operator view are not distinct concepts;
- workspace and projection polygons currently default to the same rectangle;
- cropping/padding diagnostics are not shown;
- the existing layer is a camera texture, not a semantic top-down view.

### G. Hardware feedback and tooling

Gripper current and encoders are separate hardware-feedback projects, but both
need:

- sensor-specific calibration;
- explicit data freshness and validity;
- diagnostic-first integration;
- fault thresholds;
- no unbounded automatic correction.

## 5. Revised Priority Order

1. **Pose authority, stale-plan rejection, and Home semantics.**
   Nothing else is safe to trust while tabs can plan from different poses.
2. **Physical model truth: joint convention, DH, TCP, and calibration audit.**
   The measured Z error must be understood before cosmetic Kinematics work or
   task tuning.
3. **One motion contract and honest settings.**
   Program and Tasks should not be expanded while motion sources apply
   different constraints.
4. **General position library and task/drop-zone separation.**
   This removes an abstraction leak used by Tasks, Program, and operator
   shortcuts.
5. **Kinematics pane diagnostics and stress testing.**
   Build it on the corrected pose/model/motion contracts.
6. **Task workflow refinement.**
   Use the separated position/drop-zone model and universal motion planner.
7. **Program persistence, step preview, and demos.**
   Preserve the current builder and add missing workflow pieces after motion
   semantics are stable.
8. **Workspace coverage audit and semantic projected view.**
   Fix calibration coverage before building a richer visual layer.
9. **Servo-gripper current sensing.**
   Add only after motion/state/tool semantics are dependable and sensor
   hardware is selected.
10. **Encoder diagnostics for base and shoulder.**
    First fix known-pose semantics and calibrate raw readback. Bounded
    correction remains later work.

This differs from the old roadmap by putting state and physical truth ahead of
operator-shell polish, tooling expansion, camera polish, and encoders.

## 6. Work Packages

### WP-01 — Authoritative Pose State and Stale-Plan Safety

**Problem**

Current, commanded, pending, draft, and preview poses are not tied together by
an explicit state version. A valid-looking preview can become unsafe after any
other movement.

**Current repo evidence**

- `RobotState` has reported and target angles but no pose revision.
- `build_preview()` starts from `state.reported_angles_deg`.
- `path_previews` store trajectory data but no start-pose version.
- `execute_path()` validates preview ID and Program revision, not current pose.
- `jointControlAngles()` can continue preferring frontend draft/preview state.
- task previews check TTL and config ID, but not start pose.

**Files/code areas**

- `pc_app/app/robot_state.py`
- `pc_app/app/main.py`
  - `build_preview()`
  - `execute_path()`
  - `start_joint_target_trajectory()`
  - `apply_controller_status()`
  - `align_target_to_reported()`
  - Home/Stop/Set Pose/connect/disconnect routes
- `pc_app/app/static/app.js`
  - state object
  - `jointControlAngles()`
  - `renderState()`
  - preview and execute functions
- `pc_app/tests/test_joint_space_execution.py`
- `pc_app/tests/test_program_builder_api.py`
- `pc_app/tests/test_task_execution_loop.py`
- new state-transition and stale-preview tests

**Proposed implementation approach**

1. Add a backend pose snapshot contract:
   - `pose_revision`;
   - `reported_angles_deg`;
   - `reported_at`;
   - `pose_source`;
   - `pose_known_mask` or per-joint authority;
   - `commanded_target_deg`;
   - `pending_motion` with run ID, source, mode, start pose, target, and status.
2. Increment `pose_revision` on authoritative pose changes:
   - accepted controller status that changes reported pose;
   - Set Pose;
   - simulation movement;
   - completed Home/Stop realignment;
   - encoder-authoritative updates.
3. Store in every preview:
   - `start_pose_revision`;
   - `start_reported_angles_deg`;
   - config/model fingerprint;
   - requested/commanded target;
   - source.
4. At execute time:
   - reject if model/config changed;
   - reject if pose revision changed and joint delta exceeds a small declared
     tolerance;
   - require a new preview rather than silently splicing a new first segment.
5. Treat frontend draft and ghost state as local intent only:
   - attach a `base_pose_revision`;
   - clear or explicitly rebase when authoritative pose changes;
   - never use frontend-only state as the start pose for execution.
6. Expose a compact state diagnostic in the UI:
   reported, commanded, pending, draft, preview, source, and revisions.
7. Make motion ownership explicit so only one active executor can own the
   commanded pose.

**What not to do**

- Do not move every UI draft into backend `RobotState`.
- Do not silently replan a reviewed preview at execute time.
- Do not claim that open-loop reported pose is physically measured.
- Do not add encoder correction as part of this package.

**Acceptance criteria**

- Moving from any tab updates the solid arm and all non-edited current-pose
  readouts within one WebSocket refresh cycle.
- A preview created at pose A cannot execute after the robot moves to pose B.
- The rejection message identifies the stale start pose and asks for preview
  again.
- Home, Stop, Set Pose, Program, Task, preset, and Cartesian commands all
  publish the same pose snapshot fields.
- Frontend drafts cannot override the solid-arm reported pose.
- Unknown pose blocks motion and is represented per joint where appropriate.

**Tests/manual verification**

- Unit-test pose revision increments and non-increments.
- API-test stale joint, linear, Program, and Task preview rejection.
- Test a preview created in Kinematics, then move in Control, then execute.
- Test repeated preview/execute/reset cycles.
- Browser test cross-tab values and ghost clearing.
- Hardware-gated test with delayed `STATUS` responses.

**Dependencies:** none  
**Risk:** critical

### WP-02 — Home, Set-Pose, Config-Sync, and Known-Pose Semantics

**Problem**

Home currently has two command paths and overstates what is known. Calibration
or hardware-config changes can invalidate the controller's interpretation of
the current pose.

**Current repo evidence**

- `/api/home` calls `set_targets(config.home_pose, "home")`, then sends
  `HOME`.
- firmware `handleHome()` sets a target and immediately sets `homed=true`.
- documentation says real homing switches are not included.
- `CONFIG END` replaces actuator configuration and calls
  `syncRuntimeFromCurrentPose()`.
- backend config reload marks hardware sync stale, then attempts sync.
- `config_sync_ready()` does not require disarmed hardware.

**Files/code areas**

- `pc_app/app/main.py`
  - `home()`
  - `hardware_setpose()`
  - `save_calibration()`
  - `reload_runtime_config()`
  - `sync_hardware_config()`
- `pc_app/app/protocol.py`
- `controller_firmware/platformio/src/arm_controller.cpp`
  - `handleHome()`
  - `handleSetPose()`
  - `handleConfig()`
  - status fields
- protocol documentation and hardware-sync tests

**Proposed implementation approach**

1. Define three distinct operations:
   - **Go to Home Pose:** normal planned move from a known pose.
   - **Set Pose:** operator assertion while disarmed, with explicit warning.
   - **Physical Home:** future switch/index procedure that can establish pose.
2. Until physical homing exists:
   - label the UI action `Go Home`;
   - do not allow it to establish an unknown pose;
   - do not set `homed` merely because the command was accepted.
3. Send one hardware command per operation:
   - route Go Home through normal planned trajectory execution;
   - or make `HOME` the sole controller command with the same diagnostics;
   - do not do both.
4. Mark completion only after controller idle and target tolerance.
5. Classify config changes:
   - camera/task/display changes: no hardware pose impact;
   - geometry/TCP model changes: invalidate previews/model fingerprint;
   - actuator zero/sign/gear/home changes: require disarm, sync, and explicit
     known-pose revalidation;
   - IO-only changes: require idle/disarm where output remapping can occur.
6. Log the exact reason config sync became stale and whether pose knowledge
   was invalidated.

**What not to do**

- Do not add switchless "automatic homing".
- Do not let a config save silently redefine the physical pose.
- Do not auto-SETPOSE after calibration changes.

**Acceptance criteria**

- One Home click causes one controller movement command.
- The simulated arm follows the first Home/Go Home action.
- Home completion updates reported pose, FK, controls, and previews together.
- A calibration-position move followed by Go Home does not require unrelated
  resync.
- If actuator mapping changed, the UI clearly requires disarm, sync, and pose
  revalidation.
- `homed` means physically established home only; otherwise the state says
  `known` with an accurate source such as `setpose` or `open_loop_estimate`.

**Tests/manual verification**

- Add dedicated Home API tests; none currently exist.
- Assert the serial command sequence contains only one movement command.
- Test Home after TCP calibration preview/execute.
- Test Home after model-only save versus actuator-mapping save.
- Protocol-stub tests for accepted, moving, completed, stopped, and faulted
  Home states.

**Dependencies:** WP-01  
**Risk:** critical

### WP-03 — DH, Joint Convention, TCP, and Physical Calibration Truth

**Problem**

The physical robot lands about 20–30 mm low in Z. The current settings expose
geometry, DH rows, joint calibration, tool offsets, and command correction
without one clear physical transform chain.

**Current repo evidence**

- local samples show approximately -27 to -32 mm physical Z residual while IK
  target residual is below 0.7 mm;
- local active magnet TCP Z is 158 mm;
- local final DH `a4`/wrist length is 42 mm;
- local actuator zero/sign values differ from DH row zero/sign values;
- command correction is disabled and no fitted result is stored;
- FK and Three.js independently implement the same model.

**Files/code areas**

- `pc_app/config/robot.example.yaml`
- ignored `pc_app/config/robot.local.yaml`
- `pc_app/app/config.py`
- `pc_app/app/kinematics.py`
- `pc_app/app/cartesian_calibration.py`
- `pc_app/app/static/robot_view.js`
- `pc_app/app/static/app.js` Settings calibration UI
- `docs/kinematics_calibration.md`
- kinematics and calibration tests

**Proposed implementation approach**

1. Publish one named transform chain:

   ```text
   actuator sensor/steps
   → logical joint coordinate q
   → DH model joint theta
   → wrist/flange frame
   → active tool TCP frame
   → optional Cartesian command correction
   ```

2. Rename or clearly label duplicated settings:
   - actuator zero/sign/gear mapping;
   - model/DH theta offset and sign;
   - mechanical home/reference angle.
3. Define frames:
   - robot base;
   - DH frame 4 / wrist or flange;
   - tool frame;
   - TCP;
   - workspace plane;
   - camera/image frame.
4. Audit Z in this order:
   - measurement reference and workspace Z=0;
   - tool-tip/contact-point definition;
   - physical TCP length and direction;
   - joint zero poses and direction;
   - base/shoulder axis heights and final link;
   - DH signs/offsets;
   - repeatability/backlash;
   - only then Cartesian command correction.
5. Create a known-pose calibration table with measured:
   - joint reference condition;
   - wrist/flange XYZ;
   - TCP XYZ;
   - tool and measurement method;
   - expected FK;
   - residual.
6. Treat the existing command correction as optional compensation after the
   model audit, not the first fix.
7. Add a Cartesian TCP frame in the viewport:
   - red/green/blue axes;
   - clear forward direction;
   - labels for flange and TCP;
   - toggleable calibration mode.
8. Keep backend and renderer parity tests for transforms and TCP points.

**What not to do**

- Do not tune DH lengths to compensate a wrong tool-tip reference.
- Do not fit an affine correction before checking the 158 mm active tool
  offset and physical reference.
- Do not expose two editable sources for the same parameter.
- Do not call the command-correction workflow "tool calibration" without
  qualification.

**Acceptance criteria**

- The operator can identify whether every XYZ value is flange, wrist, or TCP.
- The transform chain and units are visible in docs and diagnostics.
- At least five measured poses span left/right, near/far, and high/low Z.
- Initial target: physical TCP error ≤5 mm in Z and a separately declared XY
  tolerance, assuming the measurement method supports it.
- Changing active tool TCP changes FK, IK, preview, execution, and TCP-frame
  visualisation consistently.
- Backend FK and Three.js TCP agree for regression poses.

**Tests/manual verification**

- Golden-pose FK tests using `EXAMPLE_CONFIG_PATH`.
- Local calibration report script that does not commit local values.
- Backend/renderer transform parity fixtures.
- Hardware measurement sheet for repeatability and direction-dependent error.
- Validation with command correction disabled, then enabled only after model
  acceptance.

**Dependencies:** WP-01 and WP-02  
**Risk:** high

### WP-04 — Unified Motion Contract and Honest Motion Settings

**Problem**

Motion sources use related but non-identical planning and execution paths.
Several Settings labels do not correspond to implemented behaviour.

**Current repo evidence**

- joint, linear, and program trajectories use common builders;
- Home bypasses them;
- live Cartesian jog uses a separate servo controller;
- `motion.smoothing_alpha` is simulation-only rate limiting;
- UI "Smoothing" writes `jerk_percent`;
- `blend_percent` is used as a trapezoid ramp fraction, not waypoint blending;
- S-curve progress ignores both values;
- endpoint hardware commands send global speed/acceleration, while trajectory
  timestamps carry the detailed per-joint planning result;
- first-class TCP acceleration exists in config/backend but not normal Settings
  UI.

**Files/code areas**

- `pc_app/app/motion.py`
- `pc_app/app/cartesian_servo.py`
- `pc_app/app/main.py`
- `pc_app/app/static/app.js`
- `pc_app/app/static/index.html`
- `controller_firmware/platformio/src/arm_controller.cpp`
- motion, live-motion, task, and joint-execution tests

**Proposed implementation approach**

1. Introduce a common motion request/plan/result schema:
   - source;
   - start pose and revision;
   - target type;
   - joint or linear Cartesian mode;
   - limits used;
   - estimated duration;
   - limiting joint/constraint;
   - generated trajectory;
   - execution/progress result.
2. Route Joint Apply, Kinematics, presets, Tasks, Program, calibration moves,
   and Go Home through that contract.
3. Keep live Cartesian servoing as a distinct controller, but report the same
   limit/result fields.
4. Replace misleading settings:
   - per-joint maximum speed and acceleration;
   - default joint speed and acceleration caps;
   - TCP speed and TCP acceleration;
   - Phi speed and Phi acceleration;
   - Cartesian path spatial resolution;
   - controller/preview update rate;
   - actual supported profile type.
5. Hide or remove jerk/blend controls until they have defined, tested
   semantics.
6. Add planner output for:
   - effective per-joint limits;
   - limiting joint;
   - maximum predicted speed/acceleration;
   - path mode;
   - duration.
7. Validate firmware behaviour:
   - stepper acceleration for endpoint and queued paths;
   - servo tracking of moving trajectory targets;
   - queue timing and lag;
   - progress reporting.

**What not to do**

- Do not add more profile names without controller support.
- Do not call a sampled joint path "Cartesian linear" unless TCP linearity is
  checked.
- Do not expose simulation smoothing as a universal hardware setting.
- Do not increase hardware aggressiveness to make timing match estimates.

**Acceptance criteria**

- Every movement path reports mode, limits used, duration, and limiting
  constraint.
- Lowering a per-joint limit changes both estimate and physical/simulated
  execution.
- Linear mode stays within a declared Cartesian line-error tolerance.
- Joint mode is clearly labelled as joint interpolation with only an estimated
  TCP trace.
- Unsupported settings are absent or marked preview-only.
- Go Home uses a conservative declared profile.

**Tests/manual verification**

- Parameterised tests across all motion entry points.
- Same target from Control, Kinematics, preset, Program, and Task produces
  consistent effective limits.
- Linear-path deviation tests.
- Firmware queue timing tests with stub and hardware timestamps.
- Hardware slow-speed acceptance tests before nominal speed.

**Dependencies:** WP-01 through WP-03  
**Risk:** high

### WP-05 — General Position Library and Task-Destination Separation

**Problem**

General named positions exist but are not editable in the UI. Editable task
drop zones are labelled as presets, and color mappings are stored in Settings.

**Current repo evidence**

- `/api/named-positions` supports arbitrary keys and validation.
- Control already shows Preview and Move for named positions.
- there is no named-position create/rename/delete UI.
- Settings "Task presets" edits `drop_zones` and `color_profiles`.
- new drop zones get generated names such as `drop_zone_n`.
- newly detected color profiles carry `draft: true`.
- `drop_zones()` derives default drop zones from named `dropoff_a/b`.

**Files/code areas**

- `pc_app/app/demo_settings.py`
- `pc_app/app/tasks.py`
- `pc_app/app/main.py` named-position APIs
- `pc_app/app/static/app.js`
- `pc_app/app/static/index.html`
- config migration and demo-feature tests

**Proposed implementation approach**

1. Define a general position record:
   - stable ID;
   - custom display name;
   - optional description/tags;
   - joint or Cartesian/TCP type;
   - values and units;
   - optional preferred motion mode and tool;
   - created/updated metadata.
2. Add a Position Library UI with:
   - save current reported pose;
   - manual joint/Cartesian creation;
   - rename, duplicate, delete;
   - Preview and Go To.
3. Validate general positions only against robot model/safety, not camera
   workspace.
4. Define task destinations separately:
   - inline Cartesian drop target; or
   - reference to a general position ID;
   - grid/placement policy.
5. Move color/object-to-destination assignment into Tasks.
6. Remove implicit derivation of task drop zones from named positions after a
   compatibility migration.
7. Reserve "draft" for unsaved task/color configuration and explain it.

**What not to do**

- Do not use mutable display names as long-term references.
- Do not reject a general position for being outside the camera polygon.
- Do not keep global color-to-drop mappings in general robot Settings.
- Do not delete existing config without migration.

**Acceptance criteria**

- Any custom name can be created and persisted.
- A position can be previewed and executed with Go To.
- General positions outside the camera workspace remain valid if kinematically
  and mechanically safe.
- Task destinations can reference positions without owning them.
- No new general position is labelled draft after a successful save.
- Existing `home`, `safe`, `pickup_test`, and dropoff entries migrate safely.

**Tests/manual verification**

- CRUD API and config migration tests.
- Duplicate-name and stable-ID tests.
- outside-workspace but reachable position test.
- unreachable and joint-limit validation tests.
- browser test rename/preview/go-to/delete.

**Dependencies:** WP-01 and WP-04  
**Risk:** medium

**Implementation status - June 20, 2026**

Code complete in the current working tree:

- `position_library` is the primary committed schema, with stable IDs,
  editable display names, joint/Cartesian records, metadata, validation, and
  compatibility mirroring to legacy `named_positions`.
- Control includes create, save-current, edit, rename, duplicate, delete,
  Preview, and Go To actions for Position Library records.
- `task_destinations` remains a compatibility schema for old inline anchors,
  while the primary Tasks UI maps colors directly to Position Library IDs.
- color-to-destination assignment is edited and saved from Tasks, not from
  general robot Settings.
- legacy `named_positions`, `drop_zones`, and their APIs remain supported
  during migration.
- task destinations no longer derive implicitly from named dropoff positions.
- `draft` is treated as unsaved task/color UI state and is removed on save.
- automated migration, stable-ID, reference-integrity, persistence, safety,
  and task-planning tests are present.

Follow-up operator sweep on June 20, 2026:

- only `home` is protected as a core Position Library record;
- Position Library and Physical Model Truth panels are collapsible;
- task color mappings select Position Library records directly, while
  `task_destinations` remains a compatibility bridge in persisted config;
- grid/row/column destination editing is no longer exposed in the primary UI;
- Position Library validation errors are shown on the exact invalid record;
- Position Library Go To uses one atomic backend plan-and-start request to
  avoid a stale gap between separate preview and execute requests;
- Go Home uses the same selected speed, acceleration, and per-joint limits as
  other normal moves;
- Set Pose uses an in-app confirmation panel;
- tool changes expose a visible pending state;
- Diagnostics uses lightweight live state rendering so opening it does not
  stall the robot viewport.

Remaining verification:

- the private five-pose physical calibration measurement sweep is deferred;
- fresh rendered browser automation still needs a working browser-control
  session. Manual desktop feedback was supplied by the operator.

### WP-06 — Kinematics Pane as Cartesian Control and Diagnostic Surface

**Problem**

The Kinematics pane has useful first-pass controls but does not present the
complete state/model/limit diagnosis needed to trust Cartesian motion.

**Current repo evidence**

Already present:

- x/y/z/phi input;
- auto phi;
- joint versus linear mode;
- branch selection;
- candidate angles, position error, phi error, and rejection reasons;
- preview duration and waypoint count;
- preview/execute/stop.

Missing or unclear:

- current FK within the pane;
- requested physical TCP versus corrected model target;
- preview start pose/revision;
- joint-limit margins and limiting joint;
- predicted max speed/acceleration;
- explicit flange/TCP frame;
- clear reachability categories;
- interaction state between faders, presets, and edited targets.

**Files/code areas**

- `pc_app/app/static/index.html`
- `pc_app/app/static/app.js`
- `pc_app/app/kinematics.py`
- `pc_app/app/motion.py`
- `pc_app/app/main.py`
- `pc_app/tests/test_kinematics.py`
- `pc_app/tests/test_motion.py`

**Proposed implementation approach**

1. Present five separate values:
   - current reported TCP;
   - operator draft target;
   - corrected model/IK target;
   - selected IK solution;
   - executable preview endpoint.
2. Add diagnostics:
   - reachability category;
   - selected branch and continuity cost;
   - position/Phi error;
   - joint-limit margin per joint;
   - singularity/conditioning warning;
   - duration and limiting joint;
   - path line-error estimate;
   - calibration profile/status.
3. Make target ownership explicit:
   - following current FK;
   - user-edited;
   - loaded from position;
   - loaded from Program/Task preview.
4. Add reset-to-current and copy-current actions.
5. Keep preview and execute separate; stale preview must disable Execute.
6. Use structured backend error codes instead of only text parsing.

**What not to do**

- Do not make the pane silently execute while dragging.
- Do not hide branch changes or automatic Phi selection.
- Do not add visual polish before pose/model diagnostics are correct.

**Acceptance criteria**

- The pane explains why a target failed.
- Requested TCP, model command target, and FK result cannot be confused.
- The limiting joint and minimum joint-limit margin are visible.
- A fader or preset change makes the old preview stale.
- Linear and joint modes show different, accurate path descriptions.

**Stress-test matrix**

- near maximum reach and just outside reach;
- low Z, high Z, and workspace-plane contact;
- same XYZ with multiple Phi values;
- auto/fixed Phi transitions;
- elbow-up/down/auto branch changes;
- repeated preview/execute/reset;
- fader edits after preset preview;
- preset preview after fader edits;
- linear path crossing a near-singular region;
- local and example configuration.

**Dependencies:** WP-01, WP-03, WP-04  
**Risk:** medium-high

### WP-07 — Task Workflow, Drop Zones, and Recovery

**Problem**

The current task workflow is more advanced than the old roadmap states, but
task configuration, global color profiles, drop zones, and position presets
remain mixed.

**Current repo evidence**

- staged setup/detect/plan/run UI exists;
- detection inspection and preview exist;
- task settings expose pickup/drop Z, clearances, Phi policy, and movement
  modes;
- batch planning only blocks missing zones for relevant detections;
- closed-loop planning can pre-block missing zones for any enabled color;
- task abort tracks `holding_uncertain`, but recovery options are not defined;
- tool feedback is state-only, not physical grip confirmation.

**Files/code areas**

- `pc_app/app/tasks.py`
- `pc_app/app/main.py` task routes and execution loops
- `pc_app/app/static/app.js`
- `pc_app/app/static/index.html`
- `pc_app/tests/test_color_sorting_tasks.py`
- `pc_app/tests/test_task_execution_loop.py`

**Proposed implementation approach**

1. Use the operator flow:

   ```text
   capture detections
   → inspect/select
   → assign required detected classes to destinations
   → preview generated sequence
   → confirm
   → execute
   ```

2. Bind task preview to:
   - detection snapshot ID/timestamp;
   - pose revision;
   - config/model fingerprint;
   - task settings revision.
3. Require a destination only for a selected/relevant detected object.
   Closed-loop mode should validate each fresh capture, not every possible
   enabled color before capture.
4. Keep explicit:
   - pickup Z/contact height;
   - approach and retreat clearances;
   - release height;
   - fixed/auto/preferred Phi;
   - joint versus linear mode for each phase;
   - active tool requirement.
5. Show the complete generated step list before batch execution.
6. Define abort/recovery states:
   - no object held;
   - object possibly held;
   - grip confirmed later by sensor;
   - safe retreat available/unavailable.
7. Keep color threshold editing near detection, but task destination mapping
   in the task workflow.

**What not to do**

- Do not run a task from unsaved draft mappings.
- Do not block on colors absent from the current selected detections.
- Do not assume the tool released or holds an object without feedback.
- Do not expand full autonomy before state/motion contracts are stable.

**Acceptance criteria**

- A detected selected color without a destination blocks preview/execute.
- An undetected color without a destination does not block.
- Every generated move identifies target frame, mode, and height.
- Task preview becomes stale on pose, detection, task-setting, model, or
  destination changes.
- Stop reports current step, last completed step, and hold uncertainty.

**Tests/manual verification**

- Batch and closed-loop detected-only mapping tests.
- stale detection/pose/config/task-preview tests.
- abort during approach, grip, transfer, release, and retreat.
- simulated detection queue tests.
- hardware-gated dry run with tool disabled.

**Dependencies:** WP-01, WP-04, WP-05  
**Risk:** high

### WP-08 — Program Persistence, Step Preview, and Demo Templates

**Problem**

Program is already a capable in-memory motion builder, but lacks persistence,
per-step preview, and reusable templates.

**Current repo evidence**

Already present:

- manual joint and Cartesian steps;
- editable values;
- joint/linear mode;
- branch choice;
- reorder, duplicate, disable, delete;
- full-program preview;
- Program revision check at execute time.

Missing:

- save/load/delete;
- program metadata and stable IDs;
- per-step preview;
- built-in demo templates;
- copy template to editable user program;
- optional non-motion steps.

**Files/code areas**

- Program sections in `index.html` and `app.js`
- `build_program_trajectory()` in `motion.py`
- path preview/execute routes in `main.py`
- `test_program_builder_api.py`
- new program storage module and tests

**Proposed implementation approach**

1. Preserve the existing builder; do not rewrite it.
2. Define a versioned program schema:
   - stable ID, name, description, schema version;
   - read-only/template flag;
   - steps;
   - optional required tool;
   - timestamps.
3. Store tracked built-ins separately from ignored user programs, for example:
   - tracked read-only template directory;
   - ignored local user-program file/directory.
4. Add CRUD APIs and Program Library UI.
5. Add per-step preview using the planned end pose of preceding enabled steps,
   not blindly the current physical pose.
6. Add parameterised, read-only templates:
   - small safe square;
   - circle/polyline;
   - conservative show-off joint routine.
7. Require Copy before editing a built-in template.
8. Generate Cartesian demos relative to a declared centre/start pose and run
   the same reachability/motion preview as user programs.
9. Consider tool/wait/comment steps only after the motion-only persistence
   format is stable.

**What not to do**

- Do not hardcode demo coordinates for one geometry.
- Do not execute a template before preview.
- Do not add a database for the current single-user local application.
- Do not claim a circle is smooth until queued Cartesian tracking is validated.

**Acceptance criteria**

- A program can be built without leaving Program.
- Each step can be previewed in sequence context.
- User programs persist across restart and can be renamed/deleted.
- Built-ins are read-only until copied.
- Demos adapt or fail safely against active geometry and limits.
- Program execution uses pose-revision and universal motion gates.

**Tests/manual verification**

- schema migration and CRUD tests;
- built-in immutability tests;
- step-preview start-context tests;
- stale Program revision and stale pose tests;
- square/circle line-error tests in simulation;
- browser persistence workflow test.

**Dependencies:** WP-01, WP-04, WP-05, WP-06  
**Risk:** medium

### WP-09 — Workspace Coverage Audit and Semantic Projection View

**Problem**

The existing projected camera texture may appear cropped, and the requested
clean workspace view is not the same feature as the raw projection.

**Current repo evidence**

- `workspace_polygon_robot_mm` and `projection_polygon_robot_mm` exist;
- current example values are identical;
- detection masking uses the calibrated workspace polygon;
- camera texture generation uses the projection/workspace polygon and robot
  bounds;
- Three.js renders the projected texture and outline;
- object/task markers exist;
- simulation can queue detections but has no dedicated semantic tag/workspace
  view.

**Files/code areas**

- `pc_app/app/vision.py`
- `pc_app/app/workspace_calibration.py`
- `pc_app/app/static/robot_view.js`
- camera/task frontend code
- camera config sections
- workspace/vision tests

**Proposed implementation approach**

1. First audit and visualise:
   - detected tag corners;
   - saved image polygon;
   - robot workspace polygon;
   - projection polygon;
   - output texture bounds;
   - padding and clipped pixels.
2. Separate concepts:
   - detection-valid workspace;
   - physical work-plate projection area;
   - robot reachable workspace;
   - task destination area.
3. Add calibration diagnostics showing coverage percentage and edge margins.
4. Add a semantic top-down workspace view with vector layers:
   - workspace boundary;
   - detections;
   - drop destinations;
   - general positions;
   - planned path;
   - robot base/reference axes;
   - simulated AprilTag/ArUco markers.
5. Keep the raw camera popup and projected texture as optional diagnostic
   layers.
6. Do not clip general positions merely because they are outside the camera
   polygon; show an off-workspace indicator if useful.

**What not to do**

- Do not compensate cropping by arbitrary padding before measuring the
  polygon.
- Do not mix robot reachability with camera visibility.
- Do not require full 6-DoF camera pose for the planar semantic view.

**Acceptance criteria**

- Calibration view explains exactly which polygon cuts off an image region.
- The full intended work plate is represented or the excluded area is
  deliberately documented.
- Semantic markers align with the raw projected image within calibration
  tolerance.
- Simulation shows deterministic fake detections and tag markers.
- Turning semantic/raw layers on or off never affects task coordinates.

**Tests/manual verification**

- synthetic homography edge/corner tests;
- polygon padding and non-rectangular projection tests;
- image-size and flip-X regression tests;
- browser layer-toggle tests;
- physical marker-at-corners verification.

**Dependencies:** WP-03 and WP-05; workspace calibration audit before semantic polish  
**Risk:** medium

### WP-10 — Servo-Gripper Current Sensing and Smart Grip

**Problem**

The current gripper is position-controlled PWM only. There is no current
sensor, telemetry, grip state machine, or stall protection.

**Current repo evidence**

- tool config contains PWM and pulse mapping only;
- protocol supports `TOOL OPEN/CLOSE/SET`;
- firmware drives the output and reports logical tool state/value;
- Tasks issue open/close actions and fixed delays;
- no current/stall/backoff fields or ADC/current-sensor code exist.

**Files/code areas**

- hardware/electronics design and selected sensor documentation
- `robot.example.yaml` tool schema
- `protocol.py` and `protocol_stub.md`
- `arm_controller.cpp` and `protocol_stub.cpp`
- backend `RobotState`, diagnostics, and tool routes
- Tasks tool-step execution
- frontend tool and diagnostics panels

**Proposed implementation approach**

1. Select and document sensor hardware first:
   - sensor type;
   - measurement range;
   - ADC/I2C interface;
   - sample rate;
   - electrical isolation/grounding;
   - calibration to amperes.
2. Implement the safety state machine on the controller:
   - close;
   - detect grip threshold for a minimum duration;
   - detect hard stall threshold;
   - stop PWM progression;
   - optional bounded backoff;
   - optional bounded hold command;
   - timeout and fault;
   - safe release/stop behaviour.
3. Add config:
   - grip threshold;
   - stall threshold;
   - debounce/filter window;
   - timeout;
   - backoff amount/time;
   - hold policy and maximum hold current/time.
4. Add telemetry:
   - raw ADC;
   - calibrated current;
   - filtered current;
   - threshold state;
   - grip result/fault;
   - timestamp.
5. Use a dedicated smart-grip command/result rather than making the PC poll
   fast enough to prevent a stall.
6. Integrate Tasks only when the active tool is a validated servo gripper.
7. Add a bounded live graph/debug export in the diagnostics UI.

**What not to do**

- Do not leave the servo commanded against a hard stop indefinitely.
- Do not implement stall protection only in browser JavaScript.
- Do not guess thresholds before measuring open/close/object current.
- Do not enable smart grip for unrelated tool types.

**Acceptance criteria**

- No grip command can hold stall current past the configured timeout.
- Grip, empty close, and hard stall are distinguishable.
- Backoff/hold behaviour is bounded and logged.
- Live current is visible with units and freshness.
- Tasks record grip success/failure and stop safely on failure.

**Tests/manual verification**

- protocol-stub state-machine tests;
- bench test without robot motion;
- open/close current baseline;
- soft object, hard object, no object, jam, sensor disconnect;
- STOP/ESTOP during every grip phase;
- long-duration hold thermal test within hardware limits.

**Dependencies:** WP-01, WP-02, WP-04, validated tool IO  
**Risk:** high, hardware-dependent

### WP-11 — Encoder Diagnostics Before Correction

**Problem**

Encoder plumbing exists, but current known-pose and calibration semantics are
not safe enough for correction.

**Current repo evidence**

- firmware has optional compile-time base/shoulder AS5048A reads;
- normal controller build does not define
  `ARM_CONTROLLER_ENABLE_AS5048A`;
- CS pins are build-time constants;
- PC config encoder zero/direction is not sent to firmware;
- raw encoder angles can replace backend reported angles;
- one available encoder can make the whole pose known;
- config defaults can say `settle_correction` although implemented firmware
  mode is `off` or `readback`;
- backend faults on large commanded-versus-encoder error.

**Files/code areas**

- encoder config/defaults in `demo_settings.py` and YAML
- `arm_controller.cpp`
- `protocol.py`
- backend encoder verification and diagnostics
- Settings encoder UI
- protocol, known-pose, and hardware-sync tests

**Proposed implementation approach**

1. Change the operational first stage to diagnostics/readback only.
2. Make encoder enablement and pins config-driven or create a clearly
   documented encoder-specific firmware build.
3. Report:
   - raw sensor angle;
   - calibrated joint angle;
   - validity/freshness;
   - commanded versus measured error.
4. Apply encoder zero, sign, wrap, and gear-side convention explicitly.
5. Use a per-joint known mask:
   - base/shoulder feedback cannot make elbow/wrist known;
   - whole-arm known requires every joint to have a valid authority source.
6. Add known-pose verification at several repeatable fixtures.
7. Fault on large mismatch; never jump the commanded pose to the measurement.
8. Only after successful diagnostics consider bounded settle correction:
   - disallow large corrections;
   - limited attempts;
   - low speed;
   - explicit fault on non-convergence.

**What not to do**

- Do not jump to continuous closed-loop stepper control.
- Do not treat raw 0–360 sensor angles as calibrated joint angles.
- Do not allow one encoder to validate all four joints.
- Do not enable tracked `settle_correction` defaults before hardware proof.

**Acceptance criteria**

- Base and shoulder show raw and calibrated values separately.
- Known-pose state is correct per joint.
- Large mismatch faults without corrective motion.
- Sensor disconnect/stale data removes encoder authority.
- Known-pose verification is repeatable within a measured tolerance.

**Tests/manual verification**

- wraparound, sign, zero, stale, disconnect, and partial-availability tests;
- fixture-pose measurements at multiple angles;
- power-cycle known-pose tests;
- commanded-versus-measured trend logging;
- bounded-correction tests only in a later package.

**Dependencies:** WP-01 through WP-04 and physical encoder calibration  
**Risk:** high

## 7. Suggested First Implementation Batch

The first batch should be intentionally small and should not change kinematic
math, motion aggressiveness, camera behaviour, Tasks, or Program persistence.

### Batch scope

1. Add backend pose/state sequence fields and one state-transition helper.
2. Capture preview start pose, pose revision, and config/model fingerprint.
3. Reject stale path, Program, and Task execution.
4. Add diagnostics that show:
   - reported pose;
   - commanded target;
   - active motion/run ID;
   - preview start revision;
   - current revision;
   - rejection reason.
5. Rework `/api/home` so one operator action produces one motion command.
6. Change `homed` semantics or UI wording so switchless Go Home does not claim
   to establish an unknown pose.
7. Clear/rebase frontend joint and IK intent after Home, Stop, Set Pose, and
   authoritative pose changes.
8. Add regression tests for the observed first-home and stale-jump scenarios.

### Exact starting points

Start in:

- `pc_app/app/robot_state.py`;
- `pc_app/app/main.py` around `build_preview()`, `execute_path()`,
  `apply_controller_status()`, and `home()`;
- `pc_app/app/static/app.js` around `jointControlAngles()`, `renderState()`,
  `renderPreview()`, and the Home/Stop handlers;
- relevant API/execution tests.

### Batch exit criteria

- one Home press updates the reported arm and all current-pose readouts;
- a preview cannot execute after another movement;
- no frontend-only pose is accepted as an implicit motion start;
- existing 194 tests remain green in both local and clean-clone configurations;
- new state/home tests pass;
- browser interaction tests cover Control ↔ Kinematics ↔ preset transitions.

## 8. Deferred Items

Defer until their dependencies are complete:

- smart gripper task automation before state, motion, and basic tool IO are
  stable;
- encoder settle correction before readback, per-joint authority, and physical
  calibration are reliable;
- full task autonomy before general positions and task destinations are
  separated;
- advanced semantic camera projection before workspace polygon coverage is
  understood;
- polished Kinematics visual design before DH/TCP truth is validated;
- built-in circle/show-off execution before linear-path and queue timing are
  physically verified;
- true waypoint blending before firmware can report and enforce its timing;
- closed-loop Cartesian control from PC camera/encoders;
- switch-based physical homing until hardware, direction, and safe speed are
  defined.

## 9. Verification Plan

### Unit tests

- pose revision and state-transition rules;
- preview start-pose capture and stale detection;
- model/config fingerprint invalidation;
- transform-chain and TCP frame tests;
- joint/model zero-sign convention tests;
- effective motion-limit calculation;
- limiting-joint and duration diagnostics;
- position/drop-zone/program schema validation;
- encoder raw-to-calibrated conversion;
- gripper current state machine with simulated samples.

### Backend API tests

- Home/Go Home command sequence;
- Set Pose disarmed/known-pose behaviour;
- path/Program/Task stale-preview rejection;
- named-position CRUD and migration;
- Program CRUD and template immutability;
- task detected-only destination validation;
- motion settings consistency across entry points;
- encoder partial-known masks;
- smart-grip command/result/fault APIs.

### Frontend/manual UI tests

- first Home press;
- Control draft then tab switch;
- Kinematics preview then Control move;
- preset preview/Go To;
- fader edit then preset load;
- Program step edit/preview/save/reload;
- Task detect/assign/preview/execute;
- raw versus semantic camera layers;
- unknown/stale pose display;
- diagnostics for calibration, motion, encoder, and gripper.

Automated browser coverage should be added for these flows. The current audit
could not run the in-app browser, so rendered behaviour remains unverified.

### Simulation tests

- all motion sources from the same start/target;
- stale preview after simulated movement;
- joint and linear mode path differences;
- near-limit/unreachable IK matrix;
- fake detections and fake tag overlays;
- Task stop/recovery states;
- demo square/circle preview.

### Hardware-gated tests

Run in increasing authority:

1. protocol stub;
2. controller with all axes simulated;
3. one physical axis at conservative limits;
4. mixed physical/simulated axes;
5. full arm without tool;
6. tool IO bench test;
7. gripper current sensor;
8. encoder readback diagnostics.

Every stage requires:

- known pose;
- disarmed config changes;
- low speed/acceleration;
- clear stop access;
- physical workspace clear;
- event and status logs saved.

### Safety checks

- motion blocked on unknown or partially known required joints;
- stale previews rejected;
- config changes classified by pose impact;
- Stop/ESTOP clears queues and tool output safely;
- no automatic large encoder correction;
- no indefinite gripper stall;
- joint and TCP limits applied before command upload;
- stale live-jog input stops;
- task abort reports hold uncertainty.

### Regression matrix

| Area | Required regression |
|---|---|
| Home | one press, one command, correct first update, completion state |
| Preview | stale after any intervening pose/model change |
| Joint movement | Apply, live edit, replacement, stop |
| Kinematics | branch, Phi, limit, unreachable, linear path |
| Presets | custom name, save, preview, Go To, outside camera workspace |
| Tasks | detected-only mapping, heights/modes, stale detection, abort |
| Programs | manual inputs, per-step mode, step preview, persistence, demos |
| Camera | polygon coverage, crop/padding, raw/semantic alignment |
| Gripper | grip/no-object/stall/timeout/stop |
| Encoders | raw/calibrated/error/partial-known/fault |

### Repository checks

Before each merge:

```text
cd pc_app
python -m pytest
```

Also run from a clean clone without `robot.local.yaml`, build both firmware
targets, and run browser regression tests. Do not commit local calibration,
logs, caches, virtual environments, or generated test artefacts.

## 10. Open Questions

Only the following remain genuinely unresolved after repository inspection.

1. **What physical event defines each joint's logical zero?**
   - Resolve with photos/fixtures and a written pose table.
2. **Does current "Go Home" have any switches, hard stops, or index sensors?**
   - Inspect wiring and firmware inputs. Until proven, treat it as a normal
     move.
3. **What point was used for the local measured Z samples?**
   - Record whether it was magnet face, gripper tip, flange, or another
     contact point and its relation to workspace Z=0.
4. **Is the active magnet TCP Z=158 mm physically measured from the final DH
   frame, and in the documented tool-forward direction?**
   - Verify with direct measurement and one vertical known-pose test.
5. **Are actuator zero/sign and DH zero/sign intentionally separate for every
   joint?**
   - Write the transform equation and validate positive motion physically.
6. **What speed and acceleration values are mechanically safe?**
   - Determine through staged single-axis tests; do not infer from simulation.
7. **What current sensor and electrical interface will the gripper use?**
   - Select the part and measure baseline/stall current before defining
     thresholds.
8. **Where are the encoders mounted: motor shaft, gearbox input, or joint
   output?**
   - This determines gear conversion, backlash visibility, and correction
     authority.
9. **What is the intended camera-visible boundary versus full work-plate
   boundary?**
   - Place markers at intended edges and compare pixel, robot, and projected
     polygons.
10. **Where should user programs persist?**
    - Recommended experiment: tracked read-only templates plus one ignored
      versioned local YAML/JSON store. Confirm whether programs need sharing
      between machines before choosing a more complex store.

## Recommended Next Codex Prompt

```text
Implement only the first batch from
docs/revised_implementation_plan_from_comments.md:
WP-01 plus the minimum WP-02 Home changes.

Add backend pose revisions and preview start-pose fingerprints, reject stale
path/program/task previews, make Home send exactly one motion command, and
update frontend draft/preview clearing so one Home press and cross-tab moves
stay synchronized. Add focused regression tests. Do not change DH/IK math,
motion limits, Tasks, Program persistence, camera behavior, encoders, or tool
hardware in this batch.
```
