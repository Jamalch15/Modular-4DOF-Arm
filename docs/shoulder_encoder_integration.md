# Shoulder Encoder Integration

## Scope

The shoulder AS5048A is integrated as calibrated per-joint evidence. The robot remains PC-planned and open-loop at the motion-command level.

Continuous closed-loop stepper control, in-trajectory correction, automatic full-pose adoption, and encoder-driven TCP control are out of scope. The calibrated shoulder encoder may optionally update the shoulder planning/estimated angle while the robot is idle; this is per-joint pose tracking, not full-pose adoption.

## Pose contract

- **Commanded** is the target of the active motion transaction.
- **Planning/reported** is the pose used by FK, IK, previews, programs, tasks, presets, and the solid Three.js arm.
- **Estimated** is the controller step-count/PWM model. It is currently the same numeric vector as the planning pose.
- **Measured** is fresh calibrated evidence for one joint. It is nullable and has an independent validity mask.
- **Known** is derived from the planning-known mask. One shoulder measurement never makes the complete robot or TCP pose known.

`reported_angles_deg` remains a compatibility name for the planning pose. Diagnostic encoder telemetry updates `encoder_telemetry_revision`, not `pose_revision`, so ordinary sensor noise does not invalidate previews. When `encoders.pose_tracking.enabled` is active and all safety/health gates pass, a fresh calibrated shoulder measurement intentionally updates the shoulder entry of the planning/estimated pose, increments `pose_revision`, and marks only the shoulder joint authority as measured.

## Implemented stages

### 1. Unsafe partial support neutralized

- Legacy `settle_correction` configuration is migrated to diagnostic mode.
- Raw encoder fields no longer overwrite the planning pose. Only calibrated, fresh, stable shoulder evidence can update planning pose, and only through the explicit idle pose-tracking gate.
- Encoder availability no longer establishes firmware whole-pose authority.
- Encoder, actuator, servo, tool, SPI, and chip-select GPIO conflicts block controller sync.
- Encoder I/O and calibration changes invalidate measurement authority without discarding unrelated open-loop estimates.

### 2. Diagnostic raw readback

- The normal controller build includes runtime-configured AS5048A SPI support.
- Status includes raw count, raw degrees, parity/error health, magnetic diagnostics, age, and idle noise.
- Missing, stale, parity-invalid, CORDIC-overflow, or magnet-invalid samples have no measurement authority.
- Raw telemetry never participates in motion decisions.

### 3. Calibrated shoulder measurement

The reversible transform is:

```text
joint_deg =
reference_joint_deg
+ direction_sign
* unwrap(raw_deg - reference_raw_deg)
/ sensor_turns_per_joint_turn
```

The schema records units, reference angles, sign, wrap policy, mounting location, scale, reference description, freshness, noise, and calibration identity.

The manual guided API and Settings workflow require disarmed, stopped hardware and at least two independently known shoulder references. The fit validates sign, scale, wrap crossing, residual error, joint limits, and single-turn ambiguity.

The Settings page now exposes a fast operator workflow first:

1. Put the shoulder on one physical reference mark.
2. Enter that shoulder angle and use Set Pose if the planning pose does not already match it.
3. Disarm and use quick calibration. This stores the current raw AS5048A angle as the reference for the known shoulder angle, using an explicit direction sign and the working assumption of one sensor turn per shoulder turn.
4. Arm and run backlash check. The app approaches the same center angle from below and above, captures settled output-encoder measurements, and reports the branch separation.
5. Enable post-move correction only after the calibrated readback is stable and the current mismatch is within the configured correction limit.

This is intentionally closer to common backlash practice than the earlier sweep-first workflow: use a simple reference/offset calibration, then directly measure reversal error at the load/output side.

The Settings page still provides an assisted sweep for range validation. The operator places the shoulder at one real known start angle, uses Set Pose to assert that start angle, arms hardware, and starts the sweep. The sweep uses the normal motion planner to step through a bounded shoulder range, waits for each move to stop and settle, and captures raw encoder samples at each stopped point. In-motion encoder readings are not used as calibration authority.

For backlash-heavy arms, the assisted sweep preloads the shoulder and then captures all calibration-fit samples from one final approach direction. The initial known-angle sample is retained as a sanity check but is not used for the fit, because it may be on an unknown backlash branch. This produces a stable raw-to-joint map for the loaded branch instead of averaging two different mechanical states.

If one loaded branch is repeatable but not linear enough for the residual limit, the optional sweep validator may accept a monotonic `piecewise_linear` map. This is intended for repeatable AS5048A magnet/alignment nonlinearity or linkage nonlinearity after backlash has been taken up. It is only used inside the calibrated raw-angle range with a small extrapolation margin. It is not accepted from mixed approach directions, because that would bake backlash/lost motion into the calibration.

If bidirectional/manual samples show backlash/lost motion, the validator no longer treats that as ordinary encoder noise. It first tries the normal linear raw-to-joint fit. If that fails, it tries a direction-aware model:

```text
commanded_joint_deg =
neutral_raw_to_joint(raw_deg)
+ approach_bias_deg * approach_direction
```

The calibrated encoder transform uses either the linear slope/offset or the accepted piecewise map. The fitted branch separation is recorded as `backlash_estimate_deg` and shown in the UI. If the branch separation is localized rather than constant, the validator reports the largest paired local separation and tells the operator to use a same-direction/preloaded sweep or physical fixture references. This keeps backlash visible for diagnostics, verification thresholds, and any later bounded correction. It does not make the open-loop commanded pose equal to the measured output pose.

Joint-output mounting may receive calibrated measurement authority. Motor-shaft and gearbox-input mounting remain diagnostic-only in this implementation because persistent multi-turn turn counting is not implemented.

### 4. Per-joint evidence

`/api/state` exposes:

- `planning_angles_deg` / `reported_angles_deg`
- `estimated_angles_deg`
- `measured_angles_deg`
- `pose_known_mask`
- `measurement_valid_mask`
- `joint_authority`
- structured `encoder_evidence`
- mismatch and correction transaction state

Position Library captures retain the planning pose and add evidence metadata. Calibration captures explicitly identify the complete vector as estimated/simulated rather than measured.

### 4a. Idle shoulder pose tracking

`encoders.pose_tracking` is a PC-side policy. When enabled, the app can make the shoulder planning/estimated angle follow the calibrated shoulder encoder while the controller reports `idle` or `stopped`.

Required gates:

- encoder bus and shoulder axis enabled;
- validated joint-output shoulder calibration;
- fresh sample within age/noise limits;
- measured shoulder angle inside configured shoulder limits;
- latest controller status is `idle` or `stopped`; `moving`, ESTOP, and Fault statuses do not track;
- measured jump from the current planning shoulder does not exceed `max_jump_deg`.

When applied, only joint 2 is changed. Base, elbow, wrist, and TCP are not treated as measured. If the other joints are unknown, the whole pose remains unknown even though the shoulder bit may become known. This lets the desktop view and future plans start from the real shoulder angle after manual movement without moving the motor. App-side queued/executing metadata does not suppress an idle controller STATUS update; otherwise a pre-move status refresh could reintroduce stale open-loop shoulder estimates.

When firmware reports an active/completed correction transaction or a nonzero correction bias, that bias owns the physical-to-logical shoulder mapping. Encoder tracking continues to report residual evidence but does not rewrite the planning pose or target. This prevents an intermediate correction sample from becoming the next task step's logical target.

For hardware stepper motion, the controller's internal step position must match the encoder-tracked planning shoulder before any absolute `TRAJ`/`MOVEJ` is allowed. If encoder tracking changes the shoulder away from the controller-reported open-loop shoulder beyond the controller rebase tolerance, the PC marks `controller_pose_rebase_required`. When validated bounded correction is enabled, that tolerance is at least the correction deadband so small residuals that the robot deliberately will not correct do not force a disarm/re-arm loop. Arming from a disarmed state automatically sends `SETPOSE` with the current planning pose before `ARM 1`, which rebases the controller without moving the motor. If the controller is already armed while that rebase is required, normal motion is blocked and the operator must disarm/re-arm before moving. This prevents a path planned from the encoder shoulder from being executed by firmware that still thinks the shoulder is at an old step-count angle.

If pose tracking refuses a large jump, normal hardware motion is blocked until the operator verifies calibration/state or explicitly uses a safe recovery workflow.

Path previews tolerate only small encoder-tracked shoulder drift between Preview and Execute. The current working default is 2 degrees via `pose_tracking.preview_stale_tolerance_deg`; base, elbow, and wrist still use the strict preview-start tolerance. If the shoulder drift is inside that window, the first trajectory waypoint is rebased to the current encoder-tracked shoulder before upload. Larger drift still requires a new preview.

### 5. Settled mismatch verification

Endpoint and uploaded trajectory completion use one post-motion verification service. It waits for idle, applies the configured settle delay, requires multiple fresh stable samples, and compares measured shoulder angle with both the open-loop estimate and final command.

Policies are diagnostic, warning, or fault. A fault:

- cancels normal motion authority;
- sends `STOP` while leaving the stepper hold behavior to the controller;
- clears only the shoulder planning-known bit;
- does not overwrite the planning pose;
- requires explicit fault acknowledgement followed by Set Pose.

Sensor loss faults only when `require_encoder` and fault policy are both enabled.

### 6. Optional bounded correction

Correction is disabled in tracked defaults and after guided calibration. Enabling it requires a local validation record, joint-output mounting, valid calibration, known planning pose, fresh stable evidence, idle armed hardware, an allow-listed motion source, and bounded angle/speed/acceleration/attempt limits.

`CORRECTJ` moves only the shoulder by a relative pulse transaction after normal known-pose motion. Firmware maintains a runtime correction bias between logical and physical step positions; commanded and estimated logical angles are not rebased to the encoder. Every transaction is also checked against an explicit joint-limit margin using the fresh measured shoulder angle plus the requested delta.

Correction has three separate thresholds. `deadband_deg` is the small-error zone where the robot deliberately does nothing to avoid chasing noise. `max_delta_deg` caps each automatic correction transaction. `align_max_delta_deg` caps the total bounded recovery movement after repeated measurements.

Before eligible joint, home, path, program, and task moves, the PC automatically checks the settled shoulder against the current logical target. A recoverable mismatch is aligned without operator input using slow correction speed/acceleration, chunks no larger than `max_delta_deg`, a fresh encoder check after every chunk, and the total `align_max_delta_deg` cap. Post-move verification uses the same recovery when the ordinary `CORRECTJ` attempts are insufficient. Closed-loop tasks therefore align before camera-clear and before every movement step.

Go Home remains a planned move to the configured home pose, not physical homing. The operator-facing **Align** action remains available for startup and explicit maintenance recovery. With firmware that advertises `alignj=1`, Align can run before full Set Pose by sending the dedicated shoulder-only `ALIGNJ` command. `ALIGNJ` does not mark the full robot pose known and does not infer the other joints.

The correction transaction reports ID, requested delta, emitted steps, attempts, state, and bias. Automatic recovery does not override missing/stale/noisy encoder evidence, invalid calibration, unknown pose, ESTOP/Fault, joint-limit margins, disabled correction policy, controller communication failure, timeout, or non-convergence within `align_max_delta_deg`. These conditions latch or report a fault because a trustworthy bounded correction cannot be constructed.

Automatic post-move correction can run after eligible joint endpoint, path, program, task, and Go Home moves when the source is allow-listed. It remains disabled for live jog, calibration capture, active trajectories, unknown poses, Stop, ESTOP, or Fault. Go Home is still a normal planned move to the configured home pose, not physical homing, and does not discover absolute pose by itself.

The Settings page exposes correction through **Validate + enable bounded correction**, not a raw checkbox. Any later edit to encoder runtime settings, verification thresholds, calibration fields, or correction limits disables correction and requires a fresh validation record.

## Public interfaces

Calibration:

```text
POST /api/encoder/calibration/start
POST /api/encoder/calibration/sample
POST /api/encoder/calibration/validate
POST /api/encoder/calibration/commit
POST /api/encoder/calibration/sweep/start
GET  /api/encoder/calibration/session/{session_id}
POST /api/encoder/calibration/sweep/cancel
POST /api/encoder/correction/policy
POST /api/encoder/fault/clear
```

Protocol v4 adds `CONFIG ENCODER_BUS`, `CONFIG ENCODER`, `CONFIG ENCODER_POLICY`, encoder evidence fields, `known_mask`, correction transaction fields, `CORRECTJ`, and firmware capability `alignj=1`/command `ALIGNJ` for explicit shoulder-only startup alignment.

Legacy status lines remain parseable. Legacy `e1`/`e2` values are diagnostic-only.

## Hardware gates still requiring physical validation

- Resolve actual SCK/MISO/MOSI/CS wiring and all GPIO conflicts.
- Confirm whether the sensor is genuinely joint-output mounted.
- Measure magnet alignment and verify AS5048A diagnostic behavior.
- Validate wrap behavior and power-cycle repeatability across the complete shoulder range.
- Measure approach-direction repeatability, backlash, payload effects, and safe correction limits.
- Create a local correction validation record only after those tests pass.

Do not program AS5048A OTP zero during initial validation. Keep the reversible calibration in `robot.local.yaml`.

AS5048A parity, error, and diagnostic handling follows the [ams OSRAM AS5048A datasheet](https://look.ams-osram.com/m/287d7ad97d1ca22e/original/AS5048-DS000298.pdf).
