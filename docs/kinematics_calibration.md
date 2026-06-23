# Cartesian Accuracy Calibration

Status: implemented operator workflow; physical results still require real measurements.

## Calibration Layers

Keep these layers separate:

1. **Physical model calibration**
   - Logical joint references, DH zero offsets, base height/side offset, and
     principal link lengths.
   - Changes FK and IK by updating the existing geometry/DH configuration.
2. **Tool/TCP calibration**
   - Defines the physical point controlled by Cartesian motion and its offset
     from flange frame 4.
   - Must be measured independently from final-link geometry because forward
     TCP length and final-link length can be geometrically indistinguishable.
3. **Workspace/camera calibration**
   - Maps camera pixels to robot-base X/Y.
   - Does not measure TCP Z. The workspace-plane Z in the robot-base frame is
     a separate explicit measurement reference.
4. **Residual command correction**
   - Optional tool-specific compensation applied to Cartesian targets before
     IK.
   - Does not change FK and is not evidence that the physical model is correct.

## Operator Workflow

### 1. Verify the reference and tool

- Select the physical tool actually mounted.
- Confirm the configured TCP offset and the exact measured point.
- Measure `calibration.measurement_reference.workspace_plane_z_mm` if the work
  plate is not at robot-base Z=0.
- Use robot-base XYZ millimetres. Touch-off input is converted as:

```text
measured TCP Z =
workspace-plane Z in robot base
+ known surface height above that plane
+ TCP contact-point offset
```

- Treat hardware joint state as estimated unless every required joint has
  validated feedback.

### 2. Collect fit and validation samples

Every saved sample is bound to one executed calibration preview and records:

- intended physical target;
- model command target;
- reported joint angles and whether they are estimated or measured;
- FK-predicted TCP;
- measured TCP;
- tool/model/actuator/workspace/reference signatures;
- measurement method, approach direction, quality, timestamp, and residuals.

Use conservative previewed motion only. No automatic touch-off is performed.
Approach repeated points from the same direction when checking repeatability.

`Generate automatic target set` derives valid joint poses from the active
joint limits and model, filters their FK TCP points through the calibrated
workspace and safe height band, and selects a spread across X/Y/Z/pitch. The
operator does not choose Z or pitch because arbitrary values can produce an
unreachable or weakly identifiable dataset.

The generated set covers:

- left/right and near/far X/Y;
- at least two Z levels;
- at least two tool pitches;
- separate held-out validation targets;
- repeated targets when backlash or compliance is suspected.

The current physical-model fitter requires at least eight fit samples and
rejects poor X/Y, Z, or pitch coverage.

### 3. Interpret residuals

- `model residual = measured TCP - FK(reported joints)`
  - Tests the physical model plus joint-state and measurement quality.
- `IK target residual = FK(reported joints) - model command target`
  - Tests solver convergence and reported joint tracking separately.
- `landing residual = measured TCP - intended physical target`
  - The operator-visible Cartesian landing error.

Useful patterns:

- Nearly constant Z error at several X/Y positions:
  - check workspace/base Z reference and measured point first;
  - then shoulder/elbow references and actuator mapping.
- Z error changing with pitch:
  - check tool TCP length/sign and wrist/shoulder/elbow zeros.
- XY scale/skew:
  - verify camera homography and physical workspace dimensions;
  - then inspect link geometry and base/joint references.
- Different results at the same target or from opposite approaches:
  - likely backlash, compliance, loose mechanics, open-loop repeatability, or
    measurement noise.
- Low IK residual with large model/landing residual:
  - the solver is following the configured model; the model, reference,
    actuator mapping, or physical mechanism is wrong.

## Physical-Model Candidate Fit

The workflow offers constrained parameter groups:

- `joint_zeros`: four DH model zero offsets;
- `geometry_basic`: base height, base side offset, upper arm, and forearm;
- `joint_zeros_geometry`: both groups.

The fitter uses weighted robust nonlinear least squares, checks numerical
identifiability, and evaluates held-out validation samples. It intentionally
does not fit final-link length and forward TCP length together.

A candidate is only applicable when:

- the sample signatures still match the active configuration;
- pose coverage is adequate;
- the selected parameters are identifiable;
- held-out validation passes;
- parameter changes remain within conservative bounds.

Application requires stopped motion, disarmed hardware, and explicit operator
confirmation. It updates the existing geometry/DH values, records an audit
entry, disables residual correction, and invalidates previews. It never moves
the robot.

## Residual Command Correction

Supported models:

- `radial_reach_z_offset`;
- `constant_xyz`;
- `affine_xy_z_offset`.

Use `radial_reach_z_offset` first when the observed error is mostly:

- a constant height bias;
- a constant reach bias, where reach means radial distance from the robot base
  to the TCP in the X/Y plane.

This model preserves the target angle around the base and only changes XY
radius plus Z. It is easier to reason about than an affine XY correction and
does not hide angular, camera-skew, or workspace-scale errors. Use
`affine_xy_z_offset` only after measurements show the residual is not explained
by constant reach and Z offsets.

The reach/Z model can be entered manually in the Settings TCP calibration
panel. Enter offsets as measured-minus-commanded error: positive reach means
the TCP lands farther from the base than requested, and positive Z means it
lands higher. `Save and apply manual offsets` persists and enables the entered
values in one action. Manual offsets still use the same command-correction path,
stale signature checks, IK reachability checks, and path preview as fitted
profiles, but they do not require fit or validation samples. Values beyond the
automatic-fit correction limits remain eligible because they were explicitly
entered by the operator; the UI reports them as warnings. Treat manual values
as operator-entered compensation, not proof that the model is physically
correct.

Validation trials can use a fitted correction at conservative speed without
globally enabling it. Normal Cartesian paths use correction only when:

- global and tool-profile enable flags are set;
- tool/TCP, model, actuator, workspace, and measurement-reference signatures
  are fresh;
- at least two held-out corrected landing samples pass for fitted profiles;
- fitted correction magnitude is below configured automatic-enable limits;
- radial samples are far enough from the base axis when fitting reach
  correction;
- affine sample coverage is adequate.

Joint-space commands and live Cartesian velocity jogging remain uncorrected.
Endpoint Cartesian previews, direct IK solves, programs, named Cartesian
positions, and tasks share the same command-layer path.

## When to Refit

Refit after changing:

- mounted tool or TCP dimensions;
- DH geometry or model zero offsets;
- actuator zero, direction, gearing, or servo mapping;
- workspace-plane Z reference;
- camera/workspace calibration when camera XY was used;
- mechanical assembly, belt/gear preload, or payload;
- measurement method or marker/contact point.

## Remaining Limitations

- Open-loop controller angles are estimates, not independent joint
  measurements.
- Camera XY and manual Z accuracy limit the result.
- Compliance and backlash are load- and direction-dependent and cannot be
  represented by one static kinematic model.
- A passing software fit cannot prove the real robot improved. Re-run physical
  held-out landing targets after applying any model update.
