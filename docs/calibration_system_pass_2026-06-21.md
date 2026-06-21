# Calibration System Pass — June 21, 2026

## Existing Architecture Diagnosis

The current transform chain is:

```text
controller actuator mapping
-> logical joint coordinate q
-> DH theta mapping
-> flange frame 4
-> active tool TCP
-> optional Cartesian command correction before IK
```

Backend FK/IK and the Three.js renderer use the same DH/TCP convention.
Cartesian endpoint previews, programs, named Cartesian positions, and tasks
share the correction path. Joint commands and live Cartesian velocity jogging
do not.

Calibration values live in:

- actuator mapping: `joints[]`;
- geometry and DH: `geometry`, `links_mm`, `kinematics.dh_rows`;
- tool TCP: `tools.presets.*.tcp_offset_mm`;
- workspace/camera: `camera.calibration.workspace_aruco`;
- measurement reference: `calibration.measurement_reference`;
- residual command correction: `kinematics_calibration`.

## Z-Error Diagnosis

The legacy local samples show good software IK/FK agreement but approximately
20–32 mm physical Z error. The four fit samples average about -30.4 mm Z.

Because those samples use pitch near 0 degrees, forward TCP length has almost
no vertical authority in the configured model. A wrong forward TCP length can
still affect radial XY, but it is not the leading explanation for the measured
constant Z error.

Most likely investigation order:

1. wrong robot-base/work-plate Z reference;
2. wrong measured physical point or touch-off offset;
3. shoulder/elbow reference or actuator zero/direction;
4. base height or DH geometry;
5. backlash/compliance/open-loop repeatability;
6. TCP length using varied-pitch samples;
7. residual correction.

The existing legacy affine fit fails held-out validation and must remain
disabled.

## Implemented Changes

- Versioned, signed calibration contexts for model, actuator mapping, tool/TCP,
  workspace map, and measurement reference.
- Stale profiles no longer apply correction.
- Samples require one executed calibration preview and unchanged endpoint.
- Open-loop hardware state is recorded as estimated.
- Multi-height, multi-pitch target generation with distributed validation
  targets and coverage diagnostics.
- Orientation-aware engineering diagnostics for large Z errors.
- Constrained physical-model fitting with identifiability checks and held-out
  validation.
- Explicit, disarmed physical-model application into existing geometry/DH
  configuration; no parallel runtime model.
- Separate validation-trial correction from globally enabled correction.
- Enablement gates for freshness, validation, coverage, and correction size.
- Staged operator UI for reference verification, sample collection, physical
  model fitting, validation, and optional residual correction.

## Verification

- Complete PC application suite: `268 passed`.
- JavaScript module syntax check passed.
- Rendered desktop and mobile-sized calibration UI check passed.
- Target-generation interaction produced 16 reachable/blocked diagnostic
  entries without console errors.
- No firmware/protocol behavior changed.

## Required Physical Measurements

- Work-plate Z in the robot-base frame.
- Exact flange-to-TCP vector for each mounted tool.
- Written mechanical reference pose and positive direction for every joint.
- Repeated TCP measurements at varied X/Y/Z/pitch and consistent approach
  direction.
- Repeatability from opposite approach directions and with representative
  payload.
- Independent validation landings after any model update.
