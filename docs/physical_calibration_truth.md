# DH, Joint Convention, TCP, and Physical Calibration Truth

Status: working calibration contract, not final physical proof.

## Transform Chain

Current idea for the robot model:

```text
actuator sensor/steps
-> logical joint coordinate q
-> DH model joint theta
-> DH frame 4 / wrist-flange frame
-> active tool TCP frame
-> optional Cartesian command correction
```

The optional Cartesian command correction is last. It can compensate repeatable
landing error, but it is not proof that geometry, joint zeros, actuator signs,
or TCP dimensions are correct.

## Settings Split

- `joints[].zero_offset_deg` and `joints[].direction_sign` describe actuator
  mapping into the logical joint coordinate used by the PC/controller contract.
- `kinematics.dh_rows[].zero_offset_deg` and
  `kinematics.dh_rows[].direction_sign` describe how logical joint coordinate
  `q` becomes Standard-DH `theta` for FK and IK:

```text
theta = q * direction_sign + zero_offset_deg + theta_offset_deg
```

- `joints[].home_deg` is the configured home/reference angle. Unless hardware
  switches or index sensors prove the pose, it is not a physical homing proof.
- `tools.presets.*.tcp_offset_mm` is the active tool offset from flange to TCP.
  Tool `+Z` is the tool-forward/TCP-length direction and maps to local DH `+X`.

## Frames

- Robot base: declared base mounting reference in millimeters, `+X` sideways,
  `+Y` forward, `+Z` up.
- DH frame 4 / wrist-flange: final DH frame before tool offset.
- Tool frame: origin at flange; tool `+Z` is the configured forward direction.
- TCP: Cartesian FK/IK point used by previews, tasks, and programs.
- Workspace plane: camera-calibrated robot X/Y plane with its robot-base Z
  stored separately in `calibration.measurement_reference`; not the full reach
  limit.
- Camera/image: pixel frame transformed into workspace X/Y by calibration.

## Z Error Audit Order

For the current observed low-Z error, audit in this order:

1. Measurement reference and workspace `Z=0`.
2. The physical point being measured: magnet face, gripper tip, flange, marker,
   or another contact point.
3. Active tool TCP length and sign.
4. Joint reference zeros and positive directions.
5. Base/shoulder heights, side offsets, and final wrist/flange link.
6. DH signs and offsets.
7. Repeatability, backlash, and compliance.
8. Cartesian command correction, only after the model is accepted.

## Current Local Evidence

The saved June 19, 2026 magnet samples are legacy/unsigned samples and must be
recaptured with the staged workflow before they can enable correction.
Nevertheless, they are useful diagnostic evidence:

- four fit samples have approximately -30.4 mm mean Z residual with about
  1.8 mm standard deviation;
- FK-to-command error is below about 0.7 mm;
- the samples use tool pitch near 0 degrees, where the modeled tool-forward
  axis has almost no vertical component;
- therefore the configured forward TCP length alone is unlikely to explain
  the observed 30 mm vertical error in those poses;
- a repeated nominal target differs by roughly 7 mm in measured Z, so
  repeatability and measurement procedure also require attention;
- the affine residual fit fails the held-out validation sample and should not
  be enabled.

Working diagnosis order for new measurements:

1. robot-base versus work-plate Z datum;
2. measured physical point and contact/marker offset;
3. shoulder/elbow reference and actuator mapping;
4. base height and DH/model geometry;
5. tool TCP using poses with meaningful pitch variation;
6. backlash/compliance and direction-dependent repeatability;
7. residual command correction only after held-out validation.

## Measurement Sheet

Keep physical measurement files local unless deliberately sharing calibration.
Each row should include:

- pose ID;
- tool name;
- reported joint angles in degrees;
- reference condition;
- measured flange XYZ in mm if possible;
- measured TCP XYZ in mm;
- measurement method;
- expected FK;
- residual.

Use the local report helper from `pc_app`:

```powershell
python tools/calibration_truth_report.py --config config/robot.example.yaml
python tools/calibration_truth_report.py --measurements path\to\private_measurements.yaml
```

The helper prints to stdout by default and does not save local calibration data.
