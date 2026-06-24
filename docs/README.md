# Project Documentation

This folder contains implementation notes, calibration references, and planning
history for the ARES-4 prototype. The top-level [README](../README.md) is the
best starting point for the repository as a whole.

## System And Architecture

- [architecture.md](architecture.md): PC/controller split, data flow, and
  current motion boundaries.
- [open_questions.md](open_questions.md): unresolved design and hardware
  questions.
- [tasks/README.md](tasks/README.md): task-layer notes and generated sequence
  expectations.

## Calibration And Robot Model

- [kinematics_calibration.md](kinematics_calibration.md): operator-facing TCP
  calibration workflow.
- [physical_calibration_truth.md](physical_calibration_truth.md): DH, joint
  convention, TCP, and calibration-chain contract.
- [calibration_system_pass_2026-06-21.md](calibration_system_pass_2026-06-21.md):
  calibration architecture diagnosis and measurement workflow.
- [shoulder_encoder_integration.md](shoulder_encoder_integration.md): shoulder
  encoder authority, calibration, fault handling, and correction semantics.

## Vision

- [vision_integration.md](vision_integration.md): workspace homography,
  color-object detection, and detector-neutral vision contract.

## Planning History

These notes are retained as project history and context. They are useful for
understanding how the current implementation evolved, but the runnable code and
focused implementation docs above should take priority when they disagree.

- [planning/component_master_plan.md](planning/component_master_plan.md)
- [planning/accelerated_exam_demo_plan.md](planning/accelerated_exam_demo_plan.md)
- [planning/remaining_implementation_plan.md](planning/remaining_implementation_plan.md)
- [planning/revised_implementation_plan_from_comments.md](planning/revised_implementation_plan_from_comments.md)
