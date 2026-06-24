# ARES-4 Modular 4DOF Robot Arm

ARES-4 is a mechatronics prototype for object handling with a modular 4DOF
robot arm. The system combines a browser-based PC control app, an ESP32-S3
controller layer, interchangeable end-effector support, vision-assisted task
workflows, and the supporting mechanical and electronics design files.

The repository is intended to accompany the final report as the source-code
and design-file reference. It keeps the runnable software, firmware, test
suite, configuration template, and engineering notes in one place without
committing draft report PDFs or course handouts.

## Project Overview

The arm is built as a compact serial manipulator for small-object handling.
The current prototype uses two stepper-driven inner joints, two servo-driven
outer joints, and a quick-swap tool interface for end effectors such as a
gripper or electromagnet.

The software is split into two main layers:

- PC side: local GUI, robot model, forward/inverse kinematics, trajectory
  generation, task sequencing, camera processing, calibration, logging, and
  serial communication.
- Controller side: ESP32-S3 firmware for hardware-facing actuator commands,
  tool control, safety state, configuration sync, and status reporting.

The normal demo flow is:

```text
camera or operator input
-> robot-frame target
-> task sequence
-> IK and trajectory preview
-> operator approval
-> ESP32-S3 motion/tool commands
-> status feedback to the GUI
```

## Current Features

- Local FastAPI/WebSocket backend with a browser dashboard.
- 3D robot viewport with current pose, preview pose, target marker, path line,
  and projected workspace camera layer.
- Manual joint controls, Cartesian target preview, live jog modes, and
  reusable program builder.
- Pick-and-place and color-sorting task workflows with preview before
  execution.
- Standard DH-based forward kinematics and numerical Jacobian inverse
  kinematics.
- Configurable robot geometry, joint limits, tool offsets, motion limits,
  hardware IO, camera calibration, and task destinations.
- ESP32-S3 PlatformIO firmware targets for single-axis tests, no-motor protocol
  testing, and the open-loop full-arm controller.
- Shoulder AS5048A encoder readback support for calibrated evidence,
  diagnostics, and optional bounded correction workflows.
- Python test suite covering configuration, kinematics, motion, safety,
  protocol, vision/task integration, calibration, and program behavior.

## Repository Layout

```text
pc_app/                 PC dashboard, backend, tests, tools, and config template
controller_firmware/    ESP32-S3 PlatformIO firmware and serial protocol notes
docs/                   Architecture, calibration, vision, and planning notes
electronics/            KiCad schematics, PCB files, footprints, and symbols
Mechanical/             Mechanical calculation files
.github/workflows/      GitHub Actions test workflow for the PC app
```

Detailed subsystem notes:

- [pc_app/README.md](pc_app/README.md): dashboard setup, GUI guide,
  configuration, calibration, and protocol details.
- [controller_firmware/README.md](controller_firmware/README.md): PlatformIO
  environments, firmware roles, upload notes, and controller behavior.
- [docs/README.md](docs/README.md): documentation map for architecture,
  calibration, vision, task, and historical planning notes.
- [AGENTS.md](AGENTS.md): repository guidance for coding agents.

## Quick Start

The dashboard can run in simulation mode without the ESP32-S3 connected.

```powershell
cd pc_app
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
uvicorn app.main:app --reload --host 127.0.0.1 --port 8000
```

Open:

```text
http://127.0.0.1:8000
```

For later runs, activate the virtual environment and start `uvicorn` again.
The tracked `pc_app/config/robot.example.yaml` is the simulation-safe fallback.
Machine-specific calibration and hardware settings are saved in
`pc_app/config/robot.local.yaml` when present.

## Testing

Run the PC app test suite from `pc_app`:

```powershell
cd pc_app
python -m pytest
```

GitHub Actions runs the same suite on Ubuntu from a clean checkout. Tests that
depend on the committed reference robot configuration should use the example
config explicitly, because local machines may also have an ignored
`robot.local.yaml`.

## Firmware Build

PlatformIO is used for the ESP32-S3 firmware.

```powershell
cd controller_firmware\platformio
pio run -e esp32-s3-arm-controller
```

Useful upload targets are documented in
[controller_firmware/README.md](controller_firmware/README.md). The main
prototype controller is `platformio/src/arm_controller.cpp`; `main.cpp` remains
as a single-axis hardware test sketch.

## Configuration And Calibration

The PC app keeps the robot model in YAML configuration:

- `pc_app/config/robot.example.yaml` is tracked and should stay generic.
- `pc_app/config/robot.local.yaml` is ignored and stores machine-specific
  geometry, calibration, IO pins, camera settings, and task setup.
- `pc_app/config/programs.local.json` is ignored and stores local user-created
  motion programs.

Calibration-related notes are collected in [docs/README.md](docs/README.md).
The most relevant implementation references are the kinematics calibration,
workspace vision calibration, and shoulder encoder documents.

## Safety Notes

This is a prototype robot system. Preview planned motion in the dashboard
before executing it on physical hardware, keep joint limits conservative, and
keep the workspace clear while armed. Real hardware movement requires a known
pose, synced hardware configuration, an armed controller, and a valid motion
request. `STOP` and `ESTOP` paths are implemented in the app/controller flow,
but they do not replace normal lab safety practices.

The current control approach is mostly open-loop during motion. Encoder
readback is used as calibrated evidence for the shoulder joint and optional
post-move checks; it is not a complete closed-loop Cartesian control system.

## Local Files And Generated Output

The repo intentionally ignores local and generated files such as:

- Python virtual environments, caches, and test output.
- PlatformIO build output under `.pio/`.
- Local robot/app calibration files.
- KiCad lock files, backup folders, and history folders.
- Localhost server logs.

Before submitting or pushing, check:

```powershell
git status --short
```

Only source, documentation, design files, and intentional configuration
templates should be committed.

## AI Usage

AI tools were used as support for drafting, organization, code review, and
implementation assistance during development. The design choices, validation,
testing, and submitted project work remain the responsibility of the project
authors.
