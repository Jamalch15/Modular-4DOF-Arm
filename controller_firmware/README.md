# Controller Firmware Notes

This folder documents the current ESP32-S3 firmware direction.

The existing PlatformIO firmware in `platformio/src/main.cpp` is preserved as a useful hardware test tool for one stepper and one servo.

A newer open-loop full-arm controller now lives in `platformio/src/arm_controller.cpp`. It accepts dashboard hardware config, drives only enabled valid axes, and simulates disabled axes in its reported pose. This is still exploratory firmware, not final closed-loop robot control.

Before writing final motor-control firmware, collect the missing hardware details:

- Stepper driver type for base and shoulder
- Base stepper pins: direction, step, enable
- Shoulder stepper pins: direction, step, enable
- Servo pin for elbow
- Servo pin for wrist
- Stepper enable polarity
- Microstepping mode
- Gear ratios
- Motor full steps per revolution
- Degrees per output step after gearing
- Servo pulse min/max for the safe range
- Servo mechanical range and joint range mapping
- Joint zero offsets
- Homing or limit switch availability
- Hard-stop locations and safe software margins
- Power and current limits

See `protocol_stub.md` for the proposed starter serial protocol.

## Firmware Targets

Run these from the PlatformIO project:

```powershell
cd "controller_firmware\platformio"
```

Single-axis hardware test:

```powershell
pio run -e esp32-s3-devkitc-1 -t upload
```

No-motor protocol stub:

```powershell
pio run -e esp32-s3-arm-protocol-stub -t upload
pio device monitor -e esp32-s3-arm-protocol-stub
```

Full-arm open-loop controller:

```powershell
pio run -e esp32-s3-arm-controller -t upload
pio device monitor -e esp32-s3-arm-controller
```

Native USB variants are also available:

```powershell
pio run -e esp32-s3-arm-controller-native-usb -t upload
pio run -e esp32-s3-arm-controller-usb-jtag -t upload
```

## Current Full-Arm Behavior

Working assumptions:

- The PC dashboard is the source of truth for geometry, joint limits, pins, gear ratios, microstepping, and servo pulse mapping.
- The PC sends `CONFIG` lines on serial connect or explicit controller sync while hardware is disarmed.
- The ESP stores config in RAM only; reset the ESP and the PC will resync.
- Disabled axes are simulated internally.
- Enabled invalid axes reject config and block arming.
- Partial hardware is allowed and reported as `hw=mixed`.
- Motion is open-loop. Runtime-configured shoulder AS5048A feedback is separate calibrated evidence used for diagnostics and settled verification.
- Optional `CORRECTJ` is a disabled-by-default bounded post-move transaction. It maintains a runtime physical-step bias and never rebases the logical pose.
- `SETPOSE` is an explicit operator assertion used after manual positioning or pose-invalidating actuator configuration changes to establish the open-loop pose. It does not perform physical homing.
- Go Home is a normal move from a known pose. Physical homing remains deferred until switches/index hardware and safe directions are defined.
- `ESTOP` disarms and blocks motion until the controller is reset.

See [../docs/shoulder_encoder_integration.md](../docs/shoulder_encoder_integration.md) for the authority, calibration, fault, and correction contract.

If upload fails with `No serial data received`, the ESP32-S3 did not enter the
ROM bootloader on the selected COM port. Try this sequence:

1. Disconnect any serial monitor or dashboard hardware connection.
2. Hold `BOOT`.
3. Tap and release `RESET` while still holding `BOOT`.
4. Start the upload command.
5. Release `BOOT` only after PlatformIO changes from `Connecting...` to writing
   or reports the chip type.

If that still fails, try the board's other USB port. Re-run `pio device list`
and use the new COM port with either the CH343 or native-USB protocol-stub
environment.
