# Full-Arm Serial Protocol

This is the current line-based serial protocol used by the Python dashboard, the no-motor protocol stub, and the open-loop full-arm controller.

It is not final closed-loop motor-control firmware. Hardware details are still changing, so implementation should remain incremental and testable.

## Transport

Initial transport:

```text
USB-C serial from PC to ESP32-S3
```

Future transport:

```text
Bluetooth may be added later, but should implement the same command interface.
```

Line ending:

```text
\n
```

Encoding:

```text
ASCII text
```

## Commands

```text
HELLO
STATUS
CONFIG BEGIN axes=4
CONFIG JOINT index=1 name=base actuator=stepper enabled=0 step=-1 dir=-1 enable=-1 enable_low=1 driver=TB6600 full_steps=200 microsteps=16 gear=1.000 zero=0.000 sign=1 min=-160.000 max=160.000 home=0.000 max_speed=45.000 max_accel=120.000
CONFIG JOINT index=3 name=elbow actuator=servo enabled=0 pwm=-1 min_us=500 max_us=2500 freq=50 servo_range=270.000 neutral=135.000 gear=1.000 zero=0.000 sign=1 min=-120.000 max=120.000 home=20.000 max_speed=60.000 max_accel=180.000
CONFIG TOOL name=gripper active=1 type=servo_gripper tcp_x=0.000 tcp_y=0.000 tcp_z=30.000 open=0.000 close=1.000 pwm=9 min_us=500 max_us=2500 freq=50
CONFIG TOOL name=magnet active=0 type=electromagnet tcp_x=0.000 tcp_y=0.000 tcp_z=18.000 pin=-1 active_high=1
CONFIG ENCODER_BUS enabled=0 type=spi sck=12 miso=13 mosi=14 clock=1000000 sample_ms=100
CONFIG ENCODER joint=2 name=shoulder sensor=as5048a enabled=0 cs=15 reference_raw=0.000 reference_joint=20.000 sign=1 wrap=360.000 mounting=joint_output turns=1.000000 freshness_ms=500 max_noise=0.500 calibrated=0 calibration_id=none
CONFIG ENCODER_POLICY mode=diagnostic policy=diagnostic settle_ms=300 samples=3 warn=2.000 fault=5.000 hysteresis=0.250 require=0 correction=0 validation_id=none max_delta=1.000 limit_margin=2.000 correction_speed=2.000 correction_accel=10.000 attempts=2
CONFIG END
ARM 0
ARM 1
SETPOSE j1 j2 j3 j4
MOVEJ j1 j2 j3 j4 speed accel
JOGJ j1 j2 j3 j4 speed accel
JOGV v1 v2 v3 v4 accel
SERVOJ j1 j2 j3 j4 duration_s
JOG STOP
TRAJ BEGIN count=3 duration=1.000 speed=25.000 accel=100.000
TRAJ POINT index=0 t=0.000 j1=0.000 j2=20.000 j3=20.000 j4=0.000
TRAJ POINT index=1 t=0.500 j1=5.000 j2=25.000 j3=15.000 j4=0.000
TRAJ POINT index=2 t=1.000 j1=10.000 j2=30.000 j3=10.000 j4=0.000
TRAJ START
TRAJ CLEAR
CORRECTJ joint=2 delta=-0.250000 speed=2.000000 accel=10.000000 id=transaction-id
STOP
ESTOP
HOME
TOOL OPEN
TOOL CLOSE
TOOL SET value=0.000
TOOL ON
TOOL OFF
```

Angles are in degrees at the joint output, not raw motor shaft degrees.

`speed` is a provisional maximum joint speed in deg/s.

`accel` is a provisional acceleration limit in deg/s^2.

Example:

```text
MOVEJ 0.0 25.0 -30.0 10.0 25.0 100.0
```

`SERVOJ` is the preferred command for live Cartesian jogging. It gives all four joints one synchronized target and duration for the next short servo period. Cartesian velocity smoothing, direction preservation, singularity scaling, and joint-limit handling happen once on the PC. `JOGJ` and `JOGV` remain compatibility commands. Use `JOG STOP` when the operator releases the live jog control.

`TRAJ` uploads a complete timed joint-space path before motion starts. Points must be sent in increasing `index` order, `t` is seconds from the start of the trajectory, the first point must be at `t=0`, and the last point must match the declared duration. The full-arm controller interpolates between uploaded points while the actuator loop runs; this avoids treating previewed joint, Cartesian, or program paths as independent `MOVEJ` endpoints.

Current implementation note: this is still open-loop target/velocity following, not final closed-loop motion control. `HOME` is a legacy "move to configured home pose" command and requires an already-known pose; it is not physical homing. The PC dashboard normally implements Go Home through the same timed `TRAJ` execution path as other planned joint moves. `MOVEJ` remains available as a low-level endpoint command. The controller validates joint limits and clears the queue on `STOP`, `ESTOP`, `HOME`, `SETPOSE`, `MOVEJ`, disarm, and config changes. Low-level stepper pulse generation is still simple and should be validated on hardware.

## Responses

Controller identity:

```text
HELLO name=esp32s3-arm firmware=arm_controller protocol=4 config=1 encoder=1
```

Status:

```text
STATUS state=idle homed=0 armed=0 hw=mixed enabled=1000 j1=0.0 j2=20.0 j3=20.0 j4=0.0 fault=OK
```

Protocol v4 status separates open-loop joint estimates from shoulder evidence:

```text
STATUS state=idle homed=0 known=1 known_mask=1111 pose_source=open_loop_estimate armed=1 hw=mixed enabled=1100 enc=0100 enc_valid=0100 e2=20.0 er2=8192 ea2=180.0 em2=20.0 eage2=40 enoise2=0.08 evalidn2=4 ef2=none j1=12.4 j2=20.0 j3=20.0 j4=0.0 closed_loop=diagnostic correction=idle correction_id=none correction_delta=0 correction_steps=0 correction_attempts=0 cb1=0 cb2=0 cb3=0 cb4=0 tool_type=generic tool=open tool_value=0.000 fault=OK
```

`j1..j4` are estimates. Encoder availability never changes `known` or `known_mask`. Legacy `e1`/`e2` fields are retained only as diagnostic compatibility fields.

Error:

```text
ERR code=LIMIT message=joint_2_target_out_of_range
```

Acknowledgement:

```text
OK command=MOVEJ
OK command=JOGJ
OK command=JOGV
OK command=SERVOJ
OK command=JOG_STOP
OK command=TRAJ_BEGIN count=3 duration=1.000
OK command=TRAJ_POINT index=0
OK command=TRAJ_START count=3 duration=1.000
OK command=TRAJ_CLEAR
OK command=CORRECTJ joint=2 delta=-0.250000 steps=-18 attempt=1 id=transaction-id
OK command=CONFIG axes=4 hw=mixed enabled=1000 pose_invalidated=0
OK command=ARM armed=1
OK command=SETPOSE
OK command=TOOL state=open value=0.000
```

## Expected Controller States

```text
idle
homing
moving
stopped
estop
fault
```

## Hardware Config Rules

- The PC dashboard is authoritative and sends config on serial connect/save.
- `CONFIG` requires stopped, disarmed hardware.
- Changes to actuator zero, sign, gearing, servo mapping, or home reference invalidate the controller's known pose and return `pose_invalidated=1`.
- After a pose-invalidating config sync, the operator must verify the physical arm and issue `SETPOSE` while disarmed.
- ESP config is RAM-only for now.
- Unknown pins use `-1`.
- Disabled axes are simulated in reported joint state.
- Enabled stepper axes require at least STEP and DIR pins, positive full steps/rev, positive microsteps, and positive gear ratio.
- TB6600 microstep pins are physical DIP switches, so the protocol keeps only the `microsteps` value for step math.
- Enabled servo axes require PWM pin, valid pulse range, positive PWM frequency, positive servo range, and positive gear ratio.
- The active tool is configured with `CONFIG TOOL active=1`.
- An active `servo_gripper` requires a PWM pin and valid pulse range.
- An active `electromagnet` requires a GPIO pin and active polarity.
- Inactive tool presets may keep unknown pins as `-1`.
- Encoder, actuator, servo, active-tool, SPI, and chip-select GPIO uses must not conflict.
- Only the shoulder AS5048A is supported by this integration.
- Raw readback is diagnostic until calibration is validated and fresh.
- Motor/gear-side mounting is not absolute joint-output evidence.
- Correction requires joint-output mounting, validated calibration, an explicit local validation record, idle armed hardware, and bounded limits.
- `hw=hardware` means all axes are valid physical axes.
- `hw=mixed` means at least one physical axis and at least one simulated axis.
- `hw=simulated` means no physical axes are enabled.
- `hw=invalid` means at least one enabled axis has invalid config.

## Incremental Firmware Plan

1. Keep the current single-stepper/single-servo firmware available for hardware tests.
2. Keep the no-motor protocol stub available for safe PC integration tests.
3. Use `arm_controller.cpp` for full-arm open-loop testing.
4. Test one physical axis at a time by enabling only that axis in dashboard Settings.
5. Add homing only after switch availability and safe directions are known.
6. Hardware-validate AS5048A readback, wrap, disconnect, magnetic diagnostics, and calibration repeatability.
7. Keep bounded post-move correction disabled until a local validation record exists.
8. Refine synchronized low-level motion after hardware tests if the current open-loop trajectory follower is not smooth enough.

## Safety Requirements For Real Firmware

- `ESTOP` must immediately stop commanded motion and block new movement.
- `STOP` must cancel active motion without requiring a reset.
- Joint targets must be checked against configured limits.
- Unknown commands must not move hardware.
- Motion should not start until a known pose exists.
- `SETPOSE` establishes an operator-asserted open-loop pose but does not set `homed=1`.
- `homed=1` is reserved for a future physical switch/index homing procedure.
- Hardware motion must require `ARM 1`.
- Tool commands should fail safely during E-stop and should be treated like hardware motion when a physical tool is attached.
- Servo pulses must stay inside measured safe ranges.
- Stepper outputs must respect driver timing requirements.
- Enable pins and power state should fail safe where possible.

## Details Still Needed

Do not guess these before writing final motor-control code:

- Stepper driver model and timing requirements
- Base and shoulder pin assignments
- Servo pins
- Enable pin polarity
- Microstepping configuration
- Gear ratios
- Steps per revolution
- Servo pulse min/max
- Joint zero offsets
- Homing or limit switch wiring
- Safe direction for homing
- Physical hard-stop positions
- AS5048A chip-select pins, zero offsets, and direction mapping for base and shoulder
