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
CONFIG END
ARM 0
ARM 1
SETPOSE j1 j2 j3 j4
MOVEJ j1 j2 j3 j4 speed accel
JOGJ j1 j2 j3 j4 speed accel
JOGV v1 v2 v3 v4 accel
JOG STOP
TRAJ BEGIN count=3 duration=1.000 speed=25.000 accel=100.000
TRAJ POINT index=0 t=0.000 j1=0.000 j2=20.000 j3=20.000 j4=0.000
TRAJ POINT index=1 t=0.500 j1=5.000 j2=25.000 j3=15.000 j4=0.000
TRAJ POINT index=2 t=1.000 j1=10.000 j2=30.000 j3=10.000 j4=0.000
TRAJ START
TRAJ CLEAR
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

`JOGV` is the preferred command for smooth live Cartesian/joint jogging. It streams joint velocities in deg/s, the full-arm controller ramps those velocities with the provided acceleration limit, and the watchdog ramps toward zero if PC updates stop. `JOGJ` remains available as an absolute jog target compatibility command, but it can be jerky for small live updates. Use `JOG STOP` when the operator releases the live jog control.

`TRAJ` uploads a complete timed joint-space path before motion starts. Points must be sent in increasing `index` order, `t` is seconds from the start of the trajectory, the first point must be at `t=0`, and the last point must match the declared duration. The full-arm controller interpolates between uploaded points while the actuator loop runs; this avoids treating every Cartesian waypoint as an independent `MOVEJ`.

Current implementation note: this is still open-loop target/velocity following, not final closed-loop motion control. The controller validates joint limits and clears the queue on `STOP`, `ESTOP`, `HOME`, `SETPOSE`, `MOVEJ`, disarm, and config changes. Low-level stepper pulse generation is still simple and should be validated on hardware.

## Responses

Controller identity:

```text
HELLO name=esp32s3-arm firmware=arm_controller protocol=3 config=1
```

Status:

```text
STATUS state=idle homed=0 armed=0 hw=mixed enabled=1000 j1=0.0 j2=20.0 j3=20.0 j4=0.0 fault=OK
```

Newer firmware may include optional readback and tool fields:

```text
STATUS state=idle homed=1 known=1 pose_source=mixed armed=1 hw=mixed enabled=1100 enc=1100 e1=12.5 e2=-4.2 j1=12.4 j2=-4.1 j3=20.0 j4=0.0 closed_loop=readback tool_type=generic tool=open tool_value=0.000 fault=OK
```

Error:

```text
ERR code=LIMIT message=joint_2_target_out_of_range
```

Acknowledgement:

```text
OK command=MOVEJ
OK command=JOGJ
OK command=JOGV
OK command=JOG_STOP
OK command=TRAJ_BEGIN count=3 duration=1.000
OK command=TRAJ_POINT index=0
OK command=TRAJ_START count=3 duration=1.000
OK command=TRAJ_CLEAR
OK command=CONFIG axes=4 hw=mixed enabled=1000
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
6. Add AS5048A encoder feedback as readback/known-pose verification before attempting closed-loop correction.
7. Refine synchronized low-level motion after hardware tests if the current open-loop trajectory follower is not smooth enough.

## Safety Requirements For Real Firmware

- `ESTOP` must immediately stop commanded motion and block new movement.
- `STOP` must cancel active motion without requiring a reset.
- Joint targets must be checked against configured limits.
- Unknown commands must not move hardware.
- Motion should not start until a known state or homed state exists.
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
