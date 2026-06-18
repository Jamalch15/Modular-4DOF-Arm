from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
EXAMPLE_CONFIG_PATH = CONFIG_DIR / "robot.example.yaml"
LOCAL_CONFIG_PATH = CONFIG_DIR / "robot.local.yaml"
CONFIG_PATH = EXAMPLE_CONFIG_PATH

MATLAB_PROTOTYPE_GEOMETRY: dict[str, Any] = {
    "label": "MATLAB prototype",
    "status": "working_assumption",
    "source": "jacobian_ik_robotarm_analytic_seed.m",
    "units": {"length": "mm", "angle": "deg"},
    "dimensions_mm": {
        "L_1": 93.45,
        "L_2": 23.20,
        "L_3": 64.50,
        "L_4": 42.69,
        "L_5": 160.15,
        "L_6": 41.39,
        "L_7": 142.55,
        "L_8": 49.20,
        "L_9": 15.00,
    },
    "signs": {"s4": -1, "s6": -1, "s8": 1},
    "joint_limits_deg": {
        "theta1": {"min": -180.0, "max": 180.0},
        "theta2": {"min": -90.0, "max": 160.0},
        "theta3": {"min": -160.0, "max": 160.0},
        "theta4": {"min": -180.0, "max": 180.0},
    },
    "starting_pose_deg": [0.0, 0.0, -70.0, -20.0],
}

DEFAULT_GEOMETRY_CONFIG: dict[str, Any] = {
    "active_preset": "matlab_prototype",
    "presets": {"matlab_prototype": MATLAB_PROTOTYPE_GEOMETRY},
}


@dataclass(frozen=True)
class StepperHardwareConfig:
    enabled: bool
    step_pin: int
    dir_pin: int
    enable_pin: int
    enable_active_low: bool
    m0_pin: int
    m1_pin: int
    m2_pin: int
    driver_model: str
    motor_full_steps_per_rev: int
    microsteps: int
    gear_ratio: float


@dataclass(frozen=True)
class ServoHardwareConfig:
    enabled: bool
    pwm_pin: int
    pulse_min_us: int
    pulse_max_us: int
    pwm_frequency_hz: int
    servo_range_deg: float
    neutral_deg: float
    gear_ratio: float


@dataclass(frozen=True)
class JointHardwareConfig:
    stepper: StepperHardwareConfig | None = None
    servo: ServoHardwareConfig | None = None


@dataclass(frozen=True)
class JointConfig:
    name: str
    actuator: str
    min_deg: float
    max_deg: float
    home_deg: float
    max_speed_deg_s: float
    max_accel_deg_s2: float
    zero_offset_deg: float
    direction_sign: int
    hardware: JointHardwareConfig


@dataclass(frozen=True)
class DHRowConfig:
    joint_index: int
    theta_offset_deg: float
    d_mm: float
    a_mm: float
    alpha_deg: float
    joint_type: str = "revolute"
    min_deg: float = -180.0
    max_deg: float = 180.0
    zero_offset_deg: float = 0.0
    direction_sign: int = 1


@dataclass(frozen=True)
class LinkConfig:
    base_height_mm: float
    upper_arm_mm: float
    forearm_mm: float
    wrist_mm: float
    tool_mm: float
    base_side_offset_mm: float = 0.0
    dh_rows: list[DHRowConfig] = field(default_factory=list)
    tool_tcp_offset_mm: dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0})


@dataclass(frozen=True)
class KinematicsConfig:
    convention: str
    dh_rows: list[DHRowConfig]
    position_tolerance_mm: float
    orientation_tolerance_deg: float
    max_iterations: int
    damping: float


@dataclass(frozen=True)
class MotionConfig:
    update_rate_hz: float
    smoothing_alpha: float
    command_rate_limit_hz: float
    acceleration_deg_s2: float
    allow_sudden_jumps: bool


@dataclass(frozen=True)
class SerialConfig:
    port: str
    baud_rate: int
    timeout_s: float


@dataclass(frozen=True)
class RobotConfig:
    joints: list[JointConfig]
    links: LinkConfig
    kinematics: KinematicsConfig
    motion: MotionConfig
    serial: SerialConfig
    simulation_default: bool
    coordinate_frame_notes: str
    raw: dict[str, Any]
    source_path: Path

    @property
    def joint_names(self) -> list[str]:
        return [joint.name for joint in self.joints]

    @property
    def home_pose(self) -> list[float]:
        return [joint.home_deg for joint in self.joints]


def _require_number(value: Any, name: str) -> float:
    if not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    return float(value)


def _direction_sign(value: Any, name: str) -> int:
    if value in {-1, "-1", "reverse", "REV", "rev"}:
        return -1
    if value in {1, "1", "forward", "FWD", "fwd"}:
        return 1
    raise ValueError(f"{name} must be 1 or -1")


def _int_value(value: Any, name: str) -> int:
    if value is None or value in {"", "TBD", "tbd", "unknown", "UNKNOWN"}:
        return -1
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be an integer")
    return int(value)


def _positive_int(value: Any, name: str) -> int:
    result = _int_value(value, name)
    if result <= 0:
        raise ValueError(f"{name} must be positive")
    return result


def _default_dh_rows(
    base_height_mm: float,
    upper_arm_mm: float,
    forearm_mm: float,
    wrist_mm: float,
    tool_mm: float,
) -> list[DHRowConfig]:
    # Standard DH table matching the project coordinate convention:
    # robot x = DH y, robot y = -DH x, robot z = DH z.
    return [
        DHRowConfig(0, 0.0, base_height_mm, 0.0, 90.0),
        DHRowConfig(1, 90.0, 0.0, upper_arm_mm, 0.0),
        DHRowConfig(2, 0.0, 0.0, forearm_mm, 0.0),
        DHRowConfig(3, 0.0, 0.0, wrist_mm + tool_mm, 0.0),
    ]


def matlab_geometry_to_dh_rows(preset: dict[str, Any] | None = None) -> list[DHRowConfig]:
    """Build the Standard DH rows used by the measured prototype geometry."""
    source = deepcopy(preset or MATLAB_PROTOTYPE_GEOMETRY)
    dimensions = source.get("dimensions_mm", {})
    signs = source.get("signs", {})
    limits = source.get("joint_limits_deg", {})

    def length(name: str) -> float:
        return _require_number(dimensions.get(name), f"geometry.dimensions_mm.{name}")

    def sign(name: str) -> int:
        return _direction_sign(signs.get(name), f"geometry.signs.{name}")

    def limit(index: int, side: str, fallback: float) -> float:
        row_limits = limits.get(f"theta{index}", {})
        if not isinstance(row_limits, dict):
            return fallback
        return _require_number(row_limits.get(side, fallback), f"geometry.joint_limits_deg.theta{index}.{side}")

    return [
        DHRowConfig(
            joint_index=0,
            theta_offset_deg=0.0,
            d_mm=length("L_1") + length("L_3"),
            a_mm=0.0,
            alpha_deg=90.0,
            min_deg=limit(1, "min", -180.0),
            max_deg=limit(1, "max", 180.0),
        ),
        DHRowConfig(
            joint_index=1,
            theta_offset_deg=0.0,
            d_mm=sign("s4") * length("L_4"),
            a_mm=length("L_5"),
            alpha_deg=0.0,
            min_deg=limit(2, "min", -90.0),
            max_deg=limit(2, "max", 160.0),
        ),
        DHRowConfig(
            joint_index=2,
            theta_offset_deg=0.0,
            d_mm=sign("s6") * length("L_6"),
            a_mm=length("L_7"),
            alpha_deg=0.0,
            min_deg=limit(3, "min", -160.0),
            max_deg=limit(3, "max", 160.0),
        ),
        DHRowConfig(
            joint_index=3,
            theta_offset_deg=0.0,
            d_mm=sign("s8") * length("L_8"),
            a_mm=length("L_9"),
            alpha_deg=0.0,
            min_deg=limit(4, "min", -180.0),
            max_deg=limit(4, "max", 180.0),
        ),
    ]


def _parse_dh_rows(raw_rows: Any, fallback: list[DHRowConfig]) -> list[DHRowConfig]:
    if not isinstance(raw_rows, list) or not raw_rows:
        return fallback
    rows: list[DHRowConfig] = []
    for index, row in enumerate(raw_rows):
        if not isinstance(row, dict):
            raise ValueError(f"kinematics.dh_rows[{index}] must be a mapping")
        if "joint_index" in row:
            joint_index = int(row["joint_index"])
        else:
            joint_index = int(row.get("joint", index + 1)) - 1
        if joint_index < 0 or joint_index > 3:
            raise ValueError(f"kinematics.dh_rows[{index}] must reference joint_index 0..3 or joint 1..4")
        rows.append(
            DHRowConfig(
                joint_index=joint_index,
                theta_offset_deg=_require_number(
                    row.get("theta_offset_deg", row.get("theta", 0.0)),
                    f"kinematics.dh_rows[{index}].theta_offset_deg",
                ),
                d_mm=_require_number(row.get("d_mm", row.get("d", 0.0)), f"kinematics.dh_rows[{index}].d_mm"),
                a_mm=_require_number(row.get("a_mm", row.get("a", 0.0)), f"kinematics.dh_rows[{index}].a_mm"),
                alpha_deg=_require_number(
                    row.get("alpha_deg", row.get("alpha", 0.0)),
                    f"kinematics.dh_rows[{index}].alpha_deg",
                ),
                joint_type=str(row.get("joint_type", "revolute")),
                min_deg=_require_number(row.get("min_deg", -180.0), f"kinematics.dh_rows[{index}].min_deg"),
                max_deg=_require_number(row.get("max_deg", 180.0), f"kinematics.dh_rows[{index}].max_deg"),
                zero_offset_deg=_require_number(
                    row.get("zero_offset_deg", 0.0),
                    f"kinematics.dh_rows[{index}].zero_offset_deg",
                ),
                direction_sign=_direction_sign(
                    row.get("direction_sign", 1),
                    f"kinematics.dh_rows[{index}].direction_sign",
                ),
            )
        )
    if len(rows) != 4:
        raise ValueError("kinematics.dh_rows must define exactly four rows")
    return rows


def _active_tool_tcp_offset(raw_config: dict[str, Any]) -> dict[str, float]:
    tools_raw = raw_config.get("tools")
    active = "gripper"
    preset: dict[str, Any] = {}
    if isinstance(tools_raw, dict):
        active = str(tools_raw.get("active", active))
        presets = tools_raw.get("presets")
        if isinstance(presets, dict) and isinstance(presets.get(active), dict):
            preset = presets[active]
    legacy_tool = raw_config.get("tool")
    if not preset and isinstance(legacy_tool, dict):
        preset = legacy_tool
    offset = preset.get("tcp_offset_mm") if isinstance(preset, dict) else None
    if not isinstance(offset, dict):
        offset = {}
    return {
        "x": _require_number(offset.get("x", 0.0), "tools.active.tcp_offset_mm.x"),
        "y": _require_number(offset.get("y", 0.0), "tools.active.tcp_offset_mm.y"),
        "z": _require_number(offset.get("z", 0.0), "tools.active.tcp_offset_mm.z"),
    }


def _hardware_for_joint(item: dict[str, Any], actuator: str, name: str) -> JointHardwareConfig:
    hardware_raw = item.get("hardware")
    if not isinstance(hardware_raw, dict):
        placeholder = item.get("calibration_placeholder", {})
        if actuator == "servo":
            hardware_raw = {
                "servo": {
                    "enabled": False,
                    "pwm_pin": placeholder.get("servo_pin", -1),
                }
            }
        else:
            hardware_raw = {
                "stepper": {
                    "enabled": False,
                    "step_pin": placeholder.get("step_pin", -1),
                    "dir_pin": placeholder.get("dir_pin", -1),
                    "enable_pin": placeholder.get("enable_pin", -1),
                }
            }

    stepper: StepperHardwareConfig | None = None
    servo: ServoHardwareConfig | None = None
    if actuator == "stepper":
        raw = hardware_raw.get("stepper", {}) if isinstance(hardware_raw.get("stepper", {}), dict) else {}
        stepper = StepperHardwareConfig(
            enabled=bool(raw.get("enabled", False)),
            step_pin=_int_value(raw.get("step_pin", -1), f"{name}.hardware.stepper.step_pin"),
            dir_pin=_int_value(raw.get("dir_pin", -1), f"{name}.hardware.stepper.dir_pin"),
            enable_pin=_int_value(raw.get("enable_pin", -1), f"{name}.hardware.stepper.enable_pin"),
            enable_active_low=bool(raw.get("enable_active_low", True)),
            m0_pin=_int_value(raw.get("m0_pin", -1), f"{name}.hardware.stepper.m0_pin"),
            m1_pin=_int_value(raw.get("m1_pin", -1), f"{name}.hardware.stepper.m1_pin"),
            m2_pin=_int_value(raw.get("m2_pin", -1), f"{name}.hardware.stepper.m2_pin"),
            driver_model=str(raw.get("driver_model", "TB6600")),
            motor_full_steps_per_rev=_positive_int(
                raw.get("motor_full_steps_per_rev", 200),
                f"{name}.hardware.stepper.motor_full_steps_per_rev",
            ),
            microsteps=_positive_int(raw.get("microsteps", 16), f"{name}.hardware.stepper.microsteps"),
            gear_ratio=_require_number(raw.get("gear_ratio", 1.0), f"{name}.hardware.stepper.gear_ratio"),
        )
        if stepper.gear_ratio <= 0:
            raise ValueError(f"{name}.hardware.stepper.gear_ratio must be positive")
    elif actuator == "servo":
        raw = hardware_raw.get("servo", {}) if isinstance(hardware_raw.get("servo", {}), dict) else {}
        servo = ServoHardwareConfig(
            enabled=bool(raw.get("enabled", False)),
            pwm_pin=_int_value(raw.get("pwm_pin", -1), f"{name}.hardware.servo.pwm_pin"),
            pulse_min_us=_positive_int(raw.get("pulse_min_us", 500), f"{name}.hardware.servo.pulse_min_us"),
            pulse_max_us=_positive_int(raw.get("pulse_max_us", 2500), f"{name}.hardware.servo.pulse_max_us"),
            pwm_frequency_hz=_positive_int(
                raw.get("pwm_frequency_hz", 50), f"{name}.hardware.servo.pwm_frequency_hz"
            ),
            servo_range_deg=_require_number(raw.get("servo_range_deg", 270.0), f"{name}.hardware.servo.servo_range_deg"),
            neutral_deg=_require_number(raw.get("neutral_deg", 135.0), f"{name}.hardware.servo.neutral_deg"),
            gear_ratio=_require_number(raw.get("gear_ratio", 1.0), f"{name}.hardware.servo.gear_ratio"),
        )
        if servo.pulse_min_us >= servo.pulse_max_us:
            raise ValueError(f"{name}.hardware.servo.pulse_min_us must be below pulse_max_us")
        if servo.servo_range_deg <= 0:
            raise ValueError(f"{name}.hardware.servo.servo_range_deg must be positive")
        if servo.gear_ratio <= 0:
            raise ValueError(f"{name}.hardware.servo.gear_ratio must be positive")
    return JointHardwareConfig(stepper=stepper, servo=servo)


def active_config_path() -> Path:
    return LOCAL_CONFIG_PATH if LOCAL_CONFIG_PATH.exists() else EXAMPLE_CONFIG_PATH


def ensure_local_config() -> Path:
    if not LOCAL_CONFIG_PATH.exists():
        LOCAL_CONFIG_PATH.write_text(EXAMPLE_CONFIG_PATH.read_text(encoding="utf-8"), encoding="utf-8")
    return LOCAL_CONFIG_PATH


def load_config(path: str | Path | None = None) -> RobotConfig:
    config_path = Path(path) if path is not None else active_config_path()
    with config_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    motion_raw = raw.get("motion", {})
    default_joint_accel = _require_number(
        motion_raw.get("acceleration_deg_s2", 120), "motion.acceleration_deg_s2"
    )

    joint_items = raw.get("joints", [])
    if len(joint_items) != 4:
        raise ValueError("config must define exactly four joints")

    joints: list[JointConfig] = []
    for index, item in enumerate(joint_items):
        limits = item.get("limits_deg", {})
        name = item.get("name", f"joint_{index + 1}")
        min_deg = _require_number(limits.get("min"), f"{name}.limits_deg.min")
        max_deg = _require_number(limits.get("max"), f"{name}.limits_deg.max")
        home_deg = _require_number(item.get("home_deg", 0.0), f"{name}.home_deg")
        max_speed = _require_number(item.get("max_speed_deg_s", 20.0), f"{name}.max_speed_deg_s")
        max_accel = _require_number(
            item.get("max_accel_deg_s2", default_joint_accel), f"{name}.max_accel_deg_s2"
        )
        zero_offset = _require_number(item.get("zero_offset_deg", 0.0), f"{name}.zero_offset_deg")
        direction = _direction_sign(item.get("direction_sign", 1), f"{name}.direction_sign")
        if min_deg >= max_deg:
            raise ValueError(f"{name} min limit must be below max limit")
        if not min_deg <= home_deg <= max_deg:
            raise ValueError(f"{name} home pose is outside joint limits")
        if max_speed <= 0:
            raise ValueError(f"{name} max_speed_deg_s must be positive")
        if max_accel <= 0:
            raise ValueError(f"{name} max_accel_deg_s2 must be positive")
        actuator = str(item.get("actuator", "unknown"))
        joints.append(
            JointConfig(
                name=str(name),
                actuator=actuator,
                min_deg=min_deg,
                max_deg=max_deg,
                home_deg=home_deg,
                max_speed_deg_s=max_speed,
                max_accel_deg_s2=max_accel,
                zero_offset_deg=zero_offset,
                direction_sign=direction,
                hardware=_hardware_for_joint(item, actuator, str(name)),
            )
        )

    geometry_raw = raw.get("geometry", {})
    active_geometry = {}
    if isinstance(geometry_raw, dict):
        presets = geometry_raw.get("presets", {})
        active_name = geometry_raw.get("active_preset")
        if isinstance(presets, dict) and active_name in presets and isinstance(presets[active_name], dict):
            active_geometry = presets[active_name]
    active_dimensions = active_geometry.get("dimensions_mm", {}) if isinstance(active_geometry, dict) else {}

    links_raw = raw.get("links_mm", {})
    base_height = _require_number(links_raw.get("base_height", 80.0), "base_height")
    upper_arm = _require_number(links_raw.get("upper_arm", 140.0), "upper_arm")
    forearm = _require_number(links_raw.get("forearm", 120.0), "forearm")
    wrist = _require_number(links_raw.get("wrist", 60.0), "wrist")
    tool = _require_number(links_raw.get("tool", 30.0), "tool")
    base_side_offset = _require_number(
        links_raw.get("base_side_offset", active_dimensions.get("L_2", 0.0)),
        "base_side_offset",
    )
    fallback_dh = _default_dh_rows(base_height, upper_arm, forearm, wrist, tool)

    kinematics_raw = raw.get("kinematics", {})
    if not isinstance(kinematics_raw, dict):
        kinematics_raw = {}
    convention = str(kinematics_raw.get("convention", "standard_dh"))
    if convention != "standard_dh":
        raise ValueError("only standard_dh kinematics are supported")
    dh_rows = _parse_dh_rows(kinematics_raw.get("dh_rows"), fallback_dh)
    kinematics = KinematicsConfig(
        convention=convention,
        dh_rows=dh_rows,
        position_tolerance_mm=_require_number(
            kinematics_raw.get("position_tolerance_mm", 1.0),
            "kinematics.position_tolerance_mm",
        ),
        orientation_tolerance_deg=_require_number(
            kinematics_raw.get("orientation_tolerance_deg", 1.0),
            "kinematics.orientation_tolerance_deg",
        ),
        max_iterations=int(kinematics_raw.get("max_iterations", 160)),
        damping=_require_number(kinematics_raw.get("damping", 0.18), "kinematics.damping"),
    )
    if kinematics.max_iterations <= 0:
        raise ValueError("kinematics.max_iterations must be positive")
    if kinematics.damping <= 0:
        raise ValueError("kinematics.damping must be positive")
    links = LinkConfig(
        base_height,
        upper_arm,
        forearm,
        wrist,
        tool,
        base_side_offset,
        dh_rows=dh_rows,
        tool_tcp_offset_mm=_active_tool_tcp_offset(raw),
    )

    motion = MotionConfig(
        update_rate_hz=_require_number(motion_raw.get("update_rate_hz", 30), "update_rate_hz"),
        smoothing_alpha=_require_number(motion_raw.get("smoothing_alpha", 0.35), "smoothing_alpha"),
        command_rate_limit_hz=_require_number(
            motion_raw.get("command_rate_limit_hz", 12), "command_rate_limit_hz"
        ),
        acceleration_deg_s2=_require_number(motion_raw.get("acceleration_deg_s2", 120), "acceleration_deg_s2"),
        allow_sudden_jumps=bool(motion_raw.get("allow_sudden_jumps", False)),
    )
    if motion.update_rate_hz <= 0:
        raise ValueError("motion.update_rate_hz must be positive")
    if not 0 < motion.smoothing_alpha <= 1:
        raise ValueError("motion.smoothing_alpha must be in (0, 1]")
    if motion.command_rate_limit_hz <= 0:
        raise ValueError("motion.command_rate_limit_hz must be positive")

    serial_raw = raw.get("serial", {})
    serial = SerialConfig(
        port=str(serial_raw.get("last_port", serial_raw.get("port", "COM6"))),
        baud_rate=int(serial_raw.get("baud_rate", 115200)),
        timeout_s=_require_number(serial_raw.get("timeout_s", 0.2), "serial.timeout_s"),
    )

    return RobotConfig(
        joints=joints,
        links=links,
        kinematics=kinematics,
        motion=motion,
        serial=serial,
        simulation_default=bool(raw.get("simulation_default", True)),
        coordinate_frame_notes=str(raw.get("coordinate_frame_notes", "")),
        raw=raw,
        source_path=config_path,
    )


def _write_dh_rows(data: dict[str, Any], rows: list[dict[str, Any]]) -> None:
    data.setdefault("kinematics", {})
    data["kinematics"]["convention"] = "standard_dh"
    data["kinematics"]["dh_rows"] = [
        {
            "joint": int(row.get("joint", row.get("joint_index", index + 1))),
            "theta_offset_deg": float(row.get("theta_offset_deg", 0.0)),
            "d_mm": float(row.get("d_mm", 0.0)),
            "a_mm": float(row.get("a_mm", 0.0)),
            "alpha_deg": float(row.get("alpha_deg", 0.0)),
            "joint_type": str(row.get("joint_type", "revolute")),
            "min_deg": float(row.get("min_deg", -180.0)),
            "max_deg": float(row.get("max_deg", 180.0)),
            "zero_offset_deg": float(row.get("zero_offset_deg", 0.0)),
            "direction_sign": _direction_sign(row.get("direction_sign", 1), f"kinematics.dh_rows[{index}].direction_sign"),
        }
        for index, row in enumerate(rows)
    ]


def save_calibration_updates(path: str | Path, updates: dict[str, Any]) -> None:
    """Persist editable calibration values while keeping comments in the YAML file."""
    try:
        from ruamel.yaml import YAML
    except ImportError as exc:  # pragma: no cover - exercised only in misconfigured envs.
        raise RuntimeError("ruamel.yaml is required to preserve config comments") from exc

    config_path = Path(path)
    yaml_rt = YAML()
    yaml_rt.preserve_quotes = True
    yaml_rt.indent(mapping=2, sequence=4, offset=2)

    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml_rt.load(handle) or {}

    links = updates.get("links_mm")
    if isinstance(links, dict):
        data.setdefault("links_mm", {})
        for key in ["base_height", "upper_arm", "forearm", "wrist", "tool", "base_side_offset"]:
            if key in links:
                data["links_mm"][key] = float(links[key])

    kinematics = updates.get("kinematics")
    if isinstance(kinematics, dict):
        data.setdefault("kinematics", {})
        for key in ["position_tolerance_mm", "orientation_tolerance_deg", "damping"]:
            if key in kinematics:
                data["kinematics"][key] = float(kinematics[key])
        if "max_iterations" in kinematics:
            data["kinematics"]["max_iterations"] = int(kinematics["max_iterations"])
        if isinstance(kinematics.get("dh_rows"), list):
            _write_dh_rows(data, kinematics["dh_rows"])

    joints = updates.get("joints")
    home_pose_changed = False
    if isinstance(joints, list):
        data.setdefault("joints", [])
        for index, patch in enumerate(joints):
            if index >= len(data["joints"]) or not isinstance(patch, dict):
                continue
            joint = data["joints"][index]
            if "limits_deg" in patch and isinstance(patch["limits_deg"], dict):
                joint.setdefault("limits_deg", {})
                limits = patch["limits_deg"]
                if "min" in limits:
                    joint["limits_deg"]["min"] = float(limits["min"])
                if "max" in limits:
                    joint["limits_deg"]["max"] = float(limits["max"])
            for key in [
                "home_deg",
                "max_speed_deg_s",
                "max_accel_deg_s2",
                "zero_offset_deg",
            ]:
                if key in patch:
                    joint[key] = float(patch[key])
                    if key == "home_deg":
                        home_pose_changed = True
            if "direction_sign" in patch:
                joint["direction_sign"] = _direction_sign(
                    patch["direction_sign"], f"joints[{index}].direction_sign"
                )
            if "hardware" in patch and isinstance(patch["hardware"], dict):
                joint.setdefault("hardware", {})
                hardware = patch["hardware"]
                if "stepper" in hardware and isinstance(hardware["stepper"], dict):
                    stepper = joint["hardware"].setdefault("stepper", {})
                    for key in [
                        "enabled",
                        "step_pin",
                        "dir_pin",
                        "enable_pin",
                        "enable_active_low",
                        "driver_model",
                        "motor_full_steps_per_rev",
                        "microsteps",
                        "gear_ratio",
                    ]:
                        if key in hardware["stepper"]:
                            stepper[key] = hardware["stepper"][key]
                    for obsolete_key in ["m0_pin", "m1_pin", "m2_pin"]:
                        if obsolete_key in stepper:
                            stepper[obsolete_key] = -1
                if "servo" in hardware and isinstance(hardware["servo"], dict):
                    servo = joint["hardware"].setdefault("servo", {})
                    for key in [
                        "enabled",
                        "pwm_pin",
                        "pulse_min_us",
                        "pulse_max_us",
                        "pwm_frequency_hz",
                        "servo_range_deg",
                        "neutral_deg",
                        "gear_ratio",
                    ]:
                        if key in hardware["servo"]:
                            servo[key] = hardware["servo"][key]

    motion = updates.get("motion")
    if isinstance(motion, dict):
        data.setdefault("motion", {})
        for key in [
            "update_rate_hz",
            "smoothing_alpha",
            "command_rate_limit_hz",
            "acceleration_deg_s2",
        ]:
            if key in motion:
                data["motion"][key] = float(motion[key])
        if "allow_sudden_jumps" in motion:
            data["motion"]["allow_sudden_jumps"] = bool(motion["allow_sudden_jumps"])

    serial = updates.get("serial")
    if isinstance(serial, dict):
        data.setdefault("serial", {})
        if "last_port" in serial:
            data["serial"]["last_port"] = str(serial["last_port"])
            data["serial"]["port"] = str(serial["last_port"])
        if "baud_rate" in serial:
            data["serial"]["baud_rate"] = int(serial["baud_rate"])
        if "timeout_s" in serial:
            data["serial"]["timeout_s"] = float(serial["timeout_s"])

    for key in [
        "named_positions",
        "camera",
        "color_profiles",
        "drop_zones",
        "task_defaults",
        "path_defaults",
        "tool",
        "tools",
        "encoders",
        "calibration",
        "geometry",
    ]:
        if key in updates and isinstance(updates[key], dict):
            data[key] = updates[key]

    if home_pose_changed and len(data.get("joints", [])) == 4:
        data.setdefault("named_positions", {})
        data["named_positions"]["home"] = {
            "type": "joint",
            "angles_deg": [float(joint.get("home_deg", 0.0)) for joint in data["joints"]],
        }

    with config_path.open("w", encoding="utf-8") as handle:
        yaml_rt.dump(data, handle)
