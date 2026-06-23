from __future__ import annotations

from copy import deepcopy
from math import isfinite
from time import time
from typing import Any

from .config import RobotConfig


ENCODER_SCHEMA_VERSION = 2
SUPPORTED_MOUNTINGS = {"joint_output", "gearbox_input", "motor_shaft"}
SUPPORTED_POLICIES = {"diagnostic", "warning", "fault"}
ESP32_S3_VALID_GPIOS = set(range(0, 22)) | set(range(26, 49))


def default_encoder_settings() -> dict[str, Any]:
    return {
        "schema_version": ENCODER_SCHEMA_VERSION,
        "enabled": False,
        "mode": "diagnostic",
        "bus": {
            "type": "spi",
            "sck_pin": 12,
            "miso_pin": 13,
            "mosi_pin": 14,
            "clock_hz": 1_000_000,
            "sample_interval_ms": 100,
        },
        "verification": {
            "policy": "diagnostic",
            "settle_delay_ms": 300,
            "required_stable_samples": 3,
            "warning_tolerance_deg": 2.0,
            "fault_tolerance_deg": 5.0,
            "hysteresis_deg": 0.25,
            "require_encoder": False,
        },
        "correction": {
            "enabled": False,
            "validation_id": "",
            "deadband_deg": 0.75,
            "max_delta_deg": 8.0,
            "joint_limit_margin_deg": 2.0,
            "speed_deg_s": 2.0,
            "accel_deg_s2": 10.0,
            "max_attempts": 2,
            "allowed_sources": [
                "set_joint_target",
                "set_all_joint_targets",
                "home",
                "encoder_shoulder_align",
            ],
        },
        "axes": [
            {
                "joint": 2,
                "name": "shoulder",
                "sensor": "as5048a",
                "enabled": False,
                "cs_pin": 15,
                "sensor_units": "count14",
                "angle_units": "deg",
                "reference_raw_deg": 0.0,
                "reference_joint_deg": 0.0,
                "direction_sign": 1,
                "wrap_period_deg": 360.0,
                "unwrap_policy": "shortest_reference",
                "mounting_location": "joint_output",
                "sensor_turns_per_joint_turn": 1.0,
                "calibration_model": "linear",
                "calibration_map": [],
                "calibration_map_extrapolate_deg": 2.0,
                "reference_description": "",
                "freshness_timeout_ms": 500,
                "max_noise_deg": 0.5,
                "calibration_validated": False,
                "calibration_id": "",
                "calibration_validated_at": None,
            }
        ],
    }


def _merge_mapping(target: dict[str, Any], patch: dict[str, Any]) -> None:
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge_mapping(target[key], value)
        else:
            target[key] = deepcopy(value)


def normalize_encoder_settings(config_or_raw: RobotConfig | dict[str, Any] | None) -> dict[str, Any]:
    if isinstance(config_or_raw, RobotConfig):
        raw = config_or_raw.raw.get("encoders")
    else:
        raw = config_or_raw
    raw = raw if isinstance(raw, dict) else {}

    settings = default_encoder_settings()
    legacy = int(raw.get("schema_version", 1) or 1) < ENCODER_SCHEMA_VERSION
    _merge_mapping(settings, {key: value for key, value in raw.items() if key != "axes"})

    raw_axes = raw.get("axes")
    if isinstance(raw_axes, list):
        shoulder_patch = next(
            (
                axis
                for axis in raw_axes
                if isinstance(axis, dict) and int(axis.get("joint", 0) or 0) == 2
            ),
            None,
        )
        if shoulder_patch:
            axis = deepcopy(default_encoder_settings()["axes"][0])
            _merge_mapping(axis, shoulder_patch)
            axis["joint"] = 2
            axis["name"] = "shoulder"
            settings["axes"] = [axis]

    settings["schema_version"] = ENCODER_SCHEMA_VERSION
    settings["mode"] = str(settings.get("mode") or settings.get("closed_loop_mode") or "diagnostic")
    if settings["mode"] not in {"diagnostic", "verification", "bounded_correction"}:
        settings["mode"] = "diagnostic"

    verification = settings.setdefault("verification", {})
    if raw.get("fault_tolerance_deg") is not None:
        verification["fault_tolerance_deg"] = raw["fault_tolerance_deg"]
    if raw.get("settle_tolerance_deg") is not None:
        verification["warning_tolerance_deg"] = raw["settle_tolerance_deg"]

    correction = settings.setdefault("correction", {})
    if raw.get("max_correction_attempts") is not None:
        correction["max_attempts"] = raw["max_correction_attempts"]
    default_sources = default_encoder_settings()["correction"]["allowed_sources"]
    if not isinstance(correction.get("allowed_sources"), list):
        correction["allowed_sources"] = list(default_sources)
    else:
        existing_sources = {str(value) for value in correction.get("allowed_sources", [])}
        for source in default_sources:
            if source not in existing_sources:
                correction["allowed_sources"].append(source)
                existing_sources.add(source)
    correction_validated = bool(
        not legacy
        and correction.get("enabled")
        and str(correction.get("validation_id") or "").strip()
    )
    shoulder_axis = encoder_axis(settings)
    encoder_disabled = not bool(settings.get("enabled")) or not bool(
        shoulder_axis and shoulder_axis.get("enabled")
    )
    if legacy or not correction_validated or encoder_disabled:
        correction["enabled"] = False
        if settings["mode"] == "bounded_correction" or raw.get("closed_loop_mode") == "settle_correction":
            settings["mode"] = "diagnostic"

    for obsolete in [
        "closed_loop_mode",
        "settle_tolerance_deg",
        "fault_tolerance_deg",
        "max_correction_attempts",
    ]:
        settings.pop(obsolete, None)
    return settings


def encoder_axis(settings: dict[str, Any], joint_number: int = 2) -> dict[str, Any] | None:
    for axis in settings.get("axes", []):
        if isinstance(axis, dict) and int(axis.get("joint", 0) or 0) == joint_number:
            return axis
    return None


def valid_esp32_s3_gpio(pin: int) -> bool:
    return pin in ESP32_S3_VALID_GPIOS


def wrapped_delta_deg(value_deg: float, reference_deg: float, period_deg: float = 360.0) -> float:
    period = float(period_deg)
    if period <= 0:
        raise ValueError("encoder wrap period must be positive")
    half = period / 2.0
    return (float(value_deg) - float(reference_deg) + half) % period - half


def calibrated_joint_angle(raw_deg: float, axis: dict[str, Any]) -> float:
    model = str(axis.get("calibration_model") or "linear")
    calibration_map = axis.get("calibration_map")
    if model == "piecewise_linear" and isinstance(calibration_map, list) and len(calibration_map) >= 2:
        period = float(axis.get("wrap_period_deg", 360.0))
        reference_raw = float(axis.get("reference_raw_deg", 0.0))
        raw_delta = wrapped_delta_deg(float(raw_deg), reference_raw, period)
        points: list[tuple[float, float]] = []
        for entry in calibration_map:
            if not isinstance(entry, dict):
                continue
            try:
                points.append((float(entry["raw_delta_deg"]), float(entry["joint_deg"])))
            except (TypeError, ValueError, KeyError):
                continue
        points.sort(key=lambda item: item[0])
        if len(points) >= 2:
            min_delta = points[0][0]
            max_delta = points[-1][0]
            extrapolate_limit = max(0.0, float(axis.get("calibration_map_extrapolate_deg", 2.0)))
            if raw_delta < min_delta - extrapolate_limit or raw_delta > max_delta + extrapolate_limit:
                raise ValueError("raw encoder angle is outside the calibrated map range")
            if raw_delta <= min_delta:
                low, high = points[0], points[1]
            elif raw_delta >= max_delta:
                low, high = points[-2], points[-1]
            else:
                low, high = points[0], points[-1]
                for first, second in zip(points, points[1:], strict=False):
                    if first[0] <= raw_delta <= second[0]:
                        low, high = first, second
                        break
            span = high[0] - low[0]
            if abs(span) < 1e-9:
                raise ValueError("calibrated map contains duplicate raw points")
            fraction = (raw_delta - low[0]) / span
            return low[1] + fraction * (high[1] - low[1])

    turns = float(axis.get("sensor_turns_per_joint_turn", 1.0))
    if turns <= 0:
        raise ValueError("sensor_turns_per_joint_turn must be positive")
    sign = -1 if int(axis.get("direction_sign", 1)) < 0 else 1
    delta = wrapped_delta_deg(
        float(raw_deg),
        float(axis.get("reference_raw_deg", 0.0)),
        float(axis.get("wrap_period_deg", 360.0)),
    )
    return float(axis.get("reference_joint_deg", 0.0)) + sign * delta / turns


def validate_encoder_settings(config: RobotConfig, settings: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    raw_settings = config.raw.get("encoders")
    raw_axes = raw_settings.get("axes") if isinstance(raw_settings, dict) else None
    if isinstance(raw_axes, list):
        unsupported_enabled = [
            int(axis.get("joint", 0) or 0)
            for axis in raw_axes
            if isinstance(axis, dict)
            and bool(axis.get("enabled"))
            and int(axis.get("joint", 0) or 0) != 2
        ]
        if unsupported_enabled:
            errors.append(
                "only the shoulder encoder may be enabled in this integration; "
                f"disable encoder joints {unsupported_enabled}"
            )
    if int(settings.get("schema_version", 0) or 0) != ENCODER_SCHEMA_VERSION:
        errors.append(f"encoders.schema_version must be {ENCODER_SCHEMA_VERSION}")

    bus = settings.get("bus") if isinstance(settings.get("bus"), dict) else {}
    for field in ["sck_pin", "miso_pin", "mosi_pin"]:
        try:
            pin = int(bus.get(field, -1))
        except (TypeError, ValueError):
            errors.append(f"encoders.bus.{field} must be an integer GPIO")
            continue
        if bool(settings.get("enabled")) and not valid_esp32_s3_gpio(pin):
            errors.append(
                f"encoders.bus.{field} must be a valid ESP32-S3 GPIO when encoders are enabled"
            )
    for field in ["clock_hz", "sample_interval_ms"]:
        try:
            if float(bus.get(field, 0)) <= 0:
                errors.append(f"encoders.bus.{field} must be positive")
        except (TypeError, ValueError):
            errors.append(f"encoders.bus.{field} must be numeric")

    axes = settings.get("axes")
    if not isinstance(axes, list) or not axes:
        errors.append("encoders.axes must define the shoulder encoder")
        return errors
    if len(axes) != 1:
        errors.append("encoders.axes must contain only the shoulder encoder in this integration")
    for index, axis in enumerate(axes):
        if not isinstance(axis, dict):
            errors.append(f"encoders.axes[{index}] must be a mapping")
            continue
        prefix = f"encoders.axes[{index}]"
        joint = int(axis.get("joint", 0) or 0)
        if joint != 2:
            errors.append(f"{prefix}.joint must be 2 for the shoulder encoder")
            continue
        if str(axis.get("sensor", "as5048a")) != "as5048a":
            errors.append(f"{prefix}.sensor must be as5048a")
        if int(axis.get("direction_sign", 0) or 0) not in {-1, 1}:
            errors.append(f"{prefix}.direction_sign must be -1 or 1")
        model = str(axis.get("calibration_model") or "linear")
        if model not in {"single_point", "linear", "linear_with_backlash", "piecewise_linear"}:
            errors.append(
                f"{prefix}.calibration_model must be single_point, linear, linear_with_backlash, or piecewise_linear"
            )
        mounting = str(axis.get("mounting_location") or "")
        if mounting not in SUPPORTED_MOUNTINGS:
            errors.append(f"{prefix}.mounting_location must be one of {sorted(SUPPORTED_MOUNTINGS)}")
        try:
            turns = float(axis.get("sensor_turns_per_joint_turn", 0.0))
            period = float(axis.get("wrap_period_deg", 0.0))
            noise = float(axis.get("max_noise_deg", -1.0))
            freshness = float(axis.get("freshness_timeout_ms", 0.0))
            reference_raw = float(axis.get("reference_raw_deg", 0.0))
            reference_joint = float(axis.get("reference_joint_deg", 0.0))
            map_extrapolate = float(axis.get("calibration_map_extrapolate_deg", 2.0))
            if not all(
                isfinite(value)
                for value in [
                    turns,
                    period,
                    noise,
                    freshness,
                    reference_raw,
                    reference_joint,
                    map_extrapolate,
                ]
            ):
                raise ValueError
        except (TypeError, ValueError):
            errors.append(f"{prefix} calibration and health values must be finite numbers")
            continue
        if turns <= 0:
            errors.append(f"{prefix}.sensor_turns_per_joint_turn must be positive")
        if period <= 0:
            errors.append(f"{prefix}.wrap_period_deg must be positive")
        if noise < 0:
            errors.append(f"{prefix}.max_noise_deg must be non-negative")
        if freshness <= 0:
            errors.append(f"{prefix}.freshness_timeout_ms must be positive")
        if map_extrapolate < 0:
            errors.append(f"{prefix}.calibration_map_extrapolate_deg must be non-negative")
        if not config.joints[1].min_deg <= reference_joint <= config.joints[1].max_deg:
            errors.append(f"{prefix}.reference_joint_deg is outside shoulder joint limits")
        calibration_map = axis.get("calibration_map")
        if model == "piecewise_linear":
            if not isinstance(calibration_map, list) or len(calibration_map) < 2:
                errors.append(f"{prefix}.calibration_map must contain at least two points for piecewise_linear")
            else:
                previous_raw_delta: float | None = None
                for map_index, entry in enumerate(calibration_map):
                    if not isinstance(entry, dict):
                        errors.append(f"{prefix}.calibration_map[{map_index}] must be a mapping")
                        continue
                    try:
                        raw_delta = float(entry.get("raw_delta_deg"))
                        joint_deg = float(entry.get("joint_deg"))
                    except (TypeError, ValueError):
                        errors.append(f"{prefix}.calibration_map[{map_index}] values must be numeric")
                        continue
                    if not isfinite(raw_delta) or not isfinite(joint_deg):
                        errors.append(f"{prefix}.calibration_map[{map_index}] values must be finite")
                    if previous_raw_delta is not None and raw_delta <= previous_raw_delta:
                        errors.append(f"{prefix}.calibration_map raw_delta_deg values must be strictly increasing")
                    previous_raw_delta = raw_delta
                    if not config.joints[1].min_deg <= joint_deg <= config.joints[1].max_deg:
                        errors.append(f"{prefix}.calibration_map[{map_index}].joint_deg is outside shoulder limits")
        if bool(axis.get("enabled")):
            try:
                cs_pin = int(axis.get("cs_pin", -1))
            except (TypeError, ValueError):
                cs_pin = -1
            if not valid_esp32_s3_gpio(cs_pin):
                errors.append(f"{prefix}.cs_pin must be a valid ESP32-S3 GPIO when enabled")

    verification = settings.get("verification") if isinstance(settings.get("verification"), dict) else {}
    policy = str(verification.get("policy") or "diagnostic")
    if policy not in SUPPORTED_POLICIES:
        errors.append(f"encoders.verification.policy must be one of {sorted(SUPPORTED_POLICIES)}")
    try:
        warning = float(verification.get("warning_tolerance_deg", 0.0))
        fault = float(verification.get("fault_tolerance_deg", 0.0))
        settle_delay = float(verification.get("settle_delay_ms", -1.0))
        hysteresis = float(verification.get("hysteresis_deg", -1.0))
        required = int(verification.get("required_stable_samples", 0))
        if warning <= 0 or fault <= warning:
            errors.append("encoder fault tolerance must be greater than the positive warning tolerance")
        if settle_delay < 0:
            errors.append("encoders.verification.settle_delay_ms must be non-negative")
        if hysteresis < 0 or hysteresis >= warning:
            errors.append("encoder hysteresis must be non-negative and below the warning tolerance")
        if required < 1:
            errors.append("encoders.verification.required_stable_samples must be at least 1")
    except (TypeError, ValueError):
        errors.append("encoder verification thresholds must be numeric")

    correction = settings.get("correction") if isinstance(settings.get("correction"), dict) else {}
    correction_enabled = bool(correction.get("enabled"))
    try:
        deadband = float(correction.get("deadband_deg", 0.75))
        max_delta = float(correction.get("max_delta_deg", 0.0))
        limit_margin = float(correction.get("joint_limit_margin_deg", -1.0))
        speed = float(correction.get("speed_deg_s", 0.0))
        accel = float(correction.get("accel_deg_s2", 0.0))
        attempts = int(correction.get("max_attempts", 0))
        if correction_enabled and (max_delta <= 0 or speed <= 0 or accel <= 0):
            errors.append("encoder correction limits, speed, and acceleration must be positive")
        if correction_enabled and deadband < 0:
            errors.append("encoders.correction.deadband_deg must be non-negative")
        if correction_enabled and max_delta <= deadband:
            errors.append("encoder correction max delta must be greater than the correction deadband")
        if correction_enabled and limit_margin < 0:
            errors.append("encoders.correction.joint_limit_margin_deg must be non-negative")
        if correction_enabled and attempts < 1:
            errors.append("encoders.correction.max_attempts must be at least 1")
    except (TypeError, ValueError):
        if correction_enabled:
            errors.append("encoder correction limits must be numeric")
    if correction_enabled:
        axis = encoder_axis(settings)
        if not bool(settings.get("enabled")):
            errors.append("encoder correction requires the encoder bus to be enabled")
        if not axis or not bool(axis.get("enabled")):
            errors.append("encoder correction requires the shoulder encoder to be enabled")
        if not axis or str(axis.get("mounting_location")) != "joint_output":
            errors.append("encoder correction requires joint_output mounting")
        if not axis or not bool(axis.get("calibration_validated")):
            errors.append("encoder correction requires validated shoulder calibration")
        if not str(correction.get("validation_id") or "").strip():
            errors.append("encoder correction requires a validation_id")
        if config.joints[1].actuator != "stepper" or not (
            config.joints[1].hardware.stepper
            and config.joints[1].hardware.stepper.enabled
        ):
            errors.append("encoder correction requires an enabled hardware shoulder stepper")
    return errors


def empty_evidence(joint_number: int, name: str) -> dict[str, Any]:
    return {
        "joint": joint_number,
        "name": name,
        "source": "none",
        "raw_count": None,
        "raw_angle_deg": None,
        "measured_angle_deg": None,
        "valid": False,
        "fresh": False,
        "age_ms": None,
        "noise_deg": None,
        "health": "unavailable",
        "flags": [],
        "mismatch_deg": None,
        "updated_at": None,
    }


def evidence_from_status(
    *,
    joint_number: int,
    name: str,
    raw_count: int | None,
    raw_angle_deg: float | None,
    measured_angle_deg: float | None,
    valid: bool,
    age_ms: int | None,
    noise_deg: float | None,
    flags: list[str],
    freshness_timeout_ms: float,
    estimated_angle_deg: float,
) -> dict[str, Any]:
    age = None if age_ms is None else max(0, int(age_ms))
    fresh = bool(valid and age is not None and age <= float(freshness_timeout_ms))
    health = "valid" if fresh else ("stale" if valid else ("invalid" if flags else "unavailable"))
    mismatch = (
        float(measured_angle_deg) - float(estimated_angle_deg)
        if fresh and measured_angle_deg is not None
        else None
    )
    return {
        "joint": joint_number,
        "name": name,
        "source": "encoder" if raw_angle_deg is not None else "none",
        "raw_count": raw_count,
        "raw_angle_deg": raw_angle_deg,
        "measured_angle_deg": measured_angle_deg,
        "valid": bool(valid),
        "fresh": fresh,
        "age_ms": age,
        "noise_deg": noise_deg,
        "health": health,
        "flags": list(flags),
        "mismatch_deg": mismatch,
        "updated_at": time(),
    }
