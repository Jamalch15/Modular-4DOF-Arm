from __future__ import annotations

import json
import re
from copy import deepcopy
from datetime import datetime, timezone
from hashlib import sha256
from math import cos, sin, tau
from pathlib import Path
from typing import Any
from uuid import uuid4

from .config import RobotConfig
from .kinematics import forward_kinematics


PROGRAM_SCHEMA_VERSION = 1
PROGRAM_STORE_SCHEMA_VERSION = 1
CACHED_PLAN_SCHEMA_VERSION = 1
PROGRAM_STORE_PATH = Path(__file__).resolve().parents[1] / "config" / "programs.local.json"


class ProgramLibraryError(ValueError):
    """Raised when a saved program cannot be normalized or persisted."""


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ProgramLibraryError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ProgramLibraryError(f"{name} must be a finite number") from exc
    if not (-float("inf") < number < float("inf")):
        raise ProgramLibraryError(f"{name} must be a finite number")
    return number


def _slug(value: str) -> str:
    normalized = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
    return normalized or "program"


def new_program_id(name: str) -> str:
    return f"{_slug(name)}-{uuid4().hex[:8]}"


def program_motion_fingerprint(program: dict[str, Any]) -> str:
    payload = {
        "schema_version": int(program.get("schema_version", PROGRAM_SCHEMA_VERSION)),
        "required_tool": program.get("required_tool"),
        "steps": program.get("steps", []),
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return sha256(encoded).hexdigest()[:16]


def normalize_program_step(config: RobotConfig, raw: dict[str, Any], index: int) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise ProgramLibraryError(f"step {index + 1} must be an object")

    kind = str(raw.get("type") or "").strip().lower()
    if kind not in {"joint", "cartesian", "tool"}:
        raise ProgramLibraryError(f"step {index + 1} type must be joint, cartesian, or tool")

    step_id = str(raw.get("id") or f"step-{index + 1}").strip()
    label = str(raw.get("label") or f"Step {index + 1}").strip()
    if not step_id:
        raise ProgramLibraryError(f"step {index + 1} id must not be empty")
    if not label:
        raise ProgramLibraryError(f"step {index + 1} label must not be empty")

    step: dict[str, Any] = {
        "id": step_id,
        "label": label,
        "type": kind,
        "enabled": raw.get("enabled", True) is not False,
        "source": str(raw.get("source") or "program_library"),
        "source_label": str(raw.get("source_label") or raw.get("source") or "saved program"),
    }

    if kind == "tool":
        action = str(raw.get("action") or "").strip().lower()
        if action not in {"open", "close", "set", "on", "off"}:
            raise ProgramLibraryError(
                f"step {index + 1} tool action must be open, close, set, on, or off"
            )
        step["mode"] = "tool"
        step["action"] = action
        tool = str(raw.get("tool") or "").strip()
        if tool:
            step["tool"] = tool
        if action == "set":
            value = _finite_number(raw.get("value"), f"step {index + 1} value")
            if not 0.0 <= value <= 1.0:
                raise ProgramLibraryError(f"step {index + 1} value must be between 0 and 1")
            step["value"] = value
        settle_ms = _finite_number(raw.get("settle_ms", 150.0), f"step {index + 1} settle_ms")
        if settle_ms < 0:
            raise ProgramLibraryError(f"step {index + 1} settle_ms must not be negative")
        step["settle_ms"] = settle_ms
        return step

    step["mode"] = "joint"
    step["branch"] = str(raw.get("branch") or "auto")
    raw_settings = raw.get("settings")
    if raw_settings is not None:
        if not isinstance(raw_settings, dict):
            raise ProgramLibraryError(f"step {index + 1} settings must be an object")
        settings: dict[str, Any] = {}
        for key in (
            "global_speed_deg_s",
            "global_accel_deg_s2",
            "tcp_speed_mm_s",
            "tcp_accel_mm_s2",
            "phi_speed_deg_s",
            "phi_accel_deg_s2",
        ):
            if raw_settings.get(key) is None:
                continue
            value = _finite_number(raw_settings[key], f"step {index + 1} settings.{key}")
            if value <= 0:
                raise ProgramLibraryError(f"step {index + 1} settings.{key} must be positive")
            settings[key] = value
        if settings:
            step["settings"] = settings

    if kind == "joint":
        angles = raw.get("angles_deg")
        if not isinstance(angles, list) or len(angles) != len(config.joints):
            raise ProgramLibraryError(
                f"step {index + 1} must define {len(config.joints)} joint angles"
            )
        step["angles_deg"] = [
            _finite_number(value, f"step {index + 1} angles_deg[{joint_index}]")
            for joint_index, value in enumerate(angles)
        ]
        return step

    mode = str(raw.get("mode") or "joint").strip().lower()
    if mode not in {"joint", "linear"}:
        raise ProgramLibraryError(f"step {index + 1} mode must be joint or linear")
    target = raw.get("target")
    if not isinstance(target, dict):
        raise ProgramLibraryError(f"step {index + 1} is missing target")
    normalized_target = {
        "x_mm": _finite_number(target.get("x_mm"), f"step {index + 1} target.x_mm"),
        "y_mm": _finite_number(target.get("y_mm"), f"step {index + 1} target.y_mm"),
        "z_mm": _finite_number(target.get("z_mm"), f"step {index + 1} target.z_mm"),
    }
    if bool(target.get("phi_auto", False)) or target.get("phi_deg") is None:
        normalized_target["phi_auto"] = True
        if target.get("preferred_phi_deg") is not None:
            normalized_target["preferred_phi_deg"] = _finite_number(
                target.get("preferred_phi_deg"),
                f"step {index + 1} target.preferred_phi_deg",
            )
    else:
        normalized_target["phi_deg"] = _finite_number(
            target.get("phi_deg"),
            f"step {index + 1} target.phi_deg",
        )
    step["mode"] = mode
    step["target"] = normalized_target
    return step


def normalize_program(
    config: RobotConfig,
    raw: dict[str, Any],
    *,
    read_only: bool = False,
    template: bool = False,
    source: str = "user",
) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise ProgramLibraryError("program must be an object")

    name = str(raw.get("name") or "").strip()
    if not name:
        raise ProgramLibraryError("program name must not be empty")
    program_id = str(raw.get("id") or new_program_id(name)).strip()
    if not program_id:
        raise ProgramLibraryError("program id must not be empty")
    raw_steps = raw.get("steps", [])
    if not isinstance(raw_steps, list):
        raise ProgramLibraryError("program steps must be a list")

    created_at = str(raw.get("created_at") or _now_iso())
    updated_at = str(raw.get("updated_at") or created_at)
    program: dict[str, Any] = {
        "schema_version": PROGRAM_SCHEMA_VERSION,
        "id": program_id,
        "name": name,
        "description": str(raw.get("description") or "").strip(),
        "read_only": bool(read_only),
        "template": bool(template),
        "source": source,
        "steps": [
            normalize_program_step(config, step, index)
            for index, step in enumerate(raw_steps)
        ],
        "created_at": created_at,
        "updated_at": updated_at,
    }
    required_tool = raw.get("required_tool")
    if isinstance(required_tool, str) and required_tool.strip():
        program["required_tool"] = required_tool.strip()
    if isinstance(raw.get("metadata"), dict):
        program["metadata"] = deepcopy(raw["metadata"])
    if isinstance(raw.get("cached_plan"), dict):
        program["cached_plan"] = deepcopy(raw["cached_plan"])
    return program


def _neutral_demo_pose(config: RobotConfig) -> list[float]:
    fractions = [0.5, 0.54, 0.21875]
    pose = [
        joint.min_deg + (joint.max_deg - joint.min_deg) * fraction
        for joint, fraction in zip(config.joints[:3], fractions, strict=True)
    ]
    desired_phi_deg = 0.0
    wrist = desired_phi_deg - pose[1] - pose[2]
    pose.append(max(config.joints[3].min_deg, min(config.joints[3].max_deg, wrist)))
    return [round(value, 6) for value in pose]


def _demo_radius_mm(config: RobotConfig) -> float:
    model_scale = sum(
        abs(row.a_mm) + abs(row.d_mm)
        for row in config.kinematics.dh_rows
    )
    return round(max(12.0, min(32.0, model_scale * 0.045)), 3)


def _joint_step(step_id: str, label: str, angles: list[float]) -> dict[str, Any]:
    return {
        "id": step_id,
        "label": label,
        "type": "joint",
        "mode": "joint",
        "enabled": True,
        "branch": "auto",
        "source": "demo_generator",
        "source_label": "Adaptive demo",
        "angles_deg": [round(float(value), 6) for value in angles],
    }


def _cartesian_step(
    step_id: str,
    label: str,
    *,
    x_mm: float,
    y_mm: float,
    z_mm: float,
    phi_deg: float,
) -> dict[str, Any]:
    return {
        "id": step_id,
        "label": label,
        "type": "cartesian",
        "mode": "linear",
        "enabled": True,
        "branch": "auto",
        "source": "demo_generator",
        "source_label": "Adaptive demo",
        "target": {
            "x_mm": round(float(x_mm), 6),
            "y_mm": round(float(y_mm), 6),
            "z_mm": round(float(z_mm), 6),
            "phi_deg": round(float(phi_deg), 6),
        },
    }


def _shape_demo(
    config: RobotConfig,
    *,
    program_id: str,
    name: str,
    description: str,
    points: list[tuple[float, float]],
    shape: str,
) -> dict[str, Any]:
    neutral_pose = _neutral_demo_pose(config)
    center = forward_kinematics(neutral_pose, config.links)
    radius = _demo_radius_mm(config)
    steps = [_joint_step(f"{shape}-ready", "Move to drawing pose", neutral_pose)]
    for index, (x_factor, z_factor) in enumerate(points):
        steps.append(
            _cartesian_step(
                f"{shape}-{index + 1}",
                f"{name} point {index + 1}",
                x_mm=center["x_mm"] + radius * x_factor,
                y_mm=center["y_mm"],
                z_mm=center["z_mm"] + radius * z_factor,
                phi_deg=center["tool_phi_deg"],
            )
        )
    steps.append(_joint_step(f"{shape}-return", "Return to drawing pose", neutral_pose))
    return normalize_program(
        config,
        {
            "id": program_id,
            "name": name,
            "description": description,
            "steps": steps,
            "created_at": "built-in",
            "updated_at": "built-in",
            "metadata": {
                "demo_kind": shape,
                "adaptive": True,
                "drawing_plane": "robot X-Z at fixed Y",
                "radius_mm": radius,
                "preview_required": True,
            },
        },
        read_only=True,
        template=True,
        source="built_in",
    )


def _showcase_demo(config: RobotConfig) -> dict[str, Any]:
    center = _neutral_demo_pose(config)
    amplitudes = [
        min(22.0, (joint.max_deg - joint.min_deg) * fraction)
        for joint, fraction in zip(config.joints, [0.08, 0.07, 0.07, 0.09], strict=True)
    ]

    def offset(*values: float) -> list[float]:
        return [
            round(
                max(joint.min_deg, min(joint.max_deg, base + amplitude * value)),
                6,
            )
            for joint, base, amplitude, value in zip(
                config.joints,
                center,
                amplitudes,
                values,
                strict=True,
            )
        ]

    poses = [
        ("showcase-ready", "Move to showcase pose", center),
        ("showcase-left", "Base sweep left", offset(-1.0, 0.0, 0.0, 0.0)),
        ("showcase-right", "Base sweep right", offset(1.0, 0.0, 0.0, 0.0)),
        ("showcase-rise", "Shoulder rise", offset(0.35, 0.9, -0.55, -0.2)),
        ("showcase-fold", "Elbow fold", offset(-0.35, -0.45, 1.0, -0.45)),
        ("showcase-reach", "Coordinated reach", offset(0.65, 0.55, -0.75, 0.35)),
        ("showcase-wrist-a", "Wrist flourish A", offset(0.15, 0.1, -0.1, 1.0)),
        ("showcase-wrist-b", "Wrist flourish B", offset(-0.15, -0.1, 0.1, -1.0)),
        ("showcase-return", "Return to showcase pose", center),
    ]
    return normalize_program(
        config,
        {
            "id": "demo-kinematic-showcase",
            "name": "Kinematic Showcase",
            "description": (
                "A conservative joint-space routine that demonstrates coordinated base, "
                "shoulder, elbow, and wrist motion."
            ),
            "steps": [
                _joint_step(step_id, label, angles)
                for step_id, label, angles in poses
            ],
            "created_at": "built-in",
            "updated_at": "built-in",
            "metadata": {
                "demo_kind": "joint_showcase",
                "adaptive": True,
                "preview_required": True,
            },
        },
        read_only=True,
        template=True,
        source="built_in",
    )


def built_in_programs(config: RobotConfig) -> list[dict[str, Any]]:
    square_points = [
        (-1.0, -1.0),
        (1.0, -1.0),
        (1.0, 1.0),
        (-1.0, 1.0),
        (-1.0, -1.0),
    ]
    circle_segments = 24
    circle_points = [
        (cos(tau * index / circle_segments), sin(tau * index / circle_segments))
        for index in range(circle_segments + 1)
    ]
    return [
        _shape_demo(
            config,
            program_id="demo-air-square",
            name="Air Square",
            description=(
                "Draws a compact square in the robot X-Z plane using linear TCP segments."
            ),
            points=square_points,
            shape="square",
        ),
        _shape_demo(
            config,
            program_id="demo-air-circle",
            name="Air Circle",
            description=(
                "Traces a 24-segment circular polyline in the robot X-Z plane. "
                "Preview the faceted path before running."
            ),
            points=circle_points,
            shape="circle",
        ),
        _showcase_demo(config),
    ]


def _store_path(path: str | Path | None) -> Path:
    return Path(path) if path is not None else PROGRAM_STORE_PATH


def load_user_programs(
    config: RobotConfig,
    path: str | Path | None = None,
) -> dict[str, dict[str, Any]]:
    store_path = _store_path(path)
    if not store_path.exists():
        return {}
    try:
        payload = json.loads(store_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProgramLibraryError(f"could not read user program store: {exc}") from exc

    raw_programs: Any
    if isinstance(payload, list):
        raw_programs = payload
    elif isinstance(payload, dict):
        raw_programs = payload.get("programs", [])
    else:
        raise ProgramLibraryError("user program store must be an object or list")
    if isinstance(raw_programs, dict):
        raw_programs = [
            {**program, "id": program.get("id") or program_id}
            for program_id, program in raw_programs.items()
            if isinstance(program, dict)
        ]
    if not isinstance(raw_programs, list):
        raise ProgramLibraryError("user program store programs must be a list")

    programs: dict[str, dict[str, Any]] = {}
    for raw_program in raw_programs:
        program = normalize_program(config, raw_program, source="user")
        programs[program["id"]] = program
    return programs


def write_user_programs(
    programs: dict[str, dict[str, Any]],
    path: str | Path | None = None,
) -> None:
    store_path = _store_path(path)
    store_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": PROGRAM_STORE_SCHEMA_VERSION,
        "programs": sorted(
            (deepcopy(program) for program in programs.values()),
            key=lambda item: (str(item.get("name", "")).lower(), str(item.get("id", ""))),
        ),
    }
    temporary_path = store_path.with_suffix(f"{store_path.suffix}.tmp")
    try:
        temporary_path.write_text(
            json.dumps(payload, indent=2, sort_keys=False) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(store_path)
    except OSError as exc:
        raise ProgramLibraryError(f"could not save user programs: {exc}") from exc


def save_user_program(
    config: RobotConfig,
    raw: dict[str, Any],
    path: str | Path | None = None,
) -> dict[str, Any]:
    programs = load_user_programs(config, path)
    requested_id = str(raw.get("id") or "").strip()
    if requested_id.startswith("demo-"):
        raise ProgramLibraryError("built-in templates are read-only; copy the template first")
    existing = programs.get(requested_id)
    now = _now_iso()
    prepared = {
        **deepcopy(raw),
        "id": requested_id or new_program_id(str(raw.get("name") or "program")),
        "created_at": existing.get("created_at") if existing else now,
        "updated_at": now,
    }
    program = normalize_program(config, prepared, source="user")
    programs[program["id"]] = program
    write_user_programs(programs, path)
    return program


def save_user_program_cached_plan(
    config: RobotConfig,
    program_id: str,
    cached_plan: dict[str, Any],
    path: str | Path | None = None,
) -> dict[str, Any]:
    programs = load_user_programs(config, path)
    program = programs.get(program_id)
    if program is None:
        raise ProgramLibraryError(f"program {program_id} was not found")
    if program.get("read_only") or str(program_id).startswith("demo-"):
        raise ProgramLibraryError("built-in templates cannot store compiled plans")
    prepared_plan = deepcopy(cached_plan)
    prepared_plan["schema_version"] = CACHED_PLAN_SCHEMA_VERSION
    prepared_plan["program_fingerprint"] = program_motion_fingerprint(program)
    prepared_plan["saved_at"] = _now_iso()
    program["cached_plan"] = prepared_plan
    programs[program_id] = normalize_program(config, program, source="user")
    write_user_programs(programs, path)
    return programs[program_id]


def delete_user_program(
    config: RobotConfig,
    program_id: str,
    path: str | Path | None = None,
) -> bool:
    if str(program_id).startswith("demo-"):
        raise ProgramLibraryError("built-in templates cannot be deleted")
    programs = load_user_programs(config, path)
    if program_id not in programs:
        return False
    programs.pop(program_id)
    write_user_programs(programs, path)
    return True


def all_programs(
    config: RobotConfig,
    path: str | Path | None = None,
) -> list[dict[str, Any]]:
    built_ins = built_in_programs(config)
    users = sorted(
        load_user_programs(config, path).values(),
        key=lambda item: (str(item.get("name", "")).lower(), str(item.get("id", ""))),
    )
    return [*built_ins, *users]


def find_program(
    config: RobotConfig,
    program_id: str,
    path: str | Path | None = None,
) -> dict[str, Any] | None:
    for program in built_in_programs(config):
        if program["id"] == program_id:
            return program
    return load_user_programs(config, path).get(program_id)


def copy_program_to_user(
    config: RobotConfig,
    program_id: str,
    *,
    name: str | None = None,
    path: str | Path | None = None,
) -> dict[str, Any]:
    source = find_program(config, program_id, path)
    if source is None:
        raise ProgramLibraryError(f"program {program_id} was not found")
    copy_name = str(name or f"{source['name']} Copy").strip()
    copied = {
        **deepcopy(source),
        "id": new_program_id(copy_name),
        "name": copy_name,
        "read_only": False,
        "template": False,
        "source": "user",
        "created_at": _now_iso(),
        "updated_at": _now_iso(),
        "metadata": {
            **deepcopy(source.get("metadata", {})),
            "copied_from": source["id"],
        },
    }
    copied.pop("cached_plan", None)
    for index, step in enumerate(copied["steps"]):
        step["id"] = f"step-{index + 1}"
    return save_user_program(config, copied, path)
