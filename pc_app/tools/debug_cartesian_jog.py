from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.cartesian_jog_debug import simulate_cartesian_jog
from app.config import load_config
from app.kinematics import forward_kinematics


DEFAULT_POSES = {
    "home": None,
    "observed": [0.0, 62.3, 0.0, 0.0],
    "reach": [0.0, 45.0, 25.0, -20.0],
    "offset": [30.0, 60.0, 10.0, -20.0],
    "folded": [0.0, 25.0, 80.0, -50.0],
}

AXES = {
    "x+": [40.0, 0.0, 0.0],
    "x-": [-40.0, 0.0, 0.0],
    "y+": [0.0, 40.0, 0.0],
    "y-": [0.0, -40.0, 0.0],
    "z+": [0.0, 0.0, 40.0],
    "z-": [0.0, 0.0, -40.0],
}


def _status(metrics: dict[str, float | int], blocked_steps: int) -> str:
    alignment = float(metrics["alignment"])
    max_lateral = float(metrics["max_lateral_mm"])
    progress = float(metrics["progress_mm"])
    backward_steps = int(metrics["backward_steps"])
    if blocked_steps:
        return "blocked"
    if progress <= 1.0 or alignment < 0.85 or max_lateral > max(8.0, abs(progress) * 0.35) or backward_steps:
        return "bad"
    return "ok"


def main() -> int:
    parser = argparse.ArgumentParser(description="Simulate Cartesian live jog behavior from known poses.")
    parser.add_argument("--json", action="store_true", help="print full JSON results instead of a table")
    parser.add_argument("--steps", type=int, default=36)
    parser.add_argument("--dt", type=float, default=1.0 / 12.0)
    args = parser.parse_args()

    config = load_config()
    results = []
    for pose_name, pose in DEFAULT_POSES.items():
        start = config.home_pose if pose is None else pose
        fk = forward_kinematics(start, config.links)
        for axis_name, velocity in AXES.items():
            result = simulate_cartesian_jog(config, start, velocity, steps=args.steps, dt_s=args.dt)
            metrics = result["metrics"]
            results.append(
                {
                    "pose": pose_name,
                    "axis": axis_name,
                    "start_deg": start,
                    "start_fk": {"x_mm": fk["x_mm"], "y_mm": fk["y_mm"], "z_mm": fk["z_mm"]},
                    "status": _status(metrics, int(result["blocked_steps"])),
                    "blocked_steps": result["blocked_steps"],
                    "notes": result["notes"],
                    "metrics": metrics,
                }
            )

    if args.json:
        print(json.dumps(results, indent=2))
        return 0

    print("pose      axis status   progress  max_lat  align  back blocked notes")
    for item in results:
        metrics = item["metrics"]
        notes = ", ".join(item["notes"][:2])
        print(
            f"{item['pose']:<9} {item['axis']:<3} {item['status']:<8} "
            f"{float(metrics['progress_mm']):8.1f} {float(metrics['max_lateral_mm']):8.1f} "
            f"{float(metrics['alignment']):6.2f} {int(metrics['backward_steps']):5d} "
            f"{int(item['blocked_steps']):7d} {notes}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
