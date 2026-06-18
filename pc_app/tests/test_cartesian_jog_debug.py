import pytest

from app.cartesian_jog_debug import simulate_cartesian_jog
from app.config import EXAMPLE_CONFIG_PATH, load_config


@pytest.mark.parametrize(
    ("start_deg", "velocity_xyz_mm_s"),
    [
        ([0.0, 45.0, 25.0, -20.0], [40.0, 0.0, 0.0]),
        ([0.0, 25.0, 80.0, -50.0], [0.0, 40.0, 0.0]),
        ([0.0, 25.0, 80.0, -50.0], [0.0, 0.0, -40.0]),
    ],
)
def test_cartesian_jog_simulation_keeps_reachable_xyz_directions_straight(
    start_deg,
    velocity_xyz_mm_s,
):
    config = load_config(EXAMPLE_CONFIG_PATH)

    result = simulate_cartesian_jog(
        config,
        start_deg,
        velocity_xyz_mm_s,
        steps=24,
    )

    assert result["blocked_steps"] == 0
    assert result["metrics"]["progress_mm"] > 20.0
    assert result["metrics"]["alignment"] > 0.95
    assert result["metrics"]["max_lateral_mm"] < max(
        10.0,
        result["metrics"]["progress_mm"] * 0.15,
    )


def test_cartesian_jog_simulation_blocks_bad_local_direction():
    config = load_config(EXAMPLE_CONFIG_PATH)

    result = simulate_cartesian_jog(
        config,
        [0.0, 62.3, 0.0, 0.0],
        [0.0, 0.0, 40.0],
        steps=12,
    )

    assert result["blocked_steps"] == 12
    assert result["metrics"]["progress_mm"] == 0.0
    assert result["metrics"]["max_lateral_mm"] == 0.0
    assert any("excessive lateral TCP drift" in note for note in result["notes"])


def test_cartesian_jog_simulation_blocks_singular_sideways_drift():
    config = load_config(EXAMPLE_CONFIG_PATH)

    result = simulate_cartesian_jog(
        config,
        [0.0, 62.3, 0.0, 0.0],
        [0.0, -40.0, 0.0],
        steps=12,
    )

    assert result["blocked_steps"] == 12
    assert result["metrics"]["progress_mm"] == 0.0
    assert result["metrics"]["max_lateral_mm"] == 0.0
    assert any("excessive lateral TCP drift" in note for note in result["notes"])
