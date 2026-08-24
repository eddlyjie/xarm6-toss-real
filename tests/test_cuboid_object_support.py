import json
import math
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "filename, expected_dimensions, expected_mass",
    [
        ("cuboid30_20g.json", [0.0445, 0.030, 0.046], 0.020),
        ("cuboid33_26p6g.json", [0.0505, 0.0335, 0.051], 0.0266),
        ("cuboid38_37g.json", [0.0575, 0.038, 0.058], 0.037),
    ],
)
def test_cuboid_object_profiles_use_narrow_y_grip_and_box_inertia(
    filename, expected_dimensions, expected_mass
):
    profile = json.loads(
        (ROOT / "configs" / "objects" / filename).read_text(encoding="utf-8")
    )
    assert profile["shape"] == "cuboid"
    assert profile["dimensions_m"] == expected_dimensions
    assert profile["mass_kg"] == expected_mass
    assert profile["grasp"]["gripped_dimension_m"] == min(
        profile["measured_dimensions_m"]
    )
    assert profile["dimensions_m"][1] == profile["grasp"]["gripped_dimension_m"]
    x_m, y_m, z_m = expected_dimensions
    expected_inertia = [
        expected_mass * (y_m**2 + z_m**2) / 12.0,
        expected_mass * (x_m**2 + z_m**2) / 12.0,
        expected_mass * (x_m**2 + y_m**2) / 12.0,
    ]
    assert profile["principal_inertia_kg_m2"] == pytest.approx(expected_inertia)
    assert all(math.isfinite(value) and value > 0 for value in expected_inertia)


def test_native_runner_uses_cuboid_dimensions_through_geometry_and_evidence():
    runner = (ROOT / "sim/scripts/04_native_release_smoke.py").read_text(
        encoding="utf-8"
    )
    assert 'parser.add_argument(\n    "--object-profile"' in runner
    assert 'parser.add_argument(\n    "--object-dimensions-m"' in runner
    assert "size=dimensions_m" in runner
    assert "cuboid_ground_clearance_m(" in runner
    assert '"object_dimensions_m": list(args_cli.object_dimensions_m)' in runner
    assert '"object_principal_inertia_kg_m2"' in runner
    assert "spawn_cube_rotation_marker(args_cli.object_dimensions_m)" in runner
