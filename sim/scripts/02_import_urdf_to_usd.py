#!/usr/bin/env python3
"""Import the checked-in xArm6+G1 URDF with the Isaac Sim 6 API."""

from __future__ import annotations

import argparse
from pathlib import Path


SIM_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = SIM_ROOT / "assets" / "xarm6_g1" / "xarm6_g1.urdf"
DEFAULT_USD = SIM_ROOT / "assets" / "xarm6_g1" / "xarm6_g1.usd"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--usd", type=Path, default=DEFAULT_USD)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})
    try:
        from isaacsim.asset.importer.urdf import (
            URDFImporter,
            URDFImporterConfig,
        )

        importer = URDFImporter(
            URDFImporterConfig(
                urdf_path=str(args.urdf.resolve()),
                usd_path=str(args.usd.resolve()),
                merge_mesh=True,
                allow_self_collision=True,
                fix_base=True,
            )
        )
        output_path = importer.import_urdf()
        print(f"imported={output_path}")
    finally:
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
