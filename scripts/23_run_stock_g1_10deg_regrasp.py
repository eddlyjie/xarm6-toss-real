#!/usr/bin/env python3
"""Real-hardware entrypoint for the stock-G1 10-degree stable regrasp."""

from pathlib import Path
import runpy
import sys


ROOT = Path(__file__).resolve().parents[1]
TIMELINE = ROOT / "real_handoff/stock_g1_10deg_regrasp_timeline.json"
CONTROLLER = ROOT / "real_handoff/stock_g1_10deg_regrasp_controller.json"


def add_default(flag: str, value: Path) -> None:
    if flag not in sys.argv[1:]:
        sys.argv.extend((flag, str(value)))


if __name__ == "__main__":
    add_default("--timeline", TIMELINE)
    add_default("--controller", CONTROLLER)
    runpy.run_path(
        str(ROOT / "scripts/22_run_j5_dynamic_regrasp.py"),
        run_name="__main__",
    )
