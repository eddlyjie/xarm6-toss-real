#!/usr/bin/env python3
"""Report whether the current Python environment can launch Isaac work."""

from __future__ import annotations

from importlib import metadata, util
import platform


def main() -> int:
    print(f"python={platform.python_version()}")
    for module in ("isaacsim", "isaaclab", "torch"):
        available = util.find_spec(module) is not None
        try:
            version = metadata.version(module)
        except metadata.PackageNotFoundError:
            version = "not-installed"
        print(f"{module}: available={available}, version={version}")
    print("This probe does not launch the simulator.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
