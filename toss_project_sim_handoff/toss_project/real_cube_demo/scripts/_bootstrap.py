from pathlib import Path
import sys


DEMO_ROOT = Path(__file__).resolve().parents[1]
SRC = DEMO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

