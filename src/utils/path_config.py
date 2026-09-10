import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
ROBOT_DESCRIPTION_ROOT = PROJECT_ROOT / "robot_description"
MJCF_ROOT = ROBOT_DESCRIPTION_ROOT / "mjcf"
URDF_WS = ROBOT_DESCRIPTION_ROOT / "urdf"

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

__all__ = ["PROJECT_ROOT", "ROBOT_DESCRIPTION_ROOT", "MJCF_ROOT", "URDF_WS"]
