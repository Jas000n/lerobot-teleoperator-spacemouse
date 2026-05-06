"""SpaceMouse teleoperation plugin for LeRobot."""

from .adapter import SpaceMouseAdapterConfig, make_spacemouse_robot_action_processor
from .config import SpaceMouseTeleopConfig
from .patches import patch_lerobot_teleoperate
from .spacemouse import SpaceMouseTeleop

patch_lerobot_teleoperate()

__all__ = [
    "SpaceMouseAdapterConfig",
    "SpaceMouseTeleop",
    "SpaceMouseTeleopConfig",
    "make_spacemouse_robot_action_processor",
    "patch_lerobot_teleoperate",
]
