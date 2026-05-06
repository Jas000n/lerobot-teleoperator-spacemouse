from __future__ import annotations

import sys
from functools import wraps

from .adapter import make_spacemouse_robot_action_processor


def patch_lerobot_teleoperate() -> None:
    """Make the stock `lerobot-teleoperate` loop SpaceMouse-aware when this plugin is imported."""

    module = sys.modules.get("lerobot.scripts.lerobot_teleoperate")
    if module is None or getattr(module, "_spacemouse_patch_applied", False):
        return

    original_loop = module.teleop_loop

    @wraps(original_loop)
    def spacemouse_aware_loop(
        teleop,
        robot,
        fps,
        teleop_action_processor,
        robot_action_processor,
        robot_observation_processor,
        display_data=False,
        duration=None,
        display_compressed_images=False,
    ):
        if getattr(teleop, "name", None) == "spacemouse":
            robot_action_processor = make_spacemouse_robot_action_processor(teleop.config.adapter, robot)
        return original_loop(
            teleop=teleop,
            robot=robot,
            fps=fps,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            display_data=display_data,
            duration=duration,
            display_compressed_images=display_compressed_images,
        )

    module.teleop_loop = spacemouse_aware_loop
    module._spacemouse_patch_applied = True
