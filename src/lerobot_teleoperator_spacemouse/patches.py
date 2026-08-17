from __future__ import annotations

import sys
from functools import wraps
from inspect import signature

from .adapter import make_spacemouse_robot_action_processor


def patch_lerobot_teleoperate() -> None:
    """Make the stock `lerobot-teleoperate` loop SpaceMouse-aware when this plugin is imported."""

    module = sys.modules.get("lerobot.scripts.lerobot_teleoperate")
    if module is None or getattr(module, "_spacemouse_patch_applied", False):
        return

    original_loop = getattr(module, "teleop_loop", None)
    if original_loop is None:
        return
    loop_signature = signature(original_loop)

    @wraps(original_loop)
    def spacemouse_aware_loop(*args, **kwargs):
        bound = loop_signature.bind(*args, **kwargs)
        teleop = bound.arguments["teleop"]
        robot = bound.arguments["robot"]
        if getattr(teleop, "name", None) == "spacemouse":
            bound.arguments["robot_action_processor"] = make_spacemouse_robot_action_processor(
                teleop.config.adapter, robot
            )
        return original_loop(*bound.args, **bound.kwargs)

    module.teleop_loop = spacemouse_aware_loop
    module._spacemouse_patch_applied = True
