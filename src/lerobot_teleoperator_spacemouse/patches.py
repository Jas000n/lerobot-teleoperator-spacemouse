from __future__ import annotations

import sys
from functools import wraps
from inspect import signature
from typing import Any

from .adapter import make_spacemouse_robot_action_processor


def _get_call_argument(args: tuple[Any, ...], kwargs: dict[str, Any], name: str, position: int) -> Any:
    if name in kwargs:
        return kwargs[name]
    if len(args) > position:
        return args[position]
    return None


def _replace_call_argument(
    args: tuple[Any, ...], kwargs: dict[str, Any], name: str, position: int, value: Any
) -> tuple[tuple[Any, ...], dict[str, Any]]:
    if name in kwargs or len(args) <= position:
        kwargs[name] = value
        return args, kwargs

    positional = list(args)
    positional[position] = value
    return tuple(positional), kwargs


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
        teleop = bound.arguments.get("teleop")
        robot = bound.arguments["robot"]
        if getattr(teleop, "name", None) == "spacemouse":
            bound.arguments["robot_action_processor"] = make_spacemouse_robot_action_processor(
                teleop.config.adapter, robot
            )
        return original_loop(*bound.args, **bound.kwargs)

    module.teleop_loop = spacemouse_aware_loop
    module._spacemouse_patch_applied = True


def patch_lerobot_record() -> None:
    """Make the stock `lerobot-record` loop SpaceMouse-aware when this plugin is imported."""

    module = sys.modules.get("lerobot.scripts.lerobot_record")
    if module is None or getattr(module, "_spacemouse_patch_applied", False):
        return

    original_loop = getattr(module, "record_loop", None)
    if original_loop is None:
        return

    @wraps(original_loop)
    def spacemouse_aware_loop(*args, **kwargs):
        # LeRobot's safe_stop_image_writer decorator does not preserve the
        # wrapped record_loop signature in 0.5 or 0.6. Support both the keyword
        # calls used by LeRobot and the public function's positional order.
        robot = _get_call_argument(args, kwargs, "robot", 0)
        teleop = _get_call_argument(args, kwargs, "teleop", 7)
        if getattr(teleop, "name", None) == "spacemouse":
            # `lerobot-record` persists the output of teleop_action_processor as
            # the dataset action, so convert SpaceMouse deltas to robot-native
            # actions here instead of only converting the command before send.
            processor = make_spacemouse_robot_action_processor(teleop.config.adapter, robot)
            args, kwargs = _replace_call_argument(args, kwargs, "teleop_action_processor", 3, processor)
        return original_loop(*args, **kwargs)

    module.record_loop = spacemouse_aware_loop
    module._spacemouse_patch_applied = True
