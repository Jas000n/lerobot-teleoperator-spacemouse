from __future__ import annotations

import inspect
import sys
import warnings
from collections.abc import Callable
from functools import wraps
from inspect import Parameter, signature
from typing import Any

from .adapter import make_spacemouse_robot_action_processor

ISSUES_URL = "https://github.com/Jas000n/lerobot-teleoperator-spacemouse/issues"

# Arguments the teleoperate patch reads or replaces; it binds them by name.
_TELEOP_LOOP_ARGUMENTS = ("teleop", "robot", "robot_action_processor")

# Where `record_loop` takes the arguments the record patch reads or replaces. LeRobot's decorator
# hides the signature, so the patch also has to find them by position. Unchanged from 0.5 to 0.7.
_RECORD_LOOP_POSITIONS = {"robot": 0, "teleop_action_processor": 3, "teleop": 7}


def _warn_not_patched(script: str, reason: str) -> None:
    # A silent miss would leave --teleop.type=spacemouse sending raw SpaceMouse deltas to the robot.
    warnings.warn(
        f"lerobot-teleoperator-spacemouse could not hook {script}: {reason}. SpaceMouse input will not "
        f"be converted to robot actions there. This LeRobot version is probably newer than the plugin "
        f"supports; please report it at {ISSUES_URL}",
        RuntimeWarning,
        stacklevel=3,
    )


def _undecorated(func: Callable[..., Any]) -> Callable[..., Any] | None:
    """Return the function a decorator wrapped, or None when that cannot be told.

    LeRobot's `safe_stop_image_writer` does not use `functools.wraps`, so fall back to the one
    function its wrapper closes over.
    """
    wrapped = getattr(func, "__wrapped__", None)
    if wrapped is not None:
        return wrapped
    functions = []
    for cell in func.__closure__ or ():
        try:
            contents = cell.cell_contents
        except ValueError:  # empty cell
            continue
        if inspect.isfunction(contents):
            functions.append(contents)
    if not functions:
        return func  # not a decorator wrapper
    return functions[0] if len(functions) == 1 else None


def _misplaced_record_arguments(record_loop: Callable[..., Any]) -> list[str]:
    inner = _undecorated(record_loop)
    if inner is None:
        return []
    parameters = list(signature(inner).parameters.values())
    by_name = {parameter.name: (index, parameter) for index, parameter in enumerate(parameters)}
    misplaced = []
    for name, position in _RECORD_LOOP_POSITIONS.items():
        if name not in by_name:
            misplaced.append(name)
            continue
        index, parameter = by_name[name]
        positional = parameter.kind in (Parameter.POSITIONAL_ONLY, Parameter.POSITIONAL_OR_KEYWORD)
        if positional and index != position:
            misplaced.append(name)
    return misplaced


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
        _warn_not_patched("lerobot-teleoperate", "it has no teleop_loop()")
        return
    loop_signature = signature(original_loop)
    missing = [name for name in _TELEOP_LOOP_ARGUMENTS if name not in loop_signature.parameters]
    if missing:
        _warn_not_patched("lerobot-teleoperate", f"teleop_loop() has no {', '.join(missing)} argument")
        return

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
        _warn_not_patched("lerobot-record", "it has no record_loop()")
        return
    misplaced = _misplaced_record_arguments(original_loop)
    if misplaced:
        _warn_not_patched(
            "lerobot-record", f"record_loop() no longer takes {', '.join(misplaced)} where expected"
        )
        return

    @wraps(original_loop)
    def spacemouse_aware_loop(*args, **kwargs):
        # LeRobot's safe_stop_image_writer decorator does not preserve the
        # wrapped record_loop signature in 0.5 to 0.7. Support both the keyword
        # calls used by LeRobot and the public function's positional order.
        positions = _RECORD_LOOP_POSITIONS
        robot = _get_call_argument(args, kwargs, "robot", positions["robot"])
        teleop = _get_call_argument(args, kwargs, "teleop", positions["teleop"])
        if getattr(teleop, "name", None) == "spacemouse":
            # `lerobot-record` persists the output of teleop_action_processor as
            # the dataset action, so convert SpaceMouse deltas to robot-native
            # actions here instead of only converting the command before send.
            processor = make_spacemouse_robot_action_processor(teleop.config.adapter, robot)
            args, kwargs = _replace_call_argument(
                args, kwargs, "teleop_action_processor", positions["teleop_action_processor"], processor
            )
        return original_loop(*args, **kwargs)

    module.record_loop = spacemouse_aware_loop
    module._spacemouse_patch_applied = True
