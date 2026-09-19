"""Small compatibility layer for public types moved between LeRobot releases."""

try:
    from lerobot.lerobot_types import RobotAction, RobotObservation, TransitionKey
except ImportError:
    try:
        from lerobot.types import RobotAction, RobotObservation, TransitionKey
    except ImportError:
        from lerobot.processor.core import RobotAction, RobotObservation, TransitionKey

from lerobot.processor.pipeline import RobotActionProcessorStep, RobotProcessorPipeline

__all__ = [
    "RobotAction",
    "RobotActionProcessorStep",
    "RobotObservation",
    "RobotProcessorPipeline",
    "TransitionKey",
]
