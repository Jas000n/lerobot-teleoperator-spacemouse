import sys
from types import SimpleNamespace

from lerobot_teleoperator_spacemouse import patches


def test_patch_forwards_new_lerobot_loop_arguments(monkeypatch):
    received = {}

    def original_loop(teleop, robot, robot_action_processor, *, display_mode="rerun"):
        received.update(
            teleop=teleop,
            robot=robot,
            robot_action_processor=robot_action_processor,
            display_mode=display_mode,
        )
        return "done"

    module = SimpleNamespace(teleop_loop=original_loop)
    monkeypatch.setitem(sys.modules, "lerobot.scripts.lerobot_teleoperate", module)
    replacement = object()
    monkeypatch.setattr(patches, "make_spacemouse_robot_action_processor", lambda config, robot: replacement)

    patches.patch_lerobot_teleoperate()
    teleop = SimpleNamespace(name="spacemouse", config=SimpleNamespace(adapter=object()))
    robot = object()

    result = module.teleop_loop(
        teleop=teleop,
        robot=robot,
        robot_action_processor=object(),
        display_mode="foxglove",
    )

    assert result == "done"
    assert received == {
        "teleop": teleop,
        "robot": robot,
        "robot_action_processor": replacement,
        "display_mode": "foxglove",
    }
