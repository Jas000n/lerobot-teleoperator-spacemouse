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


def test_record_patch_converts_before_dataset_action_is_captured(monkeypatch):
    received = {}

    def original_loop(
        robot,
        events,
        fps,
        teleop_action_processor,
        robot_action_processor,
        robot_observation_processor,
        *,
        teleop=None,
        dataset=None,
        display_mode="rerun",
    ):
        received.update(
            robot=robot,
            events=events,
            fps=fps,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            teleop=teleop,
            dataset=dataset,
            display_mode=display_mode,
        )
        return "recorded"

    # Mirrors LeRobot's safe_stop_image_writer decorator, which intentionally
    # exposes only (*args, **kwargs) rather than preserving the loop signature.
    def decorated_loop(*args, **kwargs):
        return original_loop(*args, **kwargs)

    module = SimpleNamespace(record_loop=decorated_loop)
    monkeypatch.setitem(sys.modules, "lerobot.scripts.lerobot_record", module)
    replacement = object()
    monkeypatch.setattr(patches, "make_spacemouse_robot_action_processor", lambda config, robot: replacement)

    patches.patch_lerobot_record()
    adapter = object()
    teleop = SimpleNamespace(name="spacemouse", config=SimpleNamespace(adapter=adapter))
    robot = object()
    robot_action_processor = object()
    robot_observation_processor = object()

    result = module.record_loop(
        robot=robot,
        events={"exit_early": False},
        fps=30,
        teleop_action_processor=object(),
        robot_action_processor=robot_action_processor,
        robot_observation_processor=robot_observation_processor,
        teleop=teleop,
        dataset="dataset",
        display_mode="foxglove",
    )

    assert result == "recorded"
    assert received == {
        "robot": robot,
        "events": {"exit_early": False},
        "fps": 30,
        "teleop_action_processor": replacement,
        "robot_action_processor": robot_action_processor,
        "robot_observation_processor": robot_observation_processor,
        "teleop": teleop,
        "dataset": "dataset",
        "display_mode": "foxglove",
    }


def test_record_patch_leaves_other_teleoperators_unchanged(monkeypatch):
    received = {}

    def original_loop(teleop, robot, teleop_action_processor):
        received["teleop_action_processor"] = teleop_action_processor

    module = SimpleNamespace(record_loop=original_loop)
    monkeypatch.setitem(sys.modules, "lerobot.scripts.lerobot_record", module)
    monkeypatch.setattr(
        patches,
        "make_spacemouse_robot_action_processor",
        lambda config, robot: (_ for _ in ()).throw(AssertionError("adapter should not be created")),
    )

    patches.patch_lerobot_record()
    original_processor = object()
    module.record_loop(
        teleop=SimpleNamespace(name="so101_leader"),
        robot=object(),
        teleop_action_processor=original_processor,
    )

    assert received["teleop_action_processor"] is original_processor


def test_record_patch_preserves_default_none_teleop(monkeypatch):
    received = {}

    def original_loop(robot, teleop_action_processor, teleop=None):
        received.update(teleop=teleop, teleop_action_processor=teleop_action_processor)

    module = SimpleNamespace(record_loop=original_loop)
    monkeypatch.setitem(sys.modules, "lerobot.scripts.lerobot_record", module)

    patches.patch_lerobot_record()
    original_processor = object()
    module.record_loop(robot=object(), teleop_action_processor=original_processor)

    assert received == {"teleop": None, "teleop_action_processor": original_processor}


def test_record_patch_supports_positional_arguments_after_lerobot_decorator(monkeypatch):
    received = {}

    def original_loop(
        robot,
        events,
        fps,
        teleop_action_processor,
        robot_action_processor,
        robot_observation_processor,
        dataset=None,
        teleop=None,
    ):
        received.update(
            robot=robot,
            teleop=teleop,
            teleop_action_processor=teleop_action_processor,
        )

    def decorated_loop(*args, **kwargs):
        return original_loop(*args, **kwargs)

    module = SimpleNamespace(record_loop=decorated_loop)
    monkeypatch.setitem(sys.modules, "lerobot.scripts.lerobot_record", module)
    replacement = object()
    monkeypatch.setattr(patches, "make_spacemouse_robot_action_processor", lambda config, robot: replacement)

    patches.patch_lerobot_record()
    robot = object()
    teleop = SimpleNamespace(name="spacemouse", config=SimpleNamespace(adapter=object()))
    module.record_loop(robot, {}, 30, object(), object(), object(), None, teleop)

    assert received == {
        "robot": robot,
        "teleop": teleop,
        "teleop_action_processor": replacement,
    }
