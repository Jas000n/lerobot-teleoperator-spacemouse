"""Run the plugin inside the real `lerobot-teleoperate` and `lerobot-record` loops.

The other tests stand in for LeRobot with fakes, so they keep passing when a LeRobot release renames
or reorders something the plugin hooks into. These use the installed LeRobot and fake only the
hardware: a SpaceMouse device and an SO-101 follower.
"""

import subprocess
import sys
import textwrap
import time
from types import SimpleNamespace

import lerobot.scripts.lerobot_record as lerobot_record
import lerobot.scripts.lerobot_teleoperate as lerobot_teleoperate
import numpy as np
import pytest
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import make_default_processors
from lerobot.teleoperators.utils import make_teleoperator_from_config

from lerobot_teleoperator_spacemouse import patch_lerobot_record, patch_lerobot_teleoperate
from lerobot_teleoperator_spacemouse.adapter import SpaceMouseAdapterConfig, resolve_kinematics_config
from lerobot_teleoperator_spacemouse.config import SpaceMouseTeleopConfig
from lerobot_teleoperator_spacemouse.profiles import DEFAULT_SOARM_MOTOR_NAMES

MOTORS = list(DEFAULT_SOARM_MOTOR_NAMES)
START_POSE = np.array([0.0, -30.0, 30.0, 60.0, 0.0, 20.0])
FPS = 50
RUN_S = 0.6


def test_lerobot_discovers_the_plugin_and_hooks_both_scripts():
    # A fresh interpreter, so the plugin is imported the way the CLI imports it.
    code = textwrap.dedent(
        """
        import sys
        import warnings

        import lerobot.scripts.lerobot_record as record
        import lerobot.scripts.lerobot_teleoperate as teleoperate
        from lerobot.utils.import_utils import register_third_party_plugins

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            register_third_party_plugins()

        assert "lerobot_teleoperator_spacemouse" in sys.modules, "not discovered: is the plugin pip-installed?"
        ours = [str(w.message) for w in caught if "lerobot-teleoperator-spacemouse" in str(w.message)]
        assert not ours, ours
        assert getattr(teleoperate, "_spacemouse_patch_applied", False), "lerobot-teleoperate not hooked"
        assert getattr(record, "_spacemouse_patch_applied", False), "lerobot-record not hooked"
        """
    )
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=600)
    assert result.returncode == 0, result.stderr[-4000:]


@pytest.fixture(scope="module", autouse=True)
def hooked_scripts():
    patch_lerobot_teleoperate()
    patch_lerobot_record()
    assert lerobot_teleoperate._spacemouse_patch_applied
    assert lerobot_record._spacemouse_patch_applied


class FakeSpaceMouseDevice:
    """Stands in for the pyspacemouse device: the puck held fully along its y axis.

    The default axis map sends SpaceMouse y to the robot's +x. One fresh HID report arrives per
    control frame, then the queue is empty, as with a real device.
    """

    connected = True

    def __init__(self):
        self._report_pending = False

    def read(self):
        self._report_pending = not self._report_pending
        if not self._report_pending:
            return None
        return SimpleNamespace(
            t=time.perf_counter(), x=0.0, y=1.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, button=[0, 0]
        )

    def close(self):
        pass


class FakeFollower:
    """An SO-101 follower without hardware. It covers half the distance to each command per frame."""

    name = "so101_follower"
    robot_type = "so101_follower"

    def __init__(self):
        self.q = START_POSE.copy()
        self.commands: list[np.ndarray] = []

    @property
    def action_features(self):
        return {f"{motor}.pos": float for motor in MOTORS}

    @property
    def observation_features(self):
        return {f"{motor}.pos": float for motor in MOTORS}

    def get_observation(self):
        return {f"{motor}.pos": float(value) for motor, value in zip(MOTORS, self.q, strict=True)}

    def send_action(self, action):
        # Without the plugin's hook, the robot would be handed raw SpaceMouse deltas.
        assert set(action) == set(self.action_features), f"robot got non-joint action keys: {sorted(action)}"
        command = np.array([action[f"{motor}.pos"] for motor in MOTORS], dtype=float)
        assert np.all(np.isfinite(command))
        self.q = self.q + 0.5 * (command - self.q)
        self.commands.append(command)
        return action


def make_spacemouse(tmp_path, mode):
    config = SpaceMouseTeleopConfig(
        id="integration", calibration_dir=tmp_path, adapter=SpaceMouseAdapterConfig(mode=mode)
    )
    teleop = make_teleoperator_from_config(config)
    teleop._device = FakeSpaceMouseDevice()  # what connect() would open through pyspacemouse
    return teleop


def end_effector_motion(follower):
    """End-effector displacement from the start pose to the last command, in metres."""
    kinematics_cfg = resolve_kinematics_config(SpaceMouseAdapterConfig())
    kinematics = RobotKinematics(
        urdf_path=kinematics_cfg.urdf_path, target_frame_name=kinematics_cfg.target_frame_name
    )
    start = kinematics.forward_kinematics(START_POSE)[:3, 3].copy()
    end = kinematics.forward_kinematics(follower.commands[-1])[:3, 3].copy()
    return end - start


def assert_moved_along_x(follower, mode):
    frames = len(follower.commands)
    assert frames >= 10, f"only {frames} commands reached the robot"
    motion = end_effector_motion(follower)
    if mode == "virtual_leader":
        # The virtual arm integrates the full step every frame, regardless of the follower.
        expected = frames * SpaceMouseAdapterConfig().translation_step_m
        assert 0.6 * expected < motion[0] < 1.2 * expected, (motion, expected)
        assert np.all(np.abs(motion[1:]) < 0.25 * expected), (motion, expected)
    else:
        # IK mode re-derives its target from the lagging follower, so it covers less ground.
        assert motion[0] > 0.005, motion
        assert np.all(np.abs(motion[1:]) < 0.5 * motion[0]), motion


@pytest.mark.parametrize("mode", ["virtual_leader", "ik"])
def test_teleoperate_loop_drives_the_follower_in_joint_space(tmp_path, mode):
    follower, teleop = FakeFollower(), make_spacemouse(tmp_path, mode)
    teleop_processor, robot_processor, observation_processor = make_default_processors()

    lerobot_teleoperate.teleop_loop(
        teleop=teleop,
        robot=follower,
        fps=FPS,
        teleop_action_processor=teleop_processor,
        robot_action_processor=robot_processor,
        robot_observation_processor=observation_processor,
        duration=RUN_S,
    )

    assert_moved_along_x(follower, mode)


@pytest.mark.parametrize("mode", ["virtual_leader", "ik"])
def test_record_loop_converts_before_the_dataset_action_is_taken(tmp_path, mode):
    follower, teleop = FakeFollower(), make_spacemouse(tmp_path, mode)
    teleop_processor, robot_processor, observation_processor = make_default_processors()
    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

    # `robot_action_processor` stays LeRobot's identity pipeline, so joint commands can only reach
    # the follower if the conversion ran in `teleop_action_processor`, whose output is what
    # lerobot-record stores as the dataset action.
    lerobot_record.record_loop(
        robot=follower,
        events=events,
        fps=FPS,
        teleop_action_processor=teleop_processor,
        robot_action_processor=robot_processor,
        robot_observation_processor=observation_processor,
        teleop=teleop,
        control_time_s=RUN_S,
    )

    assert_moved_along_x(follower, mode)
