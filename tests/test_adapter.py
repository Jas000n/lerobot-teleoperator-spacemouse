from pathlib import Path

import numpy as np
from lerobot.processor.converters import create_transition

from lerobot_teleoperator_spacemouse._lerobot_compat import TransitionKey
from lerobot_teleoperator_spacemouse.adapter import (
    DEFAULT_SOARM_MOTOR_NAMES,
    EndEffectorBounds,
    EndEffectorToJoints,
    GripperVelocityToPosition,
    SpaceMouseAdapterConfig,
    SpaceMouseDeltaToEndEffector,
    resolve_kinematics_config,
    robot_accepts_direct_eef,
)


class FakeKinematics:
    def forward_kinematics(self, q):
        del q
        transform = np.eye(4)
        transform[:3, 3] = [0.1, 0.2, 0.3]
        return transform

    def inverse_kinematics(self, current, target, position_weight=1.0, orientation_weight=0.01):
        del position_weight, orientation_weight
        result = np.array(current, dtype=float)
        result[:3] = target[:3, 3]
        return result


class NoIkKinematics(FakeKinematics):
    def inverse_kinematics(self, current, target, position_weight=1.0, orientation_weight=0.01):
        raise AssertionError("IK should not run when SpaceMouse input is disabled")


def observation():
    return {f"{name}.pos": float(i) for i, name in enumerate(DEFAULT_SOARM_MOTOR_NAMES)}


def idle_action():
    return {
        "enabled": False,
        "target_x": 0.0,
        "target_y": 0.0,
        "target_z": 0.0,
        "target_wx": 0.0,
        "target_wy": 0.0,
        "target_wz": 0.0,
        "gripper_vel": 0.0,
    }


def test_default_profile_resolves_bundled_so101_urdf():
    resolved = resolve_kinematics_config(SpaceMouseAdapterConfig())

    urdf_path = Path(resolved.urdf_path)
    assert urdf_path.name == "so101_new_calib.urdf"
    assert urdf_path.exists()
    assert "mesh" not in urdf_path.read_text()
    assert resolved.target_frame_name == "gripper_frame_link"
    assert resolved.motor_names == list(DEFAULT_SOARM_MOTOR_NAMES)
    assert resolved.gripper_name == "gripper"


def test_delta_to_end_effector_uses_current_fk_pose():
    step = SpaceMouseDeltaToEndEffector(
        kinematics=FakeKinematics(),
        motor_names=DEFAULT_SOARM_MOTOR_NAMES,
        translation_step_m=0.01,
        rotation_step_rad=0.1,
    )
    transition = create_transition(
        observation=observation(),
        action={
            "enabled": True,
            "target_x": 1.0,
            "target_y": -1.0,
            "target_z": 0.5,
            "target_wx": 0.0,
            "target_wy": 0.0,
            "target_wz": 0.0,
            "gripper_vel": 1.0,
        },
    )

    output = step(transition)[TransitionKey.ACTION]

    assert output["ee.x"] == 0.11
    assert output["ee.y"] == 0.19
    assert output["ee.z"] == 0.305
    assert output["ee.gripper_vel"] == 1.0


def test_delta_to_end_effector_integrates_enabled_target():
    step = SpaceMouseDeltaToEndEffector(
        kinematics=FakeKinematics(),
        motor_names=DEFAULT_SOARM_MOTOR_NAMES,
        translation_step_m=0.01,
        rotation_step_rad=0.1,
    )
    action = {
        "enabled": True,
        "target_x": 1.0,
        "target_y": 0.0,
        "target_z": 0.0,
        "target_wx": 0.0,
        "target_wy": 0.0,
        "target_wz": 0.0,
        "gripper_vel": 0.0,
    }

    first = step(create_transition(observation=observation(), action=action))[TransitionKey.ACTION]
    second = step(create_transition(observation=observation(), action=action))[TransitionKey.ACTION]

    assert first["ee.x"] == 0.11
    assert second["ee.x"] == 0.12


def test_delta_target_does_not_wind_up_past_workspace_limit():
    step = SpaceMouseDeltaToEndEffector(
        kinematics=FakeKinematics(),
        motor_names=DEFAULT_SOARM_MOTOR_NAMES,
        translation_step_m=0.01,
        rotation_step_rad=0.1,
        workspace_max=[0.11, 1.0, 1.0],
    )
    action = {
        "enabled": True,
        "target_x": 1.0,
        "target_y": 0.0,
        "target_z": 0.0,
        "target_wx": 0.0,
        "target_wy": 0.0,
        "target_wz": 0.0,
        "gripper_vel": 0.0,
    }

    for _ in range(3):
        at_limit = step(create_transition(observation=observation(), action=action))[TransitionKey.ACTION]
    assert at_limit["ee.x"] == 0.11

    action["target_x"] = -1.0
    reversed_output = step(create_transition(observation=observation(), action=action))[TransitionKey.ACTION]
    assert reversed_output["ee.x"] == 0.1


def test_disabled_input_holds_current_joints_without_ik():
    delta = SpaceMouseDeltaToEndEffector(
        kinematics=NoIkKinematics(),
        motor_names=DEFAULT_SOARM_MOTOR_NAMES,
        translation_step_m=0.01,
        rotation_step_rad=0.1,
    )
    ik = EndEffectorToJoints(
        kinematics=NoIkKinematics(),
        motor_names=DEFAULT_SOARM_MOTOR_NAMES,
        gripper_name="gripper",
        position_weight=1.0,
        orientation_weight=0.01,
        initial_guess_current_joints=True,
    )
    transition = create_transition(
        observation=observation(),
        action={
            "enabled": False,
            "target_x": 0.0,
            "target_y": 0.0,
            "target_z": 0.0,
            "target_wx": 0.0,
            "target_wy": 0.0,
            "target_wz": 0.0,
            "gripper_vel": 0.0,
        },
    )

    ee_transition = delta(transition)
    output = ik(ee_transition)[TransitionKey.ACTION]

    assert output == observation()


def test_gripper_and_ik_emit_soarm_joint_action():
    grip = GripperVelocityToPosition(
        gripper_key="gripper.pos",
        speed_factor=2.0,
        clip_min=0.0,
        clip_max=100.0,
    )
    ik = EndEffectorToJoints(
        kinematics=FakeKinematics(),
        motor_names=DEFAULT_SOARM_MOTOR_NAMES,
        gripper_name="gripper",
        position_weight=1.0,
        orientation_weight=0.01,
        initial_guess_current_joints=True,
    )
    transition = create_transition(
        observation=observation(),
        action={
            "ee.x": 0.4,
            "ee.y": 0.5,
            "ee.z": 0.6,
            "ee.wx": 0.0,
            "ee.wy": 0.0,
            "ee.wz": 0.0,
            "ee.gripper_vel": 1.0,
        },
    )

    with_gripper = grip(transition)
    output = ik(with_gripper)[TransitionKey.ACTION]

    assert output["shoulder_pan.pos"] == 0.4
    assert output["shoulder_lift.pos"] == 0.5
    assert output["elbow_flex.pos"] == 0.6
    assert output["gripper.pos"] == 7.0


def test_disabled_input_resets_bounds_without_changing_hold_pose():
    bounds = EndEffectorBounds(
        workspace_min=[0.0, 0.0, 0.0],
        workspace_max=[1.0, 1.0, 1.0],
        max_ee_step_m=0.02,
    )
    enabled = {"enabled": True, "ee.x": 0.5, "ee.y": 0.5, "ee.z": 0.5}
    bounds(create_transition(observation={}, action=enabled))

    disabled = {"enabled": False, "ee.x": 1.2, "ee.y": 0.5, "ee.z": 0.5}
    held = bounds(create_transition(observation={}, action=disabled))[TransitionKey.ACTION]

    assert held["ee.x"] == 1.2

    resumed = {"enabled": True, "ee.x": 0.8, "ee.y": 0.5, "ee.z": 0.5}
    output = bounds(create_transition(observation={}, action=resumed))[TransitionKey.ACTION]
    assert output["ee.x"] == 0.8


def test_direct_eef_auto_detection_requires_observation_pose():
    required = dict.fromkeys(("ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz"), float)

    class RobotWithActionOnly:
        action_features = required
        observation_features = {}

    class RobotWithPoseFeedback:
        action_features = required
        observation_features = required

    assert robot_accepts_direct_eef(RobotWithActionOnly()) is False
    assert robot_accepts_direct_eef(RobotWithPoseFeedback()) is True


def _hold_steps(**kwargs):
    delta = SpaceMouseDeltaToEndEffector(
        kinematics=FakeKinematics(),
        motor_names=DEFAULT_SOARM_MOTOR_NAMES,
        translation_step_m=0.01,
        rotation_step_rad=0.1,
        hold_target_when_idle=True,
        **kwargs,
    )
    ik = EndEffectorToJoints(
        kinematics=FakeKinematics(),
        motor_names=DEFAULT_SOARM_MOTOR_NAMES,
        gripper_name="gripper",
        position_weight=1.0,
        orientation_weight=0.01,
        initial_guess_current_joints=True,
        hold_target_when_idle=True,
    )
    return delta, ik


def test_hold_mode_keeps_the_target_while_the_measurement_sags():
    # A P-controlled servo settles below its command under gravity. Re-deriving the target from
    # that measurement turns the offset into a downward creep, one sag per frame.
    delta, _ = _hold_steps()
    push = {**idle_action(), "enabled": True, "target_z": 1.0}
    measured = observation()

    target_z = delta(create_transition(observation=measured, action=push))[TransitionKey.ACTION]["ee.z"]

    for _ in range(5):
        measured = {name: value - 0.5 for name, value in measured.items()}
        held = delta(create_transition(observation=measured, action=idle_action()))[TransitionKey.ACTION]
        assert held["ee.z"] == target_z


def test_hold_mode_stops_the_target_running_away_from_an_arm_that_never_follows():
    # FakeKinematics always reports the same FK pose, so the arm never follows: without the guard
    # the target would integrate one step per frame forever.
    delta, _ = _hold_steps(max_target_lead_m=0.025)
    push = {**idle_action(), "enabled": True, "target_z": 1.0}

    for _ in range(50):
        out = delta(create_transition(observation=observation(), action=push))[TransitionKey.ACTION]

    lead = out["ee.z"] - 0.3          # FakeKinematics puts the tool at z = 0.3
    assert 0.025 <= lead <= 0.025 + 0.01 + 1e-9      # capped, at most one further step


def test_hold_mode_always_allows_motion_back_towards_the_arm():
    delta, _ = _hold_steps(max_target_lead_m=0.025)
    push = {**idle_action(), "enabled": True, "target_z": 1.0}
    for _ in range(50):
        delta(create_transition(observation=observation(), action=push))
    pull = {**idle_action(), "enabled": True, "target_z": -1.0}

    out = delta(create_transition(observation=observation(), action=pull))[TransitionKey.ACTION]

    assert out["ee.z"] < 0.3 + 0.025


def test_hold_mode_uses_one_command_source_so_idle_frames_do_not_jump():
    delta, ik = _hold_steps()
    push = {**idle_action(), "enabled": True, "target_z": 1.0}
    obs = observation()

    ik(delta(create_transition(observation=obs, action=push)))
    active = ik(delta(create_transition(observation=obs, action=push)))[TransitionKey.ACTION]
    idle = ik(delta(create_transition(observation=obs, action=idle_action())))[TransitionKey.ACTION]

    # same measurement, same target -> the idle frame must not produce a different command
    assert idle == active


def _virtual_leader(**kwargs):
    from lerobot_teleoperator_spacemouse.adapter import VirtualLeaderArm

    defaults = dict(
        kinematics=FakeKinematics(),
        motor_names=list(DEFAULT_SOARM_MOTOR_NAMES),
        gripper_name="gripper",
        translation_step_m=0.01,
        rotation_step_rad=0.1,
        gripper_speed_factor=2.0,
        gripper_min=0.0,
        gripper_max=100.0,
        position_weight=1.0,
        orientation_weight=0.01,
    )
    return VirtualLeaderArm(**{**defaults, **kwargs})


def test_virtual_leader_ignores_the_follower_after_the_first_frame():
    # The whole point of a leader arm: once it has latched, what the follower does never comes back
    # as a command. Both arms start from the same pose, then one follower wanders off.
    push = {**idle_action(), "enabled": True, "target_z": 1.0}
    tracking, stuck = _virtual_leader(), _virtual_leader()
    tracking(create_transition(observation=observation(), action=push))
    stuck(create_transition(observation=observation(), action=push))

    for frame in range(1, 10):
        good = tracking(create_transition(observation=observation(), action=push))[TransitionKey.ACTION]
        wandered = {name: value + 37.0 * frame for name, value in observation().items()}
        bad = stuck(create_transition(observation=wandered, action=push))[TransitionKey.ACTION]

    assert good == bad


def test_virtual_leader_respects_joint_limits_and_step_cap():
    push = {**idle_action(), "enabled": True, "target_z": 1.0}
    step = _virtual_leader(max_joint_step_deg=0.05, joint_limits_deg={"shoulder_pan": (-1.0, 1.0)})

    first = step(create_transition(observation=observation(), action=push))[TransitionKey.ACTION]

    assert abs(first["shoulder_pan.pos"]) <= 1.0
    assert abs(first["shoulder_lift.pos"] - 1.0) <= 0.05 + 1e-9      # observation() puts joint i at i


def test_virtual_leader_integrates_the_gripper_virtually():
    close = {**idle_action(), "enabled": True, "gripper_vel": -1.0}
    step = _virtual_leader()

    step(create_transition(observation=observation(), action=close))
    out = step(create_transition(observation=observation(), action=close))[TransitionKey.ACTION]

    assert out["gripper.pos"] == 5.0 - 2 * 2.0       # observation() puts the gripper at 5.0
