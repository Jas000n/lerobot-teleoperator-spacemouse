from pathlib import Path

import numpy as np
from lerobot.processor.converters import create_transition
from lerobot.types import TransitionKey

from lerobot_teleoperator_spacemouse.adapter import (
    DEFAULT_SOARM_MOTOR_NAMES,
    EndEffectorToJoints,
    GripperVelocityToPosition,
    SpaceMouseAdapterConfig,
    SpaceMouseDeltaToEndEffector,
    resolve_kinematics_config,
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
