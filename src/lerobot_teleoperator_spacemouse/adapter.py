from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from lerobot.model import RobotKinematics
from lerobot.processor import (
    RobotAction,
    RobotActionProcessorStep,
    RobotObservation,
    RobotProcessorPipeline,
    TransitionKey,
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.robot import Robot
from lerobot.utils.rotation import Rotation

from .profiles import DEFAULT_SOARM_MOTOR_NAMES, bundled_urdf_path, get_kinematics_profile

__all__ = [
    "DEFAULT_SOARM_MOTOR_NAMES",
    "ResolvedKinematicsConfig",
    "SpaceMouseAdapterConfig",
    "resolve_kinematics_config",
    "make_spacemouse_robot_action_processor",
]


@dataclass(frozen=True)
class ResolvedKinematicsConfig:
    urdf_path: str
    target_frame_name: str
    motor_names: list[str]
    joint_names: list[str] | None
    gripper_name: str


@dataclass(kw_only=True)
class SpaceMouseAdapterConfig:
    """Convert SpaceMouse delta commands to robot actions."""

    mode: str = "auto"

    # IK mode.
    robot_profile: str = "so101_follower"
    urdf_path: str | None = None
    target_frame_name: str | None = None
    motor_names: list[str] | None = None
    joint_names: list[str] | None = None
    gripper_name: str | None = None

    # Per control-frame deltas at full SpaceMouse deflection.
    translation_step_m: float = 0.001
    rotation_step_rad: float = 0.005

    # Optional workspace clamp. Leave unset while calibrating axes.
    workspace_min: list[float] | None = None
    workspace_max: list[float] | None = None
    max_ee_step_m: float = 0.02

    # IK weights.
    position_weight: float = 1.0
    orientation_weight: float = 0.01
    initial_guess_current_joints: bool = True

    # Gripper integration in LeRobot normalized joint units per frame.
    gripper_speed_factor: float = 2.0
    gripper_min: float = 0.0
    gripper_max: float = 100.0


def resolve_kinematics_config(cfg: SpaceMouseAdapterConfig) -> ResolvedKinematicsConfig:
    profile = get_kinematics_profile(cfg.robot_profile)
    urdf_path = cfg.urdf_path
    if urdf_path is None and profile is not None and profile.urdf_resource is not None:
        urdf_path = bundled_urdf_path(profile.urdf_resource)
    if urdf_path is None:
        raise ValueError(
            "adapter.urdf_path is required for custom IK profiles. "
            "Use adapter.robot_profile='so101_follower' for the bundled SO-101 URDF."
        )

    target_frame_name = cfg.target_frame_name or (profile.target_frame_name if profile is not None else None)
    if target_frame_name is None:
        raise ValueError("adapter.target_frame_name is required for custom IK profiles.")

    motor_names = cfg.motor_names or (list(profile.motor_names) if profile is not None else None)
    if motor_names is None:
        raise ValueError("adapter.motor_names is required for custom IK profiles.")

    joint_names = cfg.joint_names
    if joint_names is None and profile is not None and profile.joint_names is not None:
        joint_names = list(profile.joint_names)

    gripper_name = cfg.gripper_name or (profile.gripper_name if profile is not None else None)
    if gripper_name is None:
        raise ValueError("adapter.gripper_name is required for custom IK profiles.")

    return ResolvedKinematicsConfig(
        urdf_path=urdf_path,
        target_frame_name=target_frame_name,
        motor_names=list(motor_names),
        joint_names=joint_names,
        gripper_name=gripper_name,
    )


def ordered_joint_vector(
    observation: RobotObservation, motor_names: list[str], *, allow_missing_gripper: str | None = None
) -> np.ndarray:
    values: list[float] = []
    missing: list[str] = []
    for name in motor_names:
        key = f"{name}.pos"
        if key not in observation:
            if allow_missing_gripper is not None and name == allow_missing_gripper:
                continue
            missing.append(key)
            continue
        values.append(float(observation[key]))
    if missing:
        raise KeyError(f"Observation is missing joint position keys required for IK/FK: {missing}")
    return np.asarray(values, dtype=float)


class SpaceMouseDeltaToEndEffector(RobotActionProcessorStep):
    """Integrate SpaceMouse deltas from the current FK pose into an absolute EE target."""

    def __init__(
        self,
        *,
        kinematics: RobotKinematics,
        motor_names: list[str],
        translation_step_m: float,
        rotation_step_rad: float,
    ):
        self.kinematics = kinematics
        self.motor_names = motor_names
        self.translation_step_m = translation_step_m
        self.rotation_step_rad = rotation_step_rad
        self._target: np.ndarray | None = None

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            raise ValueError("Robot observation is required to compute end-effector deltas.")

        q_raw = ordered_joint_vector(observation, self.motor_names)
        current = self.kinematics.forward_kinematics(q_raw)
        enabled = bool(action.get("enabled", True))

        if not enabled:
            self._target = None
            desired = current
        else:
            if self._target is None:
                self._target = np.array(current, dtype=float, copy=True)
            delta_p = np.asarray(
                [
                    float(action.get("target_x", 0.0)) * self.translation_step_m,
                    float(action.get("target_y", 0.0)) * self.translation_step_m,
                    float(action.get("target_z", 0.0)) * self.translation_step_m,
                ],
                dtype=float,
            )
            delta_r = np.asarray(
                [
                    float(action.get("target_wx", 0.0)) * self.rotation_step_rad,
                    float(action.get("target_wy", 0.0)) * self.rotation_step_rad,
                    float(action.get("target_wz", 0.0)) * self.rotation_step_rad,
                ],
                dtype=float,
            )
            self._target[:3, 3] = self._target[:3, 3] + delta_p
            self._target[:3, :3] = self._target[:3, :3] @ Rotation.from_rotvec(delta_r).as_matrix()
            desired = self._target

        rotvec = Rotation.from_matrix(desired[:3, :3]).as_rotvec()
        return {
            "enabled": enabled,
            "ee.x": float(desired[0, 3]),
            "ee.y": float(desired[1, 3]),
            "ee.z": float(desired[2, 3]),
            "ee.wx": float(rotvec[0]),
            "ee.wy": float(rotvec[1]),
            "ee.wz": float(rotvec[2]),
            "ee.gripper_vel": float(action.get("gripper_vel", 0.0)) if enabled else 0.0,
        }

    def reset(self) -> None:
        self._target = None

    def transform_features(self, features):
        return features


class DirectEndEffectorDelta(RobotActionProcessorStep):
    """Integrate SpaceMouse deltas from EE pose keys already present in the observation."""

    def __init__(self, *, translation_step_m: float, rotation_step_rad: float):
        self.translation_step_m = translation_step_m
        self.rotation_step_rad = rotation_step_rad

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            raise ValueError("Robot observation with ee.* keys is required for direct EEF mode.")

        required = ["ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz"]
        missing = [key for key in required if key not in observation]
        if missing:
            raise KeyError(f"Direct EEF mode requires observation keys: {missing}")

        enabled = bool(action.get("enabled", True))
        delta_p = np.zeros(3, dtype=float)
        delta_r = np.zeros(3, dtype=float)
        if enabled:
            delta_p = np.asarray(
                [
                    float(action.get("target_x", 0.0)) * self.translation_step_m,
                    float(action.get("target_y", 0.0)) * self.translation_step_m,
                    float(action.get("target_z", 0.0)) * self.translation_step_m,
                ],
                dtype=float,
            )
            delta_r = np.asarray(
                [
                    float(action.get("target_wx", 0.0)) * self.rotation_step_rad,
                    float(action.get("target_wy", 0.0)) * self.rotation_step_rad,
                    float(action.get("target_wz", 0.0)) * self.rotation_step_rad,
                ],
                dtype=float,
            )

        current_rot = Rotation.from_rotvec(
            [float(observation["ee.wx"]), float(observation["ee.wy"]), float(observation["ee.wz"])]
        ).as_matrix()
        desired_rot = current_rot @ Rotation.from_rotvec(delta_r).as_matrix()
        desired_rotvec = Rotation.from_matrix(desired_rot).as_rotvec()
        return {
            "enabled": enabled,
            "ee.x": float(observation["ee.x"]) + float(delta_p[0]),
            "ee.y": float(observation["ee.y"]) + float(delta_p[1]),
            "ee.z": float(observation["ee.z"]) + float(delta_p[2]),
            "ee.wx": float(desired_rotvec[0]),
            "ee.wy": float(desired_rotvec[1]),
            "ee.wz": float(desired_rotvec[2]),
            "ee.gripper_vel": float(action.get("gripper_vel", 0.0)) if enabled else 0.0,
        }

    def transform_features(self, features):
        return features


class EndEffectorBounds(RobotActionProcessorStep):
    """Clamp and rate-limit end-effector position commands."""

    def __init__(
        self,
        *,
        workspace_min: list[float] | None,
        workspace_max: list[float] | None,
        max_ee_step_m: float,
    ):
        self.workspace_min = np.asarray(workspace_min, dtype=float) if workspace_min is not None else None
        self.workspace_max = np.asarray(workspace_max, dtype=float) if workspace_max is not None else None
        self.max_ee_step_m = max_ee_step_m
        self._last_pos: np.ndarray | None = None

    def action(self, action: RobotAction) -> RobotAction:
        pos = np.asarray([float(action["ee.x"]), float(action["ee.y"]), float(action["ee.z"])], dtype=float)
        if self.workspace_min is not None or self.workspace_max is not None:
            min_v = self.workspace_min if self.workspace_min is not None else -np.inf
            max_v = self.workspace_max if self.workspace_max is not None else np.inf
            pos = np.clip(pos, min_v, max_v)

        if self._last_pos is not None and self.max_ee_step_m > 0.0:
            diff = pos - self._last_pos
            norm = float(np.linalg.norm(diff))
            if norm > self.max_ee_step_m:
                pos = self._last_pos + diff * (self.max_ee_step_m / norm)

        self._last_pos = pos
        action["ee.x"] = float(pos[0])
        action["ee.y"] = float(pos[1])
        action["ee.z"] = float(pos[2])
        return action

    def reset(self) -> None:
        self._last_pos = None

    def transform_features(self, features):
        return features


class GripperVelocityToPosition(RobotActionProcessorStep):
    """Integrate gripper velocity into a gripper position command."""

    def __init__(self, *, gripper_key: str, speed_factor: float, clip_min: float, clip_max: float):
        self.gripper_key = gripper_key
        self.speed_factor = speed_factor
        self.clip_min = clip_min
        self.clip_max = clip_max

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            raise ValueError("Robot observation is required to integrate gripper velocity.")
        if self.gripper_key not in observation:
            raise KeyError(f"Observation is missing gripper key {self.gripper_key!r}.")
        delta = float(action.pop("ee.gripper_vel", 0.0)) * self.speed_factor
        action["ee.gripper_pos"] = float(np.clip(float(observation[self.gripper_key]) + delta, self.clip_min, self.clip_max))
        return action

    def transform_features(self, features):
        return features


class EndEffectorToJoints(RobotActionProcessorStep):
    """Convert an absolute end-effector target to joint position actions."""

    def __init__(
        self,
        *,
        kinematics: RobotKinematics,
        motor_names: list[str],
        gripper_name: str,
        position_weight: float,
        orientation_weight: float,
        initial_guess_current_joints: bool,
    ):
        self.kinematics = kinematics
        self.motor_names = motor_names
        self.gripper_name = gripper_name
        self.position_weight = position_weight
        self.orientation_weight = orientation_weight
        self.initial_guess_current_joints = initial_guess_current_joints
        self._q_curr: np.ndarray | None = None

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            raise ValueError("Robot observation is required for IK.")

        q_raw = ordered_joint_vector(observation, self.motor_names)
        if not bool(action.get("enabled", True)):
            self._q_curr = None
            return {f"{name}.pos": float(observation[f"{name}.pos"]) for name in self.motor_names}

        if self.initial_guess_current_joints or self._q_curr is None:
            self._q_curr = q_raw

        target = np.eye(4, dtype=float)
        target[:3, :3] = Rotation.from_rotvec(
            [float(action["ee.wx"]), float(action["ee.wy"]), float(action["ee.wz"])]
        ).as_matrix()
        target[:3, 3] = [float(action["ee.x"]), float(action["ee.y"]), float(action["ee.z"])]

        q_target = self.kinematics.inverse_kinematics(
            self._q_curr,
            target,
            position_weight=self.position_weight,
            orientation_weight=self.orientation_weight,
        )
        self._q_curr = q_target

        gripper_pos = float(action.get("ee.gripper_pos", observation.get(f"{self.gripper_name}.pos", 0.0)))
        robot_action: RobotAction = {}
        for idx, name in enumerate(self.motor_names):
            if name == self.gripper_name:
                robot_action[f"{name}.pos"] = gripper_pos
            else:
                robot_action[f"{name}.pos"] = float(q_target[idx])
        return robot_action

    def reset(self) -> None:
        self._q_curr = None

    def transform_features(self, features):
        return features


class FilterActionKeys(RobotActionProcessorStep):
    def __init__(self, allowed_keys: list[str]):
        self.allowed_keys = set(allowed_keys)

    def action(self, action: RobotAction) -> RobotAction:
        return {key: value for key, value in action.items() if key in self.allowed_keys}

    def transform_features(self, features):
        return features


def robot_accepts_direct_eef(robot: Robot) -> bool:
    keys = set(robot.action_features)
    return {"ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz"}.issubset(keys)


def make_spacemouse_robot_action_processor(
    cfg: SpaceMouseAdapterConfig,
    robot: Robot,
) -> RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction]:
    mode = cfg.mode
    if mode == "auto":
        mode = "eef" if robot_accepts_direct_eef(robot) else "ik"

    steps: list[RobotActionProcessorStep] = []
    if mode == "eef":
        steps.append(
            DirectEndEffectorDelta(
                translation_step_m=cfg.translation_step_m,
                rotation_step_rad=cfg.rotation_step_rad,
            )
        )
        steps.append(
            EndEffectorBounds(
                workspace_min=cfg.workspace_min,
                workspace_max=cfg.workspace_max,
                max_ee_step_m=cfg.max_ee_step_m,
            )
        )
        if "ee.gripper_pos" in robot.action_features:
            steps.append(
                GripperVelocityToPosition(
                    gripper_key="ee.gripper_pos",
                    speed_factor=cfg.gripper_speed_factor,
                    clip_min=cfg.gripper_min,
                    clip_max=cfg.gripper_max,
                )
            )
        steps.append(FilterActionKeys(list(robot.action_features)))
    elif mode == "ik":
        kinematics_cfg = resolve_kinematics_config(cfg)
        kinematics = RobotKinematics(
            urdf_path=kinematics_cfg.urdf_path,
            target_frame_name=kinematics_cfg.target_frame_name,
            joint_names=kinematics_cfg.joint_names,
        )
        steps.extend(
            [
                SpaceMouseDeltaToEndEffector(
                    kinematics=kinematics,
                    motor_names=kinematics_cfg.motor_names,
                    translation_step_m=cfg.translation_step_m,
                    rotation_step_rad=cfg.rotation_step_rad,
                ),
                EndEffectorBounds(
                    workspace_min=cfg.workspace_min,
                    workspace_max=cfg.workspace_max,
                    max_ee_step_m=cfg.max_ee_step_m,
                ),
                GripperVelocityToPosition(
                    gripper_key=f"{kinematics_cfg.gripper_name}.pos",
                    speed_factor=cfg.gripper_speed_factor,
                    clip_min=cfg.gripper_min,
                    clip_max=cfg.gripper_max,
                ),
                EndEffectorToJoints(
                    kinematics=kinematics,
                    motor_names=kinematics_cfg.motor_names,
                    gripper_name=kinematics_cfg.gripper_name,
                    position_weight=cfg.position_weight,
                    orientation_weight=cfg.orientation_weight,
                    initial_guess_current_joints=cfg.initial_guess_current_joints,
                ),
            ]
        )
    else:
        raise ValueError(f"Unsupported adapter mode: {mode}")

    return RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        steps=steps,
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )
