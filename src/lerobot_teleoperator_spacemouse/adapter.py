from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.robot import Robot
from lerobot.utils.rotation import Rotation

from ._lerobot_compat import (
    RobotAction,
    RobotActionProcessorStep,
    RobotObservation,
    RobotProcessorPipeline,
    TransitionKey,
)
from .profiles import DEFAULT_SOARM_MOTOR_NAMES, bundled_urdf_path, get_kinematics_profile

__all__ = [
    "DEFAULT_SOARM_MOTOR_NAMES",
    "VirtualLeaderArm",
    "urdf_joint_limits_deg",
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

    # "ik", "eef", "virtual_leader" or "auto" (picks "eef" when the robot exposes ee.* keys,
    # otherwise "ik"). "virtual_leader" drives the follower from a virtual leader arm, so the
    # follower's sag and stick-slip never re-enter the command - see VirtualLeaderArm.
    mode: str = "auto"

    # IK mode.
    robot_profile: str = "so101_follower"
    urdf_path: str | None = None
    target_frame_name: str | None = None
    motor_names: list[str] | None = None
    joint_names: list[str] | None = None
    gripper_name: str | None = None

    # Per control-frame deltas at full SpaceMouse deflection. 0.002 m/frame (60 mm/s at 30 fps)
    # was measured to be the sweet spot on an SO-101: a Feetech STS3215 stick-slips at low speed,
    # and sweeping the step over a logged descent gave 51% / 39% / 32% of the time stalled at
    # 0.001 / 0.002 / 0.003, while the speed ripple bottomed out at 0.002 (CoV 0.35 / 0.33 / 0.47).
    translation_step_m: float = 0.002
    rotation_step_rad: float = 0.005

    # Optional workspace clamp. Leave unset while calibrating axes.
    workspace_min: list[float] | None = None
    workspace_max: list[float] | None = None
    max_ee_step_m: float = 0.02

    # Opt-in: keep commanding the last target while the SpaceMouse is idle instead of re-deriving
    # it from the measured pose. Stops the arm sagging on release, but the target then has to be
    # leashed (below) so it cannot run away from an arm that is slower than the commanded motion.
    hold_target_when_idle: bool = False
    # Runaway guard: how far the integrated target may lead the measured pose. This is a safety
    # net, not a tracking parameter - a follower trails its command by a real amount (on an SO-101
    # the end-effector error runs to ~20 mm), and a cap anywhere near that range fights the arm
    # instead of catching a runaway. 0 disables the guard.
    max_target_lead_m: float = 0.06
    max_target_lead_rad: float = 1.0

    # virtual_leader mode. 8 deg/frame (240 deg/s) never triggers in ordinary teleoperation:
    # measured over 2727 recorded frames the largest command step was 6.0 deg.
    max_joint_step_deg: float = 8.0
    use_urdf_joint_limits: bool = True

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


def urdf_joint_limits_deg(cfg: ResolvedKinematicsConfig) -> dict[str, tuple[float, float]]:
    """Read each joint's travel from the URDF, so the virtual arm has end stops like a real one."""
    import xml.etree.ElementTree as ET

    limits: dict[str, tuple[float, float]] = {}
    root = ET.parse(cfg.urdf_path).getroot()
    for joint in root.iter("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name in cfg.motor_names and limit is not None:
            lower, upper = limit.get("lower"), limit.get("upper")
            if lower is not None and upper is not None:
                limits[name] = (float(np.rad2deg(float(lower))), float(np.rad2deg(float(upper))))
    return limits


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
        workspace_min: list[float] | None = None,
        workspace_max: list[float] | None = None,
        max_ee_step_m: float = 0.0,
        hold_target_when_idle: bool = False,
        max_target_lead_m: float = 0.0,
        max_target_lead_rad: float = 0.0,
    ):
        self.hold_target_when_idle = hold_target_when_idle
        self.max_target_lead_m = max_target_lead_m
        self.max_target_lead_rad = max_target_lead_rad
        self.kinematics = kinematics
        self.motor_names = motor_names
        self.translation_step_m = translation_step_m
        self.rotation_step_rad = rotation_step_rad
        self.workspace_min = np.asarray(workspace_min, dtype=float) if workspace_min is not None else None
        self.workspace_max = np.asarray(workspace_max, dtype=float) if workspace_max is not None else None
        self.max_ee_step_m = max_ee_step_m
        self._target: np.ndarray | None = None

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            raise ValueError("Robot observation is required to compute end-effector deltas.")

        q_raw = ordered_joint_vector(observation, self.motor_names)
        current = self.kinematics.forward_kinematics(q_raw)
        enabled = bool(action.get("enabled", True))

        if not enabled and not self.hold_target_when_idle:
            # Legacy behaviour: re-latch onto the measured pose, which also happens to bound how
            # far the target can lead the arm. See _leash for the replacement bound.
            self._target = None
            desired = current
        else:
            if self._target is None:
                self._target = np.array(current, dtype=float, copy=True)
            delta_p = np.asarray(
                [
                    float(action.get("target_x", 0.0)) * self.translation_step_m if enabled else 0.0,
                    float(action.get("target_y", 0.0)) * self.translation_step_m if enabled else 0.0,
                    float(action.get("target_z", 0.0)) * self.translation_step_m if enabled else 0.0,
                ],
                dtype=float,
            )
            delta_norm = float(np.linalg.norm(delta_p))
            if self.max_ee_step_m > 0.0 and delta_norm > self.max_ee_step_m:
                delta_p = delta_p * (self.max_ee_step_m / delta_norm)
            delta_r = np.asarray(
                [
                    float(action.get("target_wx", 0.0)) * self.rotation_step_rad,
                    float(action.get("target_wy", 0.0)) * self.rotation_step_rad,
                    float(action.get("target_wz", 0.0)) * self.rotation_step_rad,
                ],
                dtype=float,
            )
            delta_p, delta_r = self._limit_lead(delta_p, delta_r, current)
            self._target[:3, 3] = self._target[:3, 3] + delta_p
            if self.workspace_min is not None or self.workspace_max is not None:
                min_v = self.workspace_min if self.workspace_min is not None else -np.inf
                max_v = self.workspace_max if self.workspace_max is not None else np.inf
                self._target[:3, 3] = np.clip(self._target[:3, 3], min_v, max_v)
            if enabled:
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

    def _limit_lead(self, delta_p: np.ndarray, delta_r: np.ndarray, current: np.ndarray):
        """Stop the target moving further away from an arm that is already too far behind.

        The target is an open-loop integrator, so if the arm cannot keep up the gap grows every
        frame until the IK is stepping across the workspace in one go. The legacy idle branch
        bounded the gap only as a side effect of throwing the target away on every release.

        This refuses the outward part of the delta instead of pulling the target back: dragging the
        target towards the arm would tie it to the measured pose, turning a position integrator
        into a constant-error velocity loop that surges and chatters. Motion that shortens the gap
        is always allowed, so the operator can never get stuck.
        """
        if self.max_target_lead_m > 0.0:
            lead = self._target[:3, 3] - current[:3, 3]
            distance = float(np.linalg.norm(lead))
            if distance > self.max_target_lead_m:
                direction = lead / distance
                outward = float(np.dot(delta_p, direction))
                if outward > 0.0:
                    delta_p = delta_p - direction * outward
        if self.max_target_lead_rad > 0.0:
            lead_r = Rotation.from_matrix(current[:3, :3].T @ self._target[:3, :3]).as_rotvec()
            angle = float(np.linalg.norm(lead_r))
            if angle > self.max_target_lead_rad:
                direction = lead_r / angle
                outward = float(np.dot(delta_r, direction))
                if outward > 0.0:
                    delta_r = delta_r - direction * outward
        return delta_p, delta_r

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
                    float(action.get("target_x", 0.0)) * self.translation_step_m if enabled else 0.0,
                    float(action.get("target_y", 0.0)) * self.translation_step_m if enabled else 0.0,
                    float(action.get("target_z", 0.0)) * self.translation_step_m if enabled else 0.0,
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
        # Disabled input means "hold the measured pose". Do not clamp or rate-limit
        # that pose: direct-EEF robots may not expose the internal ``enabled`` key,
        # and retaining the old position here would also make the next enable start
        # from a stale limiter state.
        if not bool(action.get("enabled", True)):
            self._last_pos = None
            return action

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
        hold_target_when_idle: bool = False,
    ):
        self.hold_target_when_idle = hold_target_when_idle
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
        if not bool(action.get("enabled", True)) and not self.hold_target_when_idle:
            self._q_curr = None
            return {f"{name}.pos": float(observation[f"{name}.pos"]) for name in self.motor_names}
        # In hold mode idle frames take the same path as active ones: one command source only, so
        # a flickering `enabled` cannot make the output jump between two different laws.

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
    required = {"ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz"}
    return required.issubset(robot.action_features) and required.issubset(robot.observation_features)


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
        if "ee.gripper_pos" in robot.action_features and "ee.gripper_pos" in robot.observation_features:
            steps.append(
                GripperVelocityToPosition(
                    gripper_key="ee.gripper_pos",
                    speed_factor=cfg.gripper_speed_factor,
                    clip_min=cfg.gripper_min,
                    clip_max=cfg.gripper_max,
                )
            )
        steps.append(FilterActionKeys(list(robot.action_features)))
    elif mode == "virtual_leader":
        kinematics_cfg = resolve_kinematics_config(cfg)
        kinematics = RobotKinematics(
            urdf_path=kinematics_cfg.urdf_path,
            target_frame_name=kinematics_cfg.target_frame_name,
            joint_names=kinematics_cfg.joint_names,
        )
        steps.append(
            VirtualLeaderArm(
                kinematics=kinematics,
                motor_names=kinematics_cfg.motor_names,
                gripper_name=kinematics_cfg.gripper_name,
                translation_step_m=cfg.translation_step_m,
                rotation_step_rad=cfg.rotation_step_rad,
                gripper_speed_factor=cfg.gripper_speed_factor,
                gripper_min=cfg.gripper_min,
                gripper_max=cfg.gripper_max,
                position_weight=cfg.position_weight,
                orientation_weight=cfg.orientation_weight,
                workspace_min=cfg.workspace_min,
                workspace_max=cfg.workspace_max,
                max_joint_step_deg=cfg.max_joint_step_deg,
                joint_limits_deg=urdf_joint_limits_deg(kinematics_cfg) if cfg.use_urdf_joint_limits else None,
            )
        )
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
                    workspace_min=cfg.workspace_min,
                    workspace_max=cfg.workspace_max,
                    max_ee_step_m=cfg.max_ee_step_m,
                    hold_target_when_idle=cfg.hold_target_when_idle,
                    max_target_lead_m=cfg.max_target_lead_m,
                    max_target_lead_rad=cfg.max_target_lead_rad,
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
                    hold_target_when_idle=cfg.hold_target_when_idle,
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


class VirtualLeaderArm(RobotActionProcessorStep):
    """Drive the follower from a virtual leader arm instead of from its own measurements.

    A real leader arm works because its joint stream owes nothing to the follower: the follower can
    sag, stick or lag without any of it coming back as a command. This step reproduces that. It
    latches the follower's pose once, then keeps its own joint state, integrating the SpaceMouse
    deltas on that state alone. The measurement is read only to initialise, never to command.

    `RobotKinematics` shares one placo model between FK and IK, and `inverse_kinematics` applies
    `update_kinematics()` *after* solving, so its `current_joint_pos` argument does not seed that
    solve - whatever configuration the previous `forward_kinematics()` left behind does. Measured
    against LeRobot 0.6.1, changing only the preceding FK call moves the IK result by 32 degrees.
    So the FK below is not merely a query: it is what puts the solver on the virtual arm.
    """

    def __init__(
        self,
        *,
        kinematics: RobotKinematics,
        motor_names: list[str],
        gripper_name: str,
        translation_step_m: float,
        rotation_step_rad: float,
        gripper_speed_factor: float,
        gripper_min: float,
        gripper_max: float,
        position_weight: float,
        orientation_weight: float,
        workspace_min: list[float] | None = None,
        workspace_max: list[float] | None = None,
        max_joint_step_deg: float = 8.0,
        joint_limits_deg: dict[str, tuple[float, float]] | None = None,
    ):
        self.kinematics = kinematics
        self.motor_names = motor_names
        self.gripper_name = gripper_name
        self.translation_step_m = translation_step_m
        self.rotation_step_rad = rotation_step_rad
        self.gripper_speed_factor = gripper_speed_factor
        self.gripper_min = gripper_min
        self.gripper_max = gripper_max
        self.position_weight = position_weight
        self.orientation_weight = orientation_weight
        self.workspace_min = np.asarray(workspace_min, dtype=float) if workspace_min is not None else None
        self.workspace_max = np.asarray(workspace_max, dtype=float) if workspace_max is not None else None
        self.max_joint_step_deg = max_joint_step_deg
        self.joint_limits_deg = joint_limits_deg or {}
        self._q: np.ndarray | None = None
        self._gripper: float | None = None
        self._target: np.ndarray | None = None

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            raise ValueError("Robot observation is required to initialise the virtual leader arm.")

        if self._q is None:
            # The only time the follower is read: the virtual arm starts where the real one stands.
            self._q = ordered_joint_vector(observation, self.motor_names)
            self._gripper = float(observation.get(f"{self.gripper_name}.pos", 0.0))
            self._target = None

        # Puts the solver on the virtual configuration, and returns its pose.
        current = self.kinematics.forward_kinematics(self._q)
        if self._target is None:
            self._target = np.array(current, dtype=float, copy=True)

        if bool(action.get("enabled", True)):
            delta_p = np.asarray(
                [float(action.get(f"target_{a}", 0.0)) * self.translation_step_m for a in ("x", "y", "z")],
                dtype=float,
            )
            delta_r = np.asarray(
                [float(action.get(f"target_w{a}", 0.0)) * self.rotation_step_rad for a in ("x", "y", "z")],
                dtype=float,
            )
            self._target[:3, 3] = self._target[:3, 3] + delta_p
            if self.workspace_min is not None or self.workspace_max is not None:
                low = self.workspace_min if self.workspace_min is not None else -np.inf
                high = self.workspace_max if self.workspace_max is not None else np.inf
                self._target[:3, 3] = np.clip(self._target[:3, 3], low, high)
            self._target[:3, :3] = self._target[:3, :3] @ Rotation.from_rotvec(delta_r).as_matrix()
            self._gripper = float(
                np.clip(
                    self._gripper + float(action.get("gripper_vel", 0.0)) * self.gripper_speed_factor,
                    self.gripper_min,
                    self.gripper_max,
                )
            )

        solved = self.kinematics.inverse_kinematics(
            self._q,
            self._target,
            position_weight=self.position_weight,
            orientation_weight=self.orientation_weight,
        )

        # A leader arm is moved by a hand and stops at its own end stops; give the virtual one the
        # same two limits so a target it cannot reach cannot turn into a lurch.
        if self.max_joint_step_deg > 0.0:
            solved = self._q + np.clip(solved - self._q, -self.max_joint_step_deg, self.max_joint_step_deg)
        for index, name in enumerate(self.motor_names):
            limits = self.joint_limits_deg.get(name)
            if limits is not None:
                solved[index] = float(np.clip(solved[index], limits[0], limits[1]))

        self._q = solved
        return {
            f"{name}.pos": (float(self._gripper) if name == self.gripper_name else float(solved[index]))
            for index, name in enumerate(self.motor_names)
        }

    def reset(self) -> None:
        self._q = None
        self._gripper = None
        self._target = None

    def transform_features(self, features):
        return features
