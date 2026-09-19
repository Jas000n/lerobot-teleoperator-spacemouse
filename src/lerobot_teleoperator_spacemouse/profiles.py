from __future__ import annotations

from dataclasses import dataclass
from importlib import resources

DEFAULT_SOARM_MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


@dataclass(frozen=True)
class KinematicsProfile:
    """Defaults needed to adapt SpaceMouse Cartesian deltas to one robot family."""

    urdf_resource: str | None
    target_frame_name: str
    motor_names: tuple[str, ...]
    joint_names: tuple[str, ...] | None = None
    gripper_name: str = "gripper"


SO101_FOLLOWER_PROFILE = KinematicsProfile(
    urdf_resource="so101_new_calib.urdf",
    target_frame_name="gripper_frame_link",
    motor_names=DEFAULT_SOARM_MOTOR_NAMES,
)

_KINEMATICS_PROFILES: dict[str, KinematicsProfile] = {
    "so101_follower": SO101_FOLLOWER_PROFILE,
    "soarm_follower": SO101_FOLLOWER_PROFILE,
}


def register_kinematics_profile(name: str, profile: KinematicsProfile) -> None:
    """Register a robot profile from another package or local extension module."""

    _KINEMATICS_PROFILES[name] = profile


def get_kinematics_profile(name: str) -> KinematicsProfile | None:
    if name == "custom":
        return None
    try:
        return _KINEMATICS_PROFILES[name]
    except KeyError as exc:
        known = ", ".join(sorted([*_KINEMATICS_PROFILES, "custom"]))
        raise ValueError(f"Unknown SpaceMouse adapter robot_profile {name!r}. Known profiles: {known}") from exc


def bundled_urdf_path(filename: str) -> str:
    return str(resources.files("lerobot_teleoperator_spacemouse").joinpath("urdfs", filename))
