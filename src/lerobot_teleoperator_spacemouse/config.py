from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig

from .adapter import SpaceMouseAdapterConfig


@TeleoperatorConfig.register_subclass("spacemouse")
@dataclass(kw_only=True)
class SpaceMouseTeleopConfig(TeleoperatorConfig):
    """Configuration for a 3Dconnexion SpaceMouse teleoperator."""

    device: str | None = None
    deadzone: float = 0.08
    rescale_after_deadzone: bool = True
    max_axis_value: float = 1.0
    input_timeout_s: float = 0.08
    read_drain_count: int = 32

    x_axis: str = "y"
    y_axis: str = "x"
    z_axis: str = "z"
    wx_axis: str = "roll"
    wy_axis: str = "pitch"
    wz_axis: str = "yaw"

    x_sign: float = 1.0
    y_sign: float = -1.0
    z_sign: float = 1.0
    wx_sign: float = 1.0
    wy_sign: float = 1.0
    wz_sign: float = 1.0

    use_gripper: bool = True
    gripper_open_button: int | None = 1
    gripper_close_button: int | None = 0

    require_enable_button: bool = False
    enable_button: int | None = None

    # Used when read() returns None or the device is idle.
    idle_enabled: bool = False

    # Converts SpaceMouse Cartesian deltas to robot-native actions inside lerobot-teleoperate.
    adapter: SpaceMouseAdapterConfig = field(default_factory=SpaceMouseAdapterConfig)
