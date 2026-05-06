from __future__ import annotations

import importlib
import time
from typing import Any

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.types import RobotAction

from .config import SpaceMouseTeleopConfig

ACTION_KEYS = (
    "enabled",
    "target_x",
    "target_y",
    "target_z",
    "target_wx",
    "target_wy",
    "target_wz",
    "gripper_vel",
)

HIDAPI_INSTALL_HINT = (
    "SpaceMouse HID library is not available. In a conda environment, install it with "
    "`conda install -c conda-forge libhidapi`. On Debian/Ubuntu, install "
    "`sudo apt-get install libhidapi-hidraw0 libhidapi-dev`."
)

HIDRAW_PERMISSION_HINT = (
    "SpaceMouse was detected but could not be opened. On Linux this usually means hidraw permissions "
    "are missing. Add a udev rule for 3Dconnexion devices, reload udev, then unplug/replug the SpaceMouse. "
    "If using GROUP=plugdev, log out and back in after adding your user to plugdev."
)


def apply_deadzone(value: float, deadzone: float, *, rescale: bool = True) -> float:
    if abs(value) <= deadzone:
        return 0.0
    if not rescale or deadzone >= 1.0:
        return value
    sign = 1.0 if value >= 0.0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def get_buttons(state: Any) -> list[int]:
    buttons = getattr(state, "button", None)
    if buttons is None:
        buttons = getattr(state, "buttons", None)
    return list(buttons or [])


def button_pressed(buttons: list[int], index: int | None) -> bool:
    return index is not None and 0 <= index < len(buttons) and bool(buttons[index])


def axis_value(state: Any, axis: str, sign: float, cfg: SpaceMouseTeleopConfig) -> float:
    raw = float(getattr(state, axis, 0.0) or 0.0)
    if cfg.max_axis_value > 0.0:
        raw = max(-cfg.max_axis_value, min(cfg.max_axis_value, raw))
    return sign * apply_deadzone(raw, cfg.deadzone, rescale=cfg.rescale_after_deadzone)


def state_is_stale(state: Any, cfg: SpaceMouseTeleopConfig, *, now: float) -> bool:
    state_t = getattr(state, "t", None)
    return state_t is not None and state_t >= 0.0 and cfg.input_timeout_s > 0.0 and now - float(state_t) > cfg.input_timeout_s


def state_to_action(state: Any | None, cfg: SpaceMouseTeleopConfig, *, now: float | None = None) -> RobotAction:
    if state is None:
        return zero_action(enabled=cfg.idle_enabled)
    now = time.perf_counter() if now is None else now
    if state_is_stale(state, cfg, now=now):
        return zero_action(enabled=cfg.idle_enabled)

    buttons = get_buttons(state)
    gripper_vel = 0.0
    if cfg.use_gripper:
        if button_pressed(buttons, cfg.gripper_open_button):
            gripper_vel += 1.0
        if button_pressed(buttons, cfg.gripper_close_button):
            gripper_vel -= 1.0

    action = {
        "target_x": axis_value(state, cfg.x_axis, cfg.x_sign, cfg),
        "target_y": axis_value(state, cfg.y_axis, cfg.y_sign, cfg),
        "target_z": axis_value(state, cfg.z_axis, cfg.z_sign, cfg),
        "target_wx": axis_value(state, cfg.wx_axis, cfg.wx_sign, cfg),
        "target_wy": axis_value(state, cfg.wy_axis, cfg.wy_sign, cfg),
        "target_wz": axis_value(state, cfg.wz_axis, cfg.wz_sign, cfg),
        "gripper_vel": gripper_vel,
    }

    if cfg.require_enable_button:
        enabled = button_pressed(buttons, cfg.enable_button)
    else:
        enabled = any(abs(float(action[key])) > 0.0 for key in action)

    return {"enabled": enabled, **action}


def zero_action(*, enabled: bool = False) -> RobotAction:
    return {
        "enabled": enabled,
        "target_x": 0.0,
        "target_y": 0.0,
        "target_z": 0.0,
        "target_wx": 0.0,
        "target_wy": 0.0,
        "target_wz": 0.0,
        "gripper_vel": 0.0,
    }


def explain_open_error(exc: RuntimeError) -> RuntimeError:
    message = str(exc)
    if "HID API" in message or "hid_enumerate" in message:
        return RuntimeError(HIDAPI_INSTALL_HINT)
    if "Failed to open device" in message:
        return PermissionError(HIDRAW_PERMISSION_HINT)
    return exc


def open_spacemouse_device(driver: Any, device: str | None = None) -> Any:
    try:
        return driver.open(device=device)
    except RuntimeError as exc:
        raise explain_open_error(exc) from exc


def read_latest_state(device: Any, driver: Any | None, max_reads: int) -> Any | None:
    state = None
    last_t = object()
    for _ in range(max(1, max_reads)):
        if hasattr(device, "read"):
            next_state = device.read()
        elif driver is not None:
            next_state = driver.read()
        else:
            return state

        if next_state is None:
            return state
        state = next_state
        state_t = getattr(state, "t", None)
        if state_t == last_t:
            break
        last_t = state_t
    return state


class SpaceMouseTeleop(Teleoperator):
    """LeRobot teleoperator that reads 6-DoF commands from a 3Dconnexion SpaceMouse."""

    config_class = SpaceMouseTeleopConfig
    name = "spacemouse"

    def __init__(self, config: SpaceMouseTeleopConfig):
        super().__init__(config)
        self.config = config
        self._driver: Any | None = None
        self._device: Any | None = None
        self._last_read_t = 0.0

    @property
    def action_features(self) -> dict[str, type]:
        return {
            "enabled": bool,
            "target_x": float,
            "target_y": float,
            "target_z": float,
            "target_wx": float,
            "target_wy": float,
            "target_wz": float,
            "gripper_vel": float,
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        if self._device is None:
            return False
        return bool(getattr(self._device, "connected", True))

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        self._driver = importlib.import_module("pyspacemouse")
        self._device = open_spacemouse_device(self._driver, self.config.device)
        if self._device is None:
            devices = []
            if hasattr(self._driver, "list_devices"):
                devices = self._driver.list_devices()
            raise ConnectionError(f"No supported SpaceMouse device found. Detected devices: {devices}")
        self.configure()

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_action(self) -> RobotAction:
        if self._device is None:
            raise ConnectionError("SpaceMouseTeleop is not connected.")

        self._last_read_t = time.perf_counter()
        state = read_latest_state(self._device, self._driver, self.config.read_drain_count)
        return state_to_action(state, self.config, now=self._last_read_t)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback
        return None

    def disconnect(self) -> None:
        if self._device is not None and hasattr(self._device, "close"):
            self._device.close()
        elif self._driver is not None and hasattr(self._driver, "close"):
            self._driver.close()
        self._device = None
