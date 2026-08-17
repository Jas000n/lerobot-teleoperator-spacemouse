from types import SimpleNamespace

import pytest

from lerobot_teleoperator_spacemouse.config import SpaceMouseTeleopConfig
from lerobot_teleoperator_spacemouse.spacemouse import (
    SpaceMouseTeleop,
    apply_deadzone,
    list_connected_devices,
    open_spacemouse_device,
    read_latest_state,
    state_to_action,
)


def test_apply_deadzone_rescales_remaining_range():
    assert apply_deadzone(0.02, 0.04) == 0.0
    assert round(apply_deadzone(0.52, 0.04), 6) == 0.5
    assert round(apply_deadzone(-0.52, 0.04), 6) == -0.5


def test_state_to_action_maps_axes_and_buttons():
    cfg = SpaceMouseTeleopConfig(deadzone=0.0, gripper_open_button=1, gripper_close_button=0)
    state = SimpleNamespace(t=1.0, x=0.1, y=-0.2, z=0.3, roll=0.4, pitch=-0.5, yaw=0.6, button=[0, 1])

    action = state_to_action(state, cfg, now=1.01)

    assert action["enabled"] is True
    assert action["target_x"] == -0.2
    assert action["target_y"] == -0.1
    assert action["target_z"] == 0.3
    assert action["target_wx"] == 0.4
    assert action["target_wy"] == -0.5
    assert action["target_wz"] == 0.6
    assert action["gripper_vel"] == 1.0


def test_state_to_action_clamps_axis_values():
    cfg = SpaceMouseTeleopConfig(deadzone=0.0, max_axis_value=1.0)
    state = SimpleNamespace(t=1.0, x=3.0, y=-2.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, button=[])

    action = state_to_action(state, cfg, now=1.01)

    assert action["target_x"] == -1.0
    assert action["target_y"] == -1.0


def test_state_to_action_stops_stale_input():
    cfg = SpaceMouseTeleopConfig(deadzone=0.0, input_timeout_s=0.08)
    state = SimpleNamespace(t=1.0, x=1.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, button=[])

    action = state_to_action(state, cfg, now=1.2)

    assert action == {
        "enabled": False,
        "target_x": 0.0,
        "target_y": 0.0,
        "target_z": 0.0,
        "target_wx": 0.0,
        "target_wy": 0.0,
        "target_wz": 0.0,
        "gripper_vel": 0.0,
    }


def test_idle_enabled_applies_to_fresh_zero_state():
    cfg = SpaceMouseTeleopConfig(deadzone=0.0, idle_enabled=True)
    state = SimpleNamespace(t=1.0, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, button=[])

    action = state_to_action(state, cfg, now=1.01)

    assert action["enabled"] is True


def test_read_latest_state_drains_until_timestamp_stops():
    states = [
        SimpleNamespace(t=1.0, x=1.0),
        SimpleNamespace(t=1.1, x=0.5),
        SimpleNamespace(t=1.2, x=0.0),
        SimpleNamespace(t=1.2, x=0.0),
    ]

    class Device:
        def read(self):
            return states.pop(0)

    latest = read_latest_state(Device(), None, max_reads=32)

    assert latest.t == 1.2
    assert latest.x == 0.0
    assert len(states) == 0


def test_connect_explains_missing_hidapi(monkeypatch):
    class MissingHidDriver:
        @staticmethod
        def open(device=None):
            del device
            raise RuntimeError("HID API is probably not installed")

    monkeypatch.setattr(
        "lerobot_teleoperator_spacemouse.spacemouse.importlib.import_module",
        lambda name: MissingHidDriver if name == "pyspacemouse" else None,
    )

    teleop = SpaceMouseTeleop(SpaceMouseTeleopConfig())
    with pytest.raises(RuntimeError, match="conda install -c conda-forge libhidapi"):
        teleop.connect()


def test_connect_explains_hidraw_permissions(monkeypatch):
    class PermissionDeniedDriver:
        @staticmethod
        def open(device=None):
            del device
            raise RuntimeError("Failed to open device")

    monkeypatch.setattr(
        "lerobot_teleoperator_spacemouse.spacemouse.importlib.import_module",
        lambda name: PermissionDeniedDriver if name == "pyspacemouse" else None,
    )

    teleop = SpaceMouseTeleop(SpaceMouseTeleopConfig())
    with pytest.raises(PermissionError, match="hidraw permissions"):
        teleop.connect()


def test_open_supports_device_paths_on_both_pyspacemouse_apis():
    class NewDriver:
        @staticmethod
        def open_by_path(path):
            return ("new", path)

    class OldDriver:
        @staticmethod
        def open(*, path):
            return ("old", path)

    assert open_spacemouse_device(NewDriver, "/dev/hidraw2") == ("new", "/dev/hidraw2")
    assert open_spacemouse_device(OldDriver, "/dev/hidraw2") == ("old", "/dev/hidraw2")


def test_old_pyspacemouse_exception_gets_permission_hint():
    class OldDriver:
        @staticmethod
        def open(device=None):
            del device
            raise Exception("Failed to open device")

    with pytest.raises(PermissionError, match="hidraw permissions"):
        open_spacemouse_device(OldDriver)


def test_list_connected_devices_supports_both_pyspacemouse_apis():
    new_driver = SimpleNamespace(get_connected_devices=lambda: ["SpaceMouse Compact"])
    old_driver = SimpleNamespace(list_devices=lambda: ["SpaceNavigator"])

    assert list_connected_devices(new_driver) == ["SpaceMouse Compact"]
    assert list_connected_devices(old_driver) == ["SpaceNavigator"]


def test_list_connected_devices_explains_missing_hidapi():
    def missing_hidapi():
        raise RuntimeError("HID API is probably not installed")

    driver = SimpleNamespace(get_connected_devices=missing_hidapi)

    with pytest.raises(RuntimeError, match="conda install -c conda-forge libhidapi"):
        list_connected_devices(driver)
