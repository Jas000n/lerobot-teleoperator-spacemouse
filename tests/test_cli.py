import sys
from types import SimpleNamespace

import pytest

from lerobot_teleoperator_spacemouse.cli import test_main as run_device_test


def test_device_cli_prints_hidapi_hint_without_traceback(monkeypatch):
    def missing_hidapi():
        raise RuntimeError("HID API is probably not installed")

    driver = SimpleNamespace(get_connected_devices=missing_hidapi)
    monkeypatch.setitem(sys.modules, "pyspacemouse", driver)
    monkeypatch.setattr(sys, "argv", ["lerobot-teleoperator-spacemouse-test"])

    with pytest.raises(SystemExit, match="conda install -c conda-forge libhidapi"):
        run_device_test()
