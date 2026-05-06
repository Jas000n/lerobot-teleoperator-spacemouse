import argparse
import time

from lerobot_teleoperator_spacemouse.spacemouse import open_spacemouse_device


def test_main() -> None:
    arg_parser = argparse.ArgumentParser(description="Print SpaceMouse state.")
    arg_parser.add_argument("--device", default=None)
    arg_parser.add_argument("--duration_s", type=float, default=10.0)
    arg_parser.add_argument("--period_s", type=float, default=0.05)
    args = arg_parser.parse_args()

    import pyspacemouse

    devices = pyspacemouse.list_devices() if hasattr(pyspacemouse, "list_devices") else []
    print(f"Detected devices: {devices}")
    device = open_spacemouse_device(pyspacemouse, args.device)
    if device is None:
        raise SystemExit("No supported SpaceMouse device found.")

    start = time.perf_counter()
    try:
        while time.perf_counter() - start < args.duration_s:
            state = device.read() if hasattr(device, "read") else pyspacemouse.read()
            print(state)
            time.sleep(args.period_s)
    finally:
        if hasattr(device, "close"):
            device.close()
        else:
            pyspacemouse.close()
