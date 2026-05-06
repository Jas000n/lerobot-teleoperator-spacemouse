import argparse
import time

from lerobot_teleoperator_spacemouse.spacemouse import open_spacemouse_device, read_latest_state


def test_main() -> None:
    arg_parser = argparse.ArgumentParser(description="Print SpaceMouse state.")
    arg_parser.add_argument("--device", default=None)
    arg_parser.add_argument("--duration_s", type=float, default=30.0)
    arg_parser.add_argument("--fps", type=float, default=60.0)
    arg_parser.add_argument("--drain_count", type=int, default=32)
    args = arg_parser.parse_args()

    import pyspacemouse

    devices = pyspacemouse.list_devices() if hasattr(pyspacemouse, "list_devices") else []
    print(f"Detected devices: {devices}")
    device = open_spacemouse_device(pyspacemouse, args.device)
    if device is None:
        raise SystemExit("No supported SpaceMouse device found.")

    period_s = 1.0 / args.fps
    start = time.perf_counter()
    try:
        while time.perf_counter() - start < args.duration_s:
            t0 = time.perf_counter()
            state = read_latest_state(device, pyspacemouse, args.drain_count)
            if state is not None:
                print(state)
            elapsed = time.perf_counter() - t0
            remaining = period_s - elapsed
            if remaining > 0:
                time.sleep(remaining)
    finally:
        if hasattr(device, "close"):
            device.close()
        else:
            pyspacemouse.close()
