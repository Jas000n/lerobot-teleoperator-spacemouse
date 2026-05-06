# lerobot-teleoperator-spacemouse

[![PyPI](https://img.shields.io/pypi/v/lerobot-teleoperator-spacemouse)](https://pypi.org/project/lerobot-teleoperator-spacemouse/)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/pypi/pyversions/lerobot-teleoperator-spacemouse)](https://pypi.org/project/lerobot-teleoperator-spacemouse/)

A [LeRobot](https://github.com/huggingface/lerobot) plugin that turns a [3Dconnexion SpaceMouse](https://3dconnexion.com/spacemouse) into a 6-DoF robot teleoperator.

Push the puck to move the end-effector in X/Y/Z. Tilt/twist to rotate. Press the side buttons to open or close the gripper. Works out of the box with SO-101 / SO-ARM follower robots via built-in inverse kinematics, and supports any custom URDF through a profile API.

---

## Features

- **6-DoF teleoperation** — translation and rotation deltas from SpaceMouse physical axes
- **IK mode** — converts end-effector targets to joint commands using LeRobot's `RobotKinematics` and a bundled SO-101 URDF (no calibration file needed)
- **Direct EEF mode** — passes Cartesian targets straight to robots that already accept `ee.*` action keys
- **Gripper control** — left/right buttons open and close the gripper at a configurable speed
- **Axis remapping & sign flip** — remap any physical SpaceMouse axis to any robot axis; flip signs per axis
- **Deadzone & timeout** — configurable deadzone and input-stale timeout prevent drift when the puck is released
- **Workspace clamping** — optional per-axis position limits and max-step rate limiter
- **Custom robot profiles** — register your own URDF + frame + motor list at runtime

---

## Installation

```bash
pip install lerobot-teleoperator-spacemouse
```

SO-101 / SO-ARM IK support is included by default. For other robot families (e.g. Dynamixel-based arms), register a custom kinematics profile — see [Extending to Other Robots](#extending-to-other-robots).

---

## Quick Start

### SO-101 follower over USB

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_robot \
  --teleop.type=spacemouse \
  --teleop.adapter.mode=ik \
  --fps=60
```

### Conservative first run (smaller steps, display enabled)

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_robot \
  --teleop.type=spacemouse \
  --teleop.adapter.mode=ik \
  --teleop.deadzone=0.1 \
  --teleop.input_timeout_s=0.08 \
  --teleop.adapter.translation_step_m=0.001 \
  --teleop.adapter.rotation_step_rad=0.005 \
  --display_data=true
```

### Test your SpaceMouse (no robot needed)

```bash
lerobot-teleoperator-spacemouse-test
```

This prints raw SpaceMouse state for 30 seconds so you can verify axes and buttons before connecting a robot.

---

## Configuration Reference

All options are set via `--teleop.<option>` or `--teleop.adapter.<option>` on the command line.

### Teleoperator options (`--teleop.*`)

| Option | Default | Description |
|---|---|---|
| `device` | `None` | HID device path. Auto-detected when `None`. |
| `deadzone` | `0.08` | Ignore inputs below this fraction of full deflection. |
| `rescale_after_deadzone` | `True` | Rescale remaining range to `[0, 1]` after deadzone. |
| `max_axis_value` | `1.0` | Clamp raw axis values to `[-max, max]`. |
| `input_timeout_s` | `0.08` | Zero-out input if no new state arrives within this window (seconds). |
| `read_drain_count` | `32` | Max HID reads per `get_action()` call to drain the queue. |
| `x_axis` | `"y"` | SpaceMouse attribute name mapped to robot `target_x`. |
| `y_axis` | `"x"` | SpaceMouse attribute name mapped to robot `target_y`. |
| `z_axis` | `"z"` | SpaceMouse attribute name mapped to robot `target_z`. |
| `wx_axis` | `"roll"` | SpaceMouse attribute name mapped to robot `target_wx`. |
| `wy_axis` | `"pitch"` | SpaceMouse attribute name mapped to robot `target_wy`. |
| `wz_axis` | `"yaw"` | SpaceMouse attribute name mapped to robot `target_wz`. |
| `x_sign` | `1.0` | Sign multiplier for `target_x`. Use `-1.0` to flip. |
| `y_sign` | `-1.0` | Sign multiplier for `target_y`. |
| `z_sign` | `1.0` | Sign multiplier for `target_z`. |
| `wx_sign` | `1.0` | Sign multiplier for `target_wx`. |
| `wy_sign` | `1.0` | Sign multiplier for `target_wy`. |
| `wz_sign` | `1.0` | Sign multiplier for `target_wz`. |
| `use_gripper` | `True` | Enable gripper button control. |
| `gripper_open_button` | `1` | Button index that opens the gripper. |
| `gripper_close_button` | `0` | Button index that closes the gripper. |
| `require_enable_button` | `False` | Require a dedicated button to enable motion. |
| `idle_enabled` | `False` | Whether `enabled=True` is sent when the device is idle. |

### Adapter options (`--teleop.adapter.*`)

| Option | Default | Description |
|---|---|---|
| `mode` | `"auto"` | `"ik"`, `"eef"`, or `"auto"` (uses `"eef"` if robot exposes `ee.*` keys). |
| `robot_profile` | `"so101_follower"` | Built-in kinematics profile. Use `"custom"` to supply your own URDF. |
| `translation_step_m` | `0.001` | End-effector translation per frame at full SpaceMouse deflection (meters). |
| `rotation_step_rad` | `0.005` | End-effector rotation per frame at full deflection (radians). |
| `workspace_min` | `None` | `[x, y, z]` lower workspace bound (meters). |
| `workspace_max` | `None` | `[x, y, z]` upper workspace bound (meters). |
| `max_ee_step_m` | `0.02` | Maximum allowed end-effector step per frame (rate limiter). |
| `position_weight` | `1.0` | IK position tracking weight. |
| `orientation_weight` | `0.01` | IK orientation tracking weight. |
| `gripper_speed_factor` | `2.0` | Gripper velocity → position integration scale. |
| `gripper_min` | `0.0` | Gripper position lower bound (normalized joint units). |
| `gripper_max` | `100.0` | Gripper position upper bound (normalized joint units). |

---

## Linux HID Setup

The SpaceMouse uses the hidraw subsystem. On Linux you need the shared library and a udev rule.

**conda environment:**

```bash
conda install -c conda-forge libhidapi
```

**Debian / Ubuntu system Python:**

```bash
sudo apt-get install libhidapi-hidraw0 libhidapi-dev
```

**udev rule (allows non-root access):**

```bash
sudo tee /etc/udev/rules.d/99-spacemouse-hidraw.rules >/dev/null <<'EOF'
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", MODE="0660", GROUP="plugdev", TAG+="uaccess"
EOF
sudo usermod -aG plugdev "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug the SpaceMouse. Log out and back in after the group change.

---

## Extending to Other Robots

### One-off custom URDF

```bash
lerobot-teleoperate \
  --teleop.type=spacemouse \
  --teleop.adapter.mode=ik \
  --teleop.adapter.robot_profile=custom \
  --teleop.adapter.urdf_path=/path/to/robot.urdf \
  --teleop.adapter.target_frame_name=gripper_tip \
  --teleop.adapter.motor_names='["joint1","joint2","joint3","joint4","joint5","gripper"]' \
  --teleop.adapter.gripper_name=gripper
```

### Reusable profile (Python)

Register a profile from your own package and it becomes available as a named `robot_profile`:

```python
from lerobot_teleoperator_spacemouse.profiles import KinematicsProfile, register_kinematics_profile

register_kinematics_profile(
    "my_arm",
    KinematicsProfile(
        urdf_resource=None,           # set urdf_path on the adapter instead
        target_frame_name="tip_link",
        motor_names=("j1", "j2", "j3", "j4", "j5", "gripper"),
        gripper_name="gripper",
    ),
)
```

Then use `--teleop.adapter.robot_profile=my_arm`.

---

## Troubleshooting

**Arm keeps moving after releasing the SpaceMouse** — Increase `--teleop.deadzone` (e.g. `0.15`) or lower `--teleop.input_timeout_s` (e.g. `0.05`).

**Motion is too fast / too slow** — Tune `--teleop.adapter.translation_step_m` and `--teleop.adapter.rotation_step_rad`.

**Axis direction is wrong** — Flip the sign: `--teleop.x_sign=-1`, `--teleop.y_sign=-1`, etc.

**Device not found / permission denied** — Run `lerobot-teleoperator-spacemouse-test` first. Follow the [Linux HID Setup](#linux-hid-setup) section if it fails.

---

## Development

```bash
git clone https://github.com/Jas000n/lerobot-teleoperator-spacemouse.git
cd lerobot-teleoperator-spacemouse
pip install -e ".[dev]"
pytest
```


---

## License

Apache 2.0.
