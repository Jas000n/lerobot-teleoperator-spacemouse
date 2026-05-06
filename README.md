# LeRobot SpaceMouse Teleoperator

`lerobot-teleoperator-spacemouse` is a LeRobot third-party teleoperator package for 3Dconnexion SpaceMouse devices.

Install it with:

```bash
pip install "lerobot-teleoperator-spacemouse[soarm]"
```

The main user entry point is the stock LeRobot command:

```bash
lerobot-teleoperate \
  --teleop.type=spacemouse \
  ...
```

This package also exposes `lerobot-teleoperator-spacemouse-test` for device diagnostics.

The PyPI install name is hyphenated, but the package metadata and Python module use
`lerobot_teleoperator_spacemouse` so stock LeRobot releases can auto-discover it through their
existing `lerobot_teleoperator_` plugin prefix.

## What it does

- Reads 6-DoF SpaceMouse input through `pyspacemouse`.
- Emits LeRobot teleoperator actions: `target_x/y/z`, `target_wx/wy/wz`, `gripper_vel`, `enabled`.
- Converts SpaceMouse deltas to end-effector targets.
- For SO-101 follower, converts end-effector targets to joint commands through LeRobot's `RobotKinematics`
  and a bundled kinematics-only URDF.
- Leaves a direct end-effector mode and a custom IK profile path for future robot extensions.

## Linux HID setup

If the SpaceMouse cannot be opened, install the HID shared library first.

For conda environments:

```bash
conda install -c conda-forge libhidapi
```

For Debian/Ubuntu system Python environments:

```bash
sudo apt-get install libhidapi-hidraw0 libhidapi-dev
```

Then add udev permissions:

```bash
sudo tee /etc/udev/rules.d/99-spacemouse-hidraw.rules >/dev/null <<'EOF'
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", MODE="0660", GROUP="plugdev", TAG+="uaccess"
EOF
sudo usermod -aG plugdev "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug the SpaceMouse. Log out and back in after changing groups.

## Test the device

```bash
lerobot-teleoperator-spacemouse-test
```

## SO-ARM teleoperation

Example for SO-101:

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=black \
  --teleop.type=spacemouse \
  --teleop.adapter.mode=ik \
  --fps=60
```

Recommended first run with a conservative step size:

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=black \
  --teleop.type=spacemouse \
  --teleop.adapter.mode=ik \
  --teleop.deadzone=0.1 \
  --teleop.input_timeout_s=0.08 \
  --teleop.read_drain_count=32 \
  --teleop.adapter.translation_step_m=0.001 \
  --teleop.adapter.rotation_step_rad=0.005 \
  --display_data=true
```

By default, `--teleop.adapter.robot_profile=so101_follower` uses the bundled
`so101_new_calib.urdf`. Override `--teleop.adapter.urdf_path=...` only when you have a calibrated
URDF you want to use instead.

By default, SpaceMouse physical X/Y input is swapped so forward/back drives robot `target_x` and
left/right drives robot `target_y`. Tune signs with `--teleop.x_sign=-1`, `--teleop.y_sign=-1`,
etc. if the motion direction is reversed.
If the arm continues moving after you release the SpaceMouse, increase `--teleop.deadzone` or lower
`--teleop.input_timeout_s`. If it moves too far per push, lower
`--teleop.adapter.translation_step_m` and `--teleop.adapter.rotation_step_rad`.

## Extending Other Robots

Other arms are intentionally not wired in yet. For a one-off custom arm, use:

```bash
lerobot-teleoperate \
  --teleop.type=spacemouse \
  --teleop.adapter.mode=ik \
  --teleop.adapter.robot_profile=custom \
  --teleop.adapter.urdf_path=/path/to/robot.urdf \
  --teleop.adapter.target_frame_name=your_tip_frame \
  ...
```

Custom IK profiles also need `--teleop.adapter.motor_names=...` and
`--teleop.adapter.gripper_name=...` when they differ from the SO-101 defaults.

For a reusable extension, register a `KinematicsProfile` from Python with
`lerobot_teleoperator_spacemouse.profiles.register_kinematics_profile()`.

## Blackwell / RTX PRO 6000 note

For Blackwell GPUs, prefer a PyTorch wheel with CUDA 12.8 or newer. A LeRobot-compatible stable option is:

```bash
pip install torch==2.10.0 torchvision==0.25.0 torchaudio==2.10.0 \
  --index-url https://download.pytorch.org/whl/cu128
```

Check the environment:

```bash
python - <<'PY'
import torch
print(torch.__version__, torch.version.cuda)
print(torch.cuda.is_available())
print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "no cuda")
PY
```

## Package development

```bash
conda activate lerobot-spacemouse
pip install -e ".[dev,soarm]"
pytest
python -m build
twine upload dist/*
```
