# SO-100 Control

Real-time teleoperation for the [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) 5-DOF robot arm.

Supports **joint-space** and **Cartesian** control via joystick or keyboard, with an optional **PyBullet simulation** — no hardware required to get started.

---

## Install

```bash
git clone https://github.com/your-org/SO100_Control.git
cd SO100_Control

conda create -n so100 python=3.10 -y
conda activate so100

pip install lerobot
pip install "numpy<2"   # pybullet requires numpy 1.x
pip install -e .
```

---

## Running — simulation (no hardware)

```bash
# Joint control with keyboard
python main.py --simulate --device keyboard

# Cartesian control with keyboard
python main.py --simulate --device keyboard --control-mode cartesian

# Joystick (falls back to keyboard if none detected)
python main.py --simulate
```

---

## Running — real hardware

### 1. Find your serial port

```bash
# macOS
ls /dev/tty.usb*

# Linux
ls /dev/ttyUSB* /dev/ttyACM*
```

### 2. Create your robot config

```bash
cp robots/so100.example.yaml robots/so100.yaml
# open robots/so100.yaml and set:  port: /dev/tty.usbmodemXXXXXX
```

### 3. Run

```bash
python main.py --robot robots/so100.yaml --device keyboard
```

On first run the arm homes to a safe pose and waits until it arrives before handing control to you.

### Calibration

The bundled calibration (`so100_robot_control/configs/randy/so100.json`) was recorded on the reference hardware. If your arm behaves strangely (joints feel reversed or angles are off) run the interactive calibration once:

```bash
# in robots/so100.yaml, set:  calibrate: true
python main.py --robot robots/so100.yaml --device keyboard
# After calibration completes, set calibrate back to false
```

---

## CLI reference

```
python main.py [options]

  --robot PATH              YAML robot config (see robots/so100.example.yaml)
  --device {joystick,keyboard}
                            Input device (default: joystick; auto-falls back to keyboard)
  --control-mode {joint,cartesian}
                            Control mode (default: joint)
  --simulate                Run in PyBullet simulation only (no hardware)
```

Also available as an installed script:

```bash
so100-control --simulate --device keyboard
```

---

## Controls

### Keyboard

| Key | Action |
|---|---|
| ↑ / ↓ | Move active joint / Cartesian axis |
| ← / → | Base rotation (joint 0) |
| `1`–`5` | Select active joint or Cartesian axis |
| `G` | Toggle gripper open / closed |
| `M` | Toggle upper / lower control mode |
| `S` | Print current joint state |
| `ESC` | Emergency stop |
| `Q` | Quit |

### Joystick

| Input | Action |
|---|---|
| Y-axis | Move active joint / Cartesian axis |
| X-axis | Base rotation |
| Buttons 1–3 | Select active element |
| Button 11 | Toggle upper/lower mode |
| Button 5 | Print current state |
| Button 10 | Emergency stop |

---

## Architecture

```
main.py
  └── RobotController          controllers/combined_controller.py
        ├── RobotInterface     robot_interface.py        ← lerobot 0.4 hardware driver
        │     └── RobotKinematics  simulation/           ← symbolic FK / Jacobian IK
        ├── KeyboardController / JoystickController      ← teleop input (updates target)
        │     └── BaseController  base_controller.py
        └── RobotSimulation    simulation/               ← PyBullet (--simulate only)
```

**Control loop design** — input and motor commanding are fully decoupled:

- The **input thread** (keyboard/joystick event loop) only updates `target_joint_angles`.
- The **control thread** sends `target_joint_angles` to the motors **every tick at 30 Hz**, regardless of whether input changed. This gives smooth hold behaviour without relying on the motor's internal PID alone.

---

## Project layout

```
so100_robot_control/
├── base_controller.py          Abstract controller base
├── robot_interface.py          Hardware abstraction (lerobot 0.4 SOFollower)
├── configs/
│   ├── configs.py              SO100Config dataclass
│   ├── robot_loader.py         YAML → config loader
│   ├── randy/                  Bundled calibration files
│   └── randy_urdf/             Bundled URDF + STL meshes
├── controllers/
│   └── combined_controller.py  RobotController orchestrator + CLI
├── simulation/
│   ├── robot_kinematics.py     Symbolic FK / Jacobian IK
│   ├── robot_simulation.py     PyBullet visualisation
│   └── joint_map.txt           Real ↔ sim angle reference
└── teleop_devices/
    ├── joystick_listener.py    Joystick input (pyglet)
    └── keyboard_listener.py    Keyboard input (pygame)

robots/
└── so100.example.yaml          Robot config template (copy → so100.yaml)
```

---

## Angle mapping (real ↔ sim)

Hardware reports degrees (`real space`). The URDF / IK works in radians with a different sign convention (`sim space`).

| Joint | real → sim |
|---|---|
| shoulder_pan  | `sim = -real_deg` |
| shoulder_lift | `sim = -(real_deg - 90)` |
| elbow_flex    | `sim = real_deg - 90` |
| wrist_flex    | `sim = real_deg - 90` |
| wrist_roll    | `sim = -(real_deg - 90)` |

See `so100_robot_control/simulation/joint_map.txt` for the full reference table.

---

## License

MIT
