# SO100 Robot Control

A Python package for controlling the SO-ARM100 robot system. This library provides interfaces for programming, simulating, and operating the SO-ARM100 robotic arm.

## Features

- Robot simulation using PyBullet
- Direct control interface for the SO-ARM100 robot
- Various control modes including position, velocity, and torque control
- Support for reinforcement learning applications with Gym integration

## Installation

### Using Conda (Recommended)

1. Clone the repository:
   ```
   git clone https://github.com/yourusername/SO100_Control.git
   cd SO100_Control
   ```

2. Create a new conda environment:
   ```
   conda create -n so100 python=3.10
   conda activate so100
   ```

3. Install the package and its dependencies:
   ```
   conda install -c conda-forge numpy pybullet
   conda install -c pytorch pytorch
   pip install -e .
   ```

### Using Pip

You can also install the package using pip:

```
pip install -e .
```

## Usage

```python
import so100_robot_control as so100

# Initialize the robot
robot = so100.SO100Robot()

# Move the robot to a target position
robot.move_to_position([0.2, 0.3, 0.4])

# Get the current state of the robot
state = robot.get_state()

# Control the robot with torque commands
robot.apply_torque([1.0, 2.0, 1.5, 0.5, 0.8, 1.0])
```

## Running the Controller

To run the robot controller, use the main.py script:

```bash
python main.py --mode joystick  # For joystick control
# or
python main.py --mode keyboard  # For keyboard control
```

## Configuration

### Robot Configuration

The SO100 robot uses the RandyConfig class defined in `so100_robot_control/configs/configs.py`. Make sure to update the port configuration to match your hardware setup:

```python
@RobotConfig.register_subclass("randy")
@dataclass
class RandyConfig(ManipulatorRobotConfig):
    calibration_dir: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), "randy")
    max_relative_target: int | None = None

    follower_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "main": FeetechMotorsBusConfig(
                port="/dev/YOUR_PORT_HERE",  # Update this to your port
                motors={
                    # name: (index, model)
                    "shoulder_pan": [1, "sts3215"],
                    "shoulder_lift": [2, "sts3215"],
                    "elbow_flex": [3, "sts3215"],
                    "wrist_flex": [4, "sts3215"],
                    "wrist_roll": [5, "sts3215"],
                    "gripper": [6, "sts3215"],
                },
            ),
        }
    )
    # ... other config parameters ...
```

### Manipulator Modifications

If you're using a custom robot configuration, you need to modify the `lerobot/common/robot_devices/robots/manipulator.py` file to add your robot type in three locations:

1. In the `connect()` method where robot types are checked for torque mode:
   ```python
   if self.robot_type in ["koch", "koch_bimanual", "aloha"]:
       from lerobot.common.robot_devices.motors.dynamixel import TorqueMode
   elif self.robot_type in ["so100", "randy", "chatot", "moss", "lekiwi", "YOUR_ROBOT_TYPE"]:
       from lerobot.common.robot_devices.motors.feetech import TorqueMode
   ```

2. In the `activate_calibration()` method:
   ```python
   if self.robot_type in ["koch", "koch_bimanual", "aloha"]:
       from lerobot.common.robot_devices.robots.dynamixel_calibration import run_arm_calibration
       calibration = run_arm_calibration(arm, self.robot_type, name, arm_type)
   elif self.robot_type in ["so100", "randy", "chatot", "moss", "lekiwi", "YOUR_ROBOT_TYPE"]:
       from lerobot.common.robot_devices.robots.feetech_calibration import run_arm_manual_calibration
       calibration = run_arm_manual_calibration(arm, self.robot_type, name, arm_type)
   ```

3. In the preset selection:
   ```python
   if self.robot_type in ["koch", "koch_bimanual"]:
       self.set_koch_robot_preset()
   elif self.robot_type == "aloha":
       self.set_aloha_robot_preset()
   elif self.robot_type in ["so100", "randy", "chatot", "moss", "lekiwi", "YOUR_ROBOT_TYPE"]:
       self.set_so100_robot_preset()
   ```

### Calibration Files

Make sure to create a calibration directory for your robot:

```
mkdir -p /Users/denizbekleyisseven/workspace/SO100_Control/so100_robot_control/configs/YOUR_CONFIG_NAME
```

During the first run, calibration files will be generated in this directory. Subsequent runs will use these calibration files.

## Dependencies

- Python ≥ 3.10
- NumPy ≥ 1.19.0
- PyBullet ≥ 3.0.0
- PyTorch ≥ 1.10.0
- Pygame ≥ 2.6.0
- Pyglet ≥ 2.1.0
- Draccus ≥ 0.10.0
- Feetech-servo-sdk ≥ 1.0.0
- Gym ≥ 0.21.0 (for reinforcement learning applications)

## Development

Install development dependencies:

```
pip install -e ".[dev]"
```

Run tests:

```
pytest
```

Format code:

```
black .
isort .
```

## License

[MIT License](LICENSE)

## Contact

For questions or support, please contact Deniz Bekleyis Seven at dseven1995@gmail.com.
