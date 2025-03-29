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

## Dependencies

- Python ≥ 3.10
- NumPy ≥ 1.19.0
- PyBullet ≥ 3.0.0
- PyTorch ≥ 1.10.0
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
