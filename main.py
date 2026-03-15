"""
Entry point for the SO-100 robot controller.

Usage examples
--------------
# Joystick, joint control (default)
python main.py

# Keyboard, cartesian control, simulation only (no hardware needed)
python main.py --device keyboard --control-mode cartesian --simulate

# Custom robot config (port, motors, URDF from a YAML file)
python main.py --robot robots/so100.yaml
"""
from so100_robot_control.controllers.teleop_session import main

if __name__ == "__main__":
    main()
