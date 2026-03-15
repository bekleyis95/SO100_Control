"""
Robot configuration dataclasses for SO-100 and compatible arms.

For port-only overrides, use RobotConfigLoader with a YAML file —
see so100_robot_control/configs/robot_loader.py.
"""
import os
from dataclasses import dataclass, field

_HERE = os.path.dirname(os.path.abspath(__file__))

_DEFAULT_URDF = os.path.join(
    _HERE,
    "randy_urdf",
    "SO_5DOF_ARM100_8j_URDF.SLDASM",
    "urdf",
    "SO_5DOF_ARM100_8j_URDF.SLDASM.urdf",
)

_DEFAULT_CALIBRATION = os.path.join(_HERE, "randy")


@dataclass
class SO100Config:
    """
    Configuration for the SO-ARM100 5-DOF arm with Feetech STS3215 motors.

    This is our project's config object.  When connecting to real hardware,
    robot_interface converts it to lerobot's SOFollowerRobotConfig so the
    correct motor driver and calibration flow are used.

    To change the serial port without editing this file, use a YAML robot
    config — see ``robots/so100.example.yaml`` and RobotConfigLoader.
    """
    urdf_path: str = _DEFAULT_URDF
    calibration_dir: str = _DEFAULT_CALIBRATION

    # Robot ID — used as the calibration filename: {calibration_dir}/{robot_id}.json
    robot_id: str = "so100"

    # Serial port for the follower arm
    port: str = "/dev/tty.usbmodem58FA0919081"

    # Set to False to skip the interactive calibration flow on connect.
    # Use this when the robot has been previously calibrated and the
    # calibration file already exists at {calibration_dir}/{robot_id}.json
    calibrate: bool = False

    # Safety: max joint movement per step (None = unlimited)
    max_relative_target: float | None = None

    # Use degrees for joint angles (True matches lerobot default)
    use_degrees: bool = True

    # Camera configs (passed through to lerobot as-is)
    cameras: dict = field(default_factory=dict)


# Backward-compatible alias (old name used in some notebooks / scripts)
RandyConfig = SO100Config
