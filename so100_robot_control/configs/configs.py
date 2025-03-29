from lerobot.common.robot_devices.robots.configs import *

from dataclasses import field
import os

@RobotConfig.register_subclass("randy")
@dataclass
class RandyConfig(ManipulatorRobotConfig):
    # get the current working directory
    
    calibration_dir: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), "randy")
    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    follower_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "main": FeetechMotorsBusConfig(
                port="/dev/tty.usbmodem58FA0919081",
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

    leader_arms : dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
        }
    )

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
        }
    )

    mock: bool = False


@RobotConfig.register_subclass("chatot")
@dataclass
class ChatotConfig(ManipulatorRobotConfig):
    # get the current working directory
    
    calibration_dir: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), "chatot")
    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    follower_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "main": FeetechMotorsBusConfig(
                port=None, # HIER UMCHANGEN HERR HABERHAUSEN
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

    leader_arms : dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
        }
    )

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
        }
    )

    mock: bool = False
