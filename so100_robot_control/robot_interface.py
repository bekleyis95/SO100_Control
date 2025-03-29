import logging
from lerobot.common.robot_devices.robots.utils import *
import torch
from so100_robot_control.configs.configs import RandyConfig


class SO100RobotInterface:
    """
    Interface for communicating with the robot's proprietary SDK.
    """
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        # Initialize SDK connection here
        self.logger.info("SDK initialized.")
        self.robot = make_robot_from_config(RandyConfig())
        self.robot.connect()
        print(f"{self.robot.capture_observation()}")

    def send_joint_positions(self, joint_positions: torch.Tensor):
        """
        Send joint positions to the robot.
        :param joint_positions: List of joint angles in radians.a
        """
        assert isinstance(joint_positions, torch.Tensor), "Joint positions must be a torch.Tensor."
        self.robot.send_action(joint_positions)

    def stop_robot(self):
        """
        Stop the robot's motion.
        """
        self.logger.info("Stopping the robot.")
        self.robot.disconnect()
        self.logger.info("Robot disconnected.")
       
