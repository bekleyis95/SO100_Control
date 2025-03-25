import logging

class SDKInterface:
    """
    Interface for communicating with the robot's proprietary SDK.
    """
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        # Initialize SDK connection here
        self.logger.info("SDK initialized.")

    def send_joint_positions(self, joint_positions):
        """
        Send joint positions to the robot.
        :param joint_positions: List of joint angles in radians.
        """
        self.logger.debug(f"Sending joint positions: {joint_positions}")
        # SDK-specific code to send joint positions

    def stop_robot(self):
        """
        Stop the robot's motion.
        """
        self.logger.info("Stopping the robot.")
        # SDK-specific code to stop the robot
