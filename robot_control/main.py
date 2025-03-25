import logging
import numpy as np
from robot_control.sdk_interface import SDKInterface
from robot_control.kinematics import Kinematics
from robot_control.trajectory import Trajectory

def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    # Initialize components
    sdk = SDKInterface()
    kinematics = Kinematics("urdf/my_robot.urdf")

    # Define target pose (4D: xyz + yaw)
    target_pose = np.eye(4)
    target_pose[:3, 3] = [0.5, 0.0, 0.5]  # xyz
    # Compute IK
    joint_angles = kinematics.compute_ik(target_pose)

    # Interpolate trajectory
    current_angles = np.zeros(len(joint_angles))  # Assume starting at zero
    trajectory = Trajectory.interpolate(current_angles, joint_angles, steps=100)

    # Execute trajectory
    for step in trajectory:
        sdk.send_joint_positions(step)

    sdk.stop_robot()
    logger.info("Motion complete.")

if __name__ == "__main__":
    main()
