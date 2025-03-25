from urdfpy import URDF
import numpy as np
from ikpy.chain import Chain

class Kinematics:
    """
    Handles inverse kinematics (IK), forward kinematics (FK), and 4D pose control.
    """
    def __init__(self, urdf_path: str):
        self.robot = URDF.load(urdf_path)
        self.chain = Chain.from_urdf_file(urdf_path)

    def compute_ik(self, target_pose: np.ndarray) -> np.ndarray:
        """
        Compute inverse kinematics for a given target pose.
        :param target_pose: 4x4 transformation matrix.
        :return: Joint angles in radians.
        """
        return self.chain.inverse_kinematics(target_pose)

    def compute_fk(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Compute forward kinematics for given joint angles.
        :param joint_angles: List of joint angles in radians.
        :return: 4x4 transformation matrix.
        """
        return self.chain.forward_kinematics(joint_angles)
