import numpy as np
from robot_control.kinematics import Kinematics

def test_ik():
    kinematics = Kinematics("urdf/my_robot.urdf")
    target_pose = np.eye(4)
    target_pose[:3, 3] = [0.5, 0.0, 0.5]
    joint_angles = kinematics.compute_ik(target_pose)
    assert len(joint_angles) > 0
