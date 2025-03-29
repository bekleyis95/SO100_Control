import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import argparse
from robot_kinematics import RobotKinematics

class RobotSimulation:
    """Controller for SO-ARM100 robot using PyBullet simulation."""
    
    def __init__(self, urdf_path, gui=True, fixed_base=True):
        """Initialize the robot simulation."""
        # Connect to PyBullet
        self.client_id = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Disable gravity and dynamics
        p.setGravity(0, 0, 0)
        p.setRealTimeSimulation(0)
        
        # Load the robot
        self.robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=fixed_base)
        
        # Configure camera view
        p.resetDebugVisualizerCamera(
            cameraDistance=0.3,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.1]
        )
        
        # Initialize joint information
        self.joint_indices = []
        self.joint_names = []
        self.discover_joints()
        
        # End effector link index
        self.end_effector_link = 5
        
        # Initialize kinematics calculator
        self.kinematics = RobotKinematics()
        
        # Default initial joint positions for downward-facing gripper
        self.initial_joint_positions = [0.0, -np.pi/2, 0.0, -np.pi, np.pi/2]
        self.current_joint_positions = np.array(self.initial_joint_positions)
        self.reset_to_initial_pose()
    
    def discover_joints(self):
        """Discover and store information about robot joints."""
        num_joints = p.getNumJoints(self.robot_id)
        
        print("Discovering robot joints:")
        print("------------------------")
        
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            
            if joint_type != p.JOINT_FIXED:
                print(f"Joint {joint_index}: {joint_name}")
                self.joint_indices.append(joint_index)
                self.joint_names.append(joint_name)
        
        print("------------------------")
        print(f"Found {len(self.joint_indices)} movable joints.")
    
    def reset_to_initial_pose(self, joint_positions=None):
        """Reset robot to initial pose with gripper facing downward."""
        if joint_positions is None:
            joint_positions = self.initial_joint_positions
        
        for i, joint_idx in enumerate(self.joint_indices[:5]):
            p.resetJointState(self.robot_id, joint_idx, joint_positions[i])
        
        self.current_joint_positions = np.array(joint_positions)
        p.stepSimulation()
        
        # Verify orientation
        _, orientation = self.get_full_end_effector_pose()
        print(f"Robot reset to initial pose. Gripper orientation (roll, pitch, yaw): {orientation}")
        print(f"Joint positions: {self.current_joint_positions}")
    
    def set_joint_positions(self, joint_positions):
        """Set joint positions directly without dynamics."""
        for i, joint_idx in enumerate(self.joint_indices[:5]):
            p.resetJointState(self.robot_id, joint_idx, joint_positions[i])
        
        self.current_joint_positions = np.array(joint_positions)
        p.stepSimulation()
    
    def get_end_effector_pose(self, full_pose=False):
        """Get current end-effector pose."""
        state = p.getLinkState(self.robot_id, self.end_effector_link, computeForwardKinematics=True)
        position = state[0]
        orientation = p.getEulerFromQuaternion(state[1])
        
        if full_pose:
            # Return full 6DOF pose [x, y, z, roll, pitch, yaw]
            return np.array([position[0], position[1], position[2], 
                            orientation[0], orientation[1], orientation[2]])
        else:
            # Return simplified 4DOF pose [x, y, z, yaw]
            return np.array([position[0], position[1], position[2], orientation[2]])
    
    def get_full_end_effector_pose(self):
        """Get complete end-effector pose including full orientation."""
        state = p.getLinkState(self.robot_id, self.end_effector_link, computeForwardKinematics=True)
        position = state[0]
        orientation = p.getEulerFromQuaternion(state[1])
        
        return position, orientation
    
    def get_all_joint_states(self, joint_positions, controllable_joints):
        """Get all joint states, filling in controlled joint positions."""
        all_joint_states = []
        for i in range(p.getNumJoints(self.robot_id)):
            if i in controllable_joints:
                idx = controllable_joints.index(i)
                all_joint_states.append(joint_positions[idx])
            else:
                state = p.getJointState(self.robot_id, i)
                all_joint_states.append(state[0])
        return all_joint_states
    
    def get_jacobian_wrapper(self, end_effector_link, local_position, positions, velocities, accelerations):
        """Wrapper for PyBullet's calculateJacobian function."""
        return p.calculateJacobian(
            bodyUniqueId=self.robot_id,
            linkIndex=end_effector_link,
            localPosition=local_position,
            objPositions=positions,
            objVelocities=velocities,
            objAccelerations=accelerations
        )
    
    def compute_jacobian(self, joint_positions, full_jacobian=False):
        """Compute the Jacobian for position and orientation."""
        return self.kinematics.compute_jacobian(
            get_link_state_fn=self.get_jacobian_wrapper,
            get_all_joint_states_fn=self.get_all_joint_states,
            joint_positions=joint_positions,
            controllable_joints=self.joint_indices[:5],
            end_effector_link=self.end_effector_link,
            full_jacobian=full_jacobian
        )
    
    def ensure_gripper_down(self, joint_positions):
        """Adjust joint positions to ensure the gripper is facing downward."""
        # Save the original positions
        original_positions = self.current_joint_positions.copy()
        
        # Temporarily set the joint positions to check orientation
        for i, joint_idx in enumerate(self.joint_indices[:5]):
            p.resetJointState(self.robot_id, joint_idx, joint_positions[i])
        
        # Get current orientation
        _, orientation = self.get_full_end_effector_pose()
        
        # Determine if adjustment is needed
        adjusted_positions = joint_positions.copy()
        if not self.kinematics.is_orientation_downward(orientation):
            # Adjust wrist pitch to compensate
            adjusted_positions[3] = -np.pi  # Set wrist pitch for downward orientation
        
        # Restore original positions
        for i, joint_idx in enumerate(self.joint_indices[:5]):
            p.resetJointState(self.robot_id, joint_idx, original_positions[i])
        
        return adjusted_positions
    
    def calculate_joint_positions_for_target(self, target_pose, enforce_down=False):
        """Calculate joint positions for a target Cartesian pose."""
        # Create wrapper functions for the kinematics calculator
        def get_pose_wrapper(full_pose):
            return self.get_end_effector_pose(full_pose=full_pose)
        
        def set_joints_wrapper(positions):
            for i, joint_idx in enumerate(self.joint_indices[:5]):
                p.resetJointState(self.robot_id, joint_idx, positions[i])
        
        def get_jacobian_wrapper(positions, full_jacobian):
            return self.compute_jacobian(positions, full_jacobian)
        
        # Call the kinematics calculator
        joint_positions, success, message = self.kinematics.calculate_ik_solution(
            get_pose_fn=get_pose_wrapper,
            set_joints_fn=set_joints_wrapper,
            get_jacobian_fn=get_jacobian_wrapper,
            current_joint_positions=self.current_joint_positions,
            target_pose=target_pose,
            enforce_down=enforce_down
        )
        
        # Apply downward orientation constraint if requested
        if enforce_down and not len(target_pose) == 6:
            joint_positions = self.ensure_gripper_down(joint_positions)
            
        return joint_positions, success, message
    
    def move_to_target(self, target_pose, steps=30, visualize=True, enforce_down=False):
        """Move to target pose by calculating joint positions and applying them directly."""
        full_dof = len(target_pose) == 6
        
        # Print target pose information
        if full_dof:
            print(f"Moving to target: x={target_pose[0]:.3f}, y={target_pose[1]:.3f}, z={target_pose[2]:.3f}, " +
                  f"roll={target_pose[3]:.3f}, pitch={target_pose[4]:.3f}, yaw={target_pose[5]:.3f}")
        else:
            print(f"Moving to target: x={target_pose[0]:.3f}, y={target_pose[1]:.3f}, z={target_pose[2]:.3f}, yaw={target_pose[3]:.3f}")
        
        # Calculate target joint positions
        target_joints, success, message = self.calculate_joint_positions_for_target(target_pose, enforce_down)
        
        if not success:
            print(f"Warning: {message}")
            print("Attempting to move to best approximation of target pose")
        
        # Create trajectory
        current_joints = self.current_joint_positions.copy()
        trajectory = self.kinematics.plan_trajectory(
            current_joints, target_joints, steps, trajectory_type="smooth")
        
        # Follow trajectory
        for step, joint_positions in enumerate(trajectory):
            # Apply downward orientation constraint at intervals if requested
            if enforce_down and step % 5 == 0 and not full_dof:
                joint_positions = self.ensure_gripper_down(joint_positions)
            
            # Set joint positions
            self.set_joint_positions(joint_positions)
            
            # Update camera to follow end effector
            current_pose = self.get_end_effector_pose()
            p.resetDebugVisualizerCamera(
                cameraDistance=0.3,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[current_pose[0], current_pose[1], current_pose[2]]
            )
            
            # Visualization
            if visualize:
                self.visualize_pose()
                time.sleep(0.03)
        
        return success
    
    def visualize_pose(self):
        """Visualize the current pose with debug text."""
        current_full_pose = self.get_end_effector_pose(full_pose=True)
        p.addUserDebugText(
            f"EE: x={current_full_pose[0]:.3f}, y={current_full_pose[1]:.3f}, z={current_full_pose[2]:.3f}",
            [0, 0, 0.3],
            lifeTime=0.1
        )
        p.addUserDebugText(
            f"RPY: {current_full_pose[3]:.2f}, {current_full_pose[4]:.2f}, {current_full_pose[5]:.2f}",
            [0, 0, 0.35],
            lifeTime=0.1
        )
        
        # Show orientation status
        if self.kinematics.is_orientation_downward(current_full_pose[3:]):
            color = [0, 1, 0]  # Green if facing down
            status = "DOWN"
        else:
            color = [1, 0, 0]  # Red if not facing down
            status = "NOT DOWN"
        
        p.addUserDebugText(
            status,
            [0, 0, 0.4],
            lifeTime=0.1,
            textColorRGB=color
        )

    # ... Drawing functions (draw_box_trajectory, draw_axes) can be included here ...

def main():
    """Main function to parse arguments and run the robot simulation."""
    parser = argparse.ArgumentParser(description='SO-ARM100 Robot Controller')
    parser.add_argument('--mode', choices=['cartesian', 'direct', 'box', 'axes', '6dof'], 
                        required=True, help='Control mode')
    parser.add_argument('--x', type=float, required=True,
                        help='Target X position')
    parser.add_argument('--y', type=float, required=True,
                        help='Target Y position')
    parser.add_argument('--z', type=float, required=True,
                        help='Target Z position')
    parser.add_argument('--roll', type=float, default=None,
                        help='Target Roll angle (radians)')
    parser.add_argument('--pitch', type=float, default=None,
                        help='Target Pitch angle (radians)')
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='Target Yaw angle (radians)')
    parser.add_argument('--enforce-down', action='store_true',
                        help='Enforce downward orientation')
    
    # Add more arguments as needed
    
    args = parser.parse_args()
    
    # Path to the URDF file
    urdf_path = "/Users/denizbekleyisseven/workspace/SO100_Control/SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf"
    
    # Create robot simulation
    robot = RobotSimulation(urdf_path)
    
    # Execute selected mode
    if args.mode == 'cartesian':
        # Position + yaw control
        desired_pose = np.array([args.x, args.y, args.z, args.yaw])
        success = robot.move_to_target(desired_pose, steps=40, enforce_down=args.enforce_down)
        if not success:
            print("Warning: Target pose may not be achieved exactly")
            
    elif args.mode == '6dof':
        # Full 6DOF position and orientation control
        if args.roll is None or args.pitch is None:
            print("Error: 6DOF mode requires roll and pitch arguments")
            return
            
        desired_pose = np.array([args.x, args.y, args.z, args.roll, args.pitch, args.yaw])
        success = robot.move_to_target(desired_pose, steps=40)
        if not success:
            print("Error: Target pose cannot be achieved with this robot configuration")
    
    # Implement other modes as needed
    
    # Keep the simulation running
    print("Demonstration complete. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()
