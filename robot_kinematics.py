import numpy as np

class RobotKinematics:
    """Robot kinematics calculations for the SO-ARM100 robot.
    
    This class handles kinematics calculations independent of simulation or physical hardware.
    It can be used for both the PyBullet simulation and the real robot.
    """
    
    def __init__(self):
        """Initialize the robot kinematics calculator."""
        # Default orientation for gripper facing down
        self.default_orientation = {
            'roll': 0.0,
            'pitch': -np.pi,  # Pointing down
            'yaw': 0.0        # Can be controlled separately
        }
        
        # IK solver parameters
        self.max_ik_iterations = 30
        self.ik_residual_threshold = 0.001
        self.ik_gain = 0.5
    
    def compute_jacobian(self, get_link_state_fn, get_all_joint_states_fn, joint_positions, controllable_joints, end_effector_link, full_jacobian=False):
        """Compute the Jacobian using the provided state functions.
        
        Args:
            get_link_state_fn: Function to get link state
            get_all_joint_states_fn: Function to get all joint states
            joint_positions: Current joint positions
            controllable_joints: List of controllable joint indices
            end_effector_link: End effector link index
            full_jacobian: Whether to calculate full 6DOF Jacobian
            
        Returns:
            np.ndarray: Jacobian matrix
        """
        # Get the current joint states for all joints
        all_joint_states = get_all_joint_states_fn(joint_positions, controllable_joints)
        
        # Create vectors for velocity and acceleration (zeros)
        zero_vec = [0.0] * len(all_joint_states)
        
        # Calculate Jacobian (this requires the specific implementation to call PyBullet or other backend)
        linear_jac, angular_jac = get_link_state_fn(
            end_effector_link=end_effector_link,
            local_position=[0, 0, 0],
            positions=all_joint_states,
            velocities=zero_vec,
            accelerations=zero_vec
        )
        
        # Create the appropriate Jacobian matrix
        if full_jacobian:
            # Create full 6×N Jacobian for position and orientation
            jacobian = np.zeros((6, len(joint_positions)))
            jacobian[0:3, :] = np.array(linear_jac)[:, controllable_joints]   # x, y, z
            jacobian[3:6, :] = np.array(angular_jac)[:, controllable_joints]  # roll, pitch, yaw
        else:
            # Create 4×N Jacobian for position and yaw only
            jacobian = np.zeros((4, len(joint_positions)))
            jacobian[0:3, :] = np.array(linear_jac)[:, controllable_joints]   # x, y, z
            jacobian[3, :] = np.array(angular_jac)[2, controllable_joints]    # yaw only
        
        return jacobian
    
    def calculate_ik_solution(self, get_pose_fn, set_joints_fn, get_jacobian_fn, 
                              current_joint_positions, target_pose, enforce_down=False):
        """Calculate inverse kinematics solution to reach the target pose.
        
        Args:
            get_pose_fn: Function to get end effector pose
            set_joints_fn: Function to set joint positions temporarily
            get_jacobian_fn: Function to compute the Jacobian
            current_joint_positions: Starting joint positions
            target_pose: Target pose as [x,y,z,yaw] or [x,y,z,roll,pitch,yaw]
            enforce_down: Whether to enforce downward orientation
            
        Returns:
            tuple: (joint_positions, success_flag, error_message)
        """
        # Start with current joint positions
        joint_positions = np.copy(current_joint_positions)
        
        # Determine if we're using full 6DOF control based on target_pose length
        full_dof = len(target_pose) == 6
        
        # If only 4DOF provided but enforce_down is true, add default orientation
        if not full_dof and enforce_down:
            # Use default downward orientation for roll and pitch
            target_pose = np.array([
                target_pose[0], target_pose[1], target_pose[2],  # x, y, z
                self.default_orientation['roll'],                # roll
                self.default_orientation['pitch'],               # pitch
                target_pose[3]                                   # yaw
            ])
            full_dof = True
        
        # IK iteration parameters
        min_error = float('inf')
        best_joints = joint_positions.copy()
        
        for iteration in range(self.max_ik_iterations):
            # Set the current joint positions temporarily
            set_joints_fn(joint_positions)
            
            # Get current end effector pose
            current_pose = get_pose_fn(full_pose=full_dof)
            
            # Calculate pose error
            pose_error = target_pose - current_pose
            error_norm = np.linalg.norm(pose_error)
            
            # Track the best solution found
            if error_norm < min_error:
                min_error = error_norm
                best_joints = joint_positions.copy()
            
            # If error is small enough, consider solution found
            if error_norm < self.ik_residual_threshold:
                return joint_positions, True, "IK solution found"
            
            # Compute Jacobian appropriate for the target DOF
            jacobian = get_jacobian_fn(joint_positions, full_jacobian=full_dof)
            
            # Try to compute pseudo-inverse for Jacobian
            try:
                # Calculate joint deltas using pseudo-inverse
                jacobian_pinv = np.linalg.pinv(jacobian)
                joint_deltas = np.matmul(jacobian_pinv, pose_error) * self.ik_gain
                
                # Update joint positions
                joint_positions += joint_deltas
                
            except np.linalg.LinAlgError:
                return best_joints, False, "Singularity encountered in IK calculation"
        
        # If we get here, max iterations reached without finding solution
        if min_error < 0.01:
            # Close enough for practical purposes
            return best_joints, True, f"Approximate solution found (error: {min_error:.4f})"
        else:
            # No reasonable solution found
            return best_joints, False, f"IK failed to converge (error: {min_error:.4f})"
    
    def plan_trajectory(self, start_positions, target_positions, steps, trajectory_type="linear"):
        """Plan a trajectory between start and target joint positions.
        
        Args:
            start_positions: Starting joint positions
            target_positions: Target joint positions
            steps: Number of interpolation steps
            trajectory_type: Type of trajectory ("linear" or "smooth")
            
        Returns:
            list: List of joint positions along the trajectory
        """
        trajectory = []
        
        for step in range(steps + 1):
            t = step / steps
            
            if trajectory_type == "smooth":
                # Smooth acceleration/deceleration (ease-in-ease-out)
                alpha = 3 * t**2 - 2 * t**3
            else:
                # Linear trajectory
                alpha = t
            
            # Interpolate joint positions
            interp_positions = start_positions * (1 - alpha) + target_positions * alpha
            trajectory.append(interp_positions)
        
        return trajectory
    
    def is_orientation_downward(self, orientation, tolerance=0.3):
        """Check if the end effector orientation is pointing downward.
        
        Args:
            orientation: [roll, pitch, yaw] orientation
            tolerance: Tolerance in radians
            
        Returns:
            bool: True if pointing downward, False otherwise
        """
        _, pitch, _ = orientation
        return abs(abs(pitch) - np.pi) < tolerance
