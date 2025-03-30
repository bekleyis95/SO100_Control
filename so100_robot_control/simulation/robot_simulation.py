import numpy as np
import pybullet as p
import pybullet_data
import time
import logging
import math

logging.basicConfig(level=logging.INFO)

class RobotSimulation:
    def __init__(self, urdf_path):
        self.urdf_path = urdf_path
        self.joint_angles = {}  # map joint indices to current angles
        self.external_real_angles = None  # new attribute for real robot angles
        self.simulate_real = False       # flag to trigger simulation from real angles
    
    def init_simulation(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        # Load URDF with fixed base
        self.robot = p.loadURDF(self.urdf_path, useFixedBase=True)
        p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        num_joints = p.getNumJoints(self.robot)
        self.num_joints = num_joints          # NEW: store number of joints
        self.time_step = 0.01                 # NEW: simulation time step
        for i in range(num_joints):
            state = p.getJointState(self.robot, i)
            self.joint_angles[i] = state[0]
        logging.info("URDF loaded with %d joints.", num_joints)
    
    def step_simulation(self):
        p.stepSimulation()
        time.sleep(0.01)  # fast internal timestep
        num_joints = p.getNumJoints(self.robot)
        for i in range(num_joints):
            state = p.getJointState(self.robot, i)
            new_angle = state[0]
            old_angle = self.joint_angles[i]
            if not math.isclose(old_angle, new_angle, rel_tol=1e-5):
                self.joint_angles[i] = new_angle
                info = p.getJointInfo(self.robot, i)
                joint_name = info[1].decode() if isinstance(info[1], bytes) else info[1]
                # logging.info("Joint '%s' (index %d) angle changed from %.5f to %.5f", joint_name, i, old_angle, new_angle)
    
    def set_joint_states(self, target_angles_deg):
        # Set each joint state to the target angle (converted from degrees to radians)
        num_joints = p.getNumJoints(self.robot)
        for i in range(num_joints):
            target_rad = math.radians(target_angles_deg[i])
            p.resetJointState(self.robot, i, target_rad)
            self.joint_angles[i] = target_rad
            info = p.getJointInfo(self.robot, i)
            joint_name = info[1].decode() if isinstance(info[1], bytes) else info[1]
    
    def sim_to_real(sim_angles):
        """
        Convert simulation angles to real angles using the relation:
        
        real[0] = -sim[0]
        real[1] = -sim[1] + 90
        real[2] = sim[2] + 90
        real[3] = sim[3] + 90
        real[4] = -sim[4] + 90
        real[5] = sim[5]
        
        Parameters:
            sim_angles (array-like): A list or numpy array of 6 simulation angles.
        
        Returns:
            np.ndarray: A numpy array of 6 real angles.
        """
        sim_angles = np.array(sim_angles)
        real = np.empty_like(sim_angles)
        
        real[0] = -sim_angles[0]
        real[1] = -sim_angles[1] + 90
        real[2] = sim_angles[2] + 90
        real[3] = sim_angles[3] + 90
        real[4] = -sim_angles[4] + 90
        real[5] = sim_angles[5]
        
        return real

    def real_to_sim(real_angles):
        """
        Convert real angles to simulation angles using the inverse relation:
        
        sim[0] = -real[0]
        sim[1] = 90 - real[1]
        sim[2] = real[2] - 90
        sim[3] = real[3] - 90
        sim[4] = 90 - real[4]
        sim[5] = real[5]
        
        Parameters:
            real_angles (array-like): A list or numpy array of 6 real angles.
        
        Returns:
            np.ndarray: A numpy array of 6 simulation angles.
        """
        real_angles = np.array(real_angles)
        sim = np.empty_like(real_angles)
        
        sim[0] = -real_angles[0]
        sim[1] = 90 - real_angles[1]
        sim[2] = real_angles[2] - 90
        sim[3] = real_angles[3] - 90
        sim[4] = 90 - real_angles[4]
        sim[5] = real_angles[5]
        
        return sim
    
    def get_tcp_pose(self):
        try:
            num_joints = p.getNumJoints(self.robot)
            tcp_state = p.getLinkState(self.robot, num_joints - 1)
            return tcp_state[4], tcp_state[5]
        except Exception as e:
            logging.error(f"Error retrieving TCP pose: {e}")
            return None, None
    
    def differential_ik(self, target_position, target_orientation, max_iters=100, threshold=1e-3, damping=0.01):
        """
        Compute joint angles using a differential IK approach with 6D control.
        
        Parameters:
            target_position (list or np.array): Desired [x, y, z] position.
            target_orientation (list or np.array): Desired quaternion [x, y, z, w].
            max_iters (int): Maximum number of iterations.
            threshold (float): Convergence threshold for combined error.
            damping (float): Damping factor for the damped least squares method.
            
        Returns:
            np.ndarray: Final joint angles in radians.
        """
        # Helper functions for quaternion operations
        def quat_inverse(q):
            return [-q[0], -q[1], -q[2], q[3]]
        
        def quat_multiply(q1, q2):
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            x = w1*x2 + x1*w2 + y1*z2 - z1*y2
            y = w1*y2 - x1*z2 + y1*w2 + z1*x2
            z = w1*z2 + x1*y2 - y1*x2 + z1*w2
            w = w1*w2 - x1*x2 - y1*y2 - z1*z2
            return [x, y, z, w]
        
        joint_angles = np.array([self.joint_angles.get(i, 0.0) for i in range(self.num_joints)])
        
        for _ in range(max_iters):
            # Get current end-effector state
            tcp_state = p.getLinkState(self.robot, self.num_joints - 1, computeForwardKinematics=True)
            current_position = np.array(tcp_state[4])
            current_orientation = tcp_state[5]  # quaternion [x,y,z,w]
            
            # Compute position error
            pos_error = np.array(target_position) - current_position
            
            # Compute orientation error.
            # Relative quaternion: q_err = target_orientation * inv(current_orientation)
            q_inv = quat_inverse(current_orientation)
            q_err = quat_multiply(target_orientation, q_inv)
            # For small angle differences, orientation error vector = 2 * (vector part of q_err)
            ori_error = 2 * np.array(q_err[:3])
            
            # Combine errors
            error = np.concatenate((pos_error, ori_error))
            if np.linalg.norm(error) < threshold:
                break
            
            # Compute Jacobians: translation and rotation parts
            jac_t, jac_r = p.calculateJacobian(
                self.robot,
                self.num_joints - 1,
                [0, 0, 0],
                joint_angles.tolist(),
                [0.0]*self.num_joints,
                [0.0]*self.num_joints
            )
            J_trans = np.array(jac_t)   # 3 x n
            J_rot = np.array(jac_r)     # 3 x n
            
            # Form full 6-D Jacobian
            J_full = np.vstack((J_trans, J_rot))  # 6 x n
            
            # Compute damped pseudo-inverse of full Jacobian
            JT = J_full.T
            inv_term = np.linalg.inv(J_full @ JT + damping * np.eye(J_full.shape[0]))
            J_inv = JT @ inv_term
            
            # Compute change in joint angles
            delta_theta = J_inv @ error
            
            # Incorporate joint limits into the update so that solutions remain feasible.
            # Given limits (in degrees):
            #   Lower limits: [-110, -110,  110,   20,  -20,  110]
            #   Upper limits: [ 110,  110, -110, -200,  200,  -10]
            lower_limits_deg = np.array([-110, -110,  110,   20,  -20,  110])
            upper_limits_deg = np.array([ 110,  110, -110, -200,  200,  -10])
            lower_limits = np.radians(lower_limits_deg)
            upper_limits = np.radians(upper_limits_deg)
            min_limits = np.minimum(lower_limits, upper_limits)
            max_limits = np.maximum(lower_limits, upper_limits)
            
            # Modify delta_theta for feasibility on a per-joint basis.
            for i in range(self.num_joints):
                candidate = joint_angles[i] + delta_theta[i]
                if candidate > max_limits[i]:
                    delta_theta[i] = max_limits[i] - joint_angles[i]
                elif candidate < min_limits[i]:
                    delta_theta[i] = min_limits[i] - joint_angles[i]
            
            joint_angles += delta_theta
            
            # Update simulation state (convert to degrees for set_joint_states)
            self.set_joint_states(np.degrees(joint_angles))
            p.stepSimulation()
            time.sleep(self.time_step)
        
        return joint_angles
    
    def run(self, simulation_mode='predefined'):
        if simulation_mode == 'predefined':
            logging.info("Starting infinite PyBullet simulation loop with predefined joint states")
            self.init_simulation()
            # Define target joint states (in degrees)
            target_states_deg = [
                [0, 90, -90, -90, 90, 0],    # extended
                [-90, 0, 0, 0, 0, 90],   # rotated
                [0, -90, 90, 0, 90, 0]       # home
            ]
            state_index = 0
            last_time = time.time()
            try:
                while True:
                    current_time = time.time()
                    if current_time - last_time >= 3.0:
                        self.set_joint_states(target_states_deg[state_index])
                        state_index = (state_index + 1) % len(target_states_deg)
                        last_time = current_time
                    self.step_simulation()
            except KeyboardInterrupt:
                logging.info("Simulation terminated by user")
                p.disconnect()
        elif simulation_mode == 'real':
            logging.info("Starting PyBullet simulation loop with real robot angles")
            self.init_simulation()
            self.simulate_real = True
            last_update = time.time()
            try:
                while True:
                    current_time = time.time()
                    if current_time - last_update >= 1.0 and self.external_real_angles is not None:
                        sim_angles = RobotSimulation.real_to_sim(self.external_real_angles)
                        self.set_joint_states(sim_angles.tolist())
                        last_update = current_time
                    self.step_simulation()
            except KeyboardInterrupt:
                logging.info("Simulation terminated by user")
                p.disconnect()
        elif simulation_mode == 'user':
            logging.info("Starting PyBullet simulation in user input mode")
            self.init_simulation()
            try:
                while True:
                    print("Current tcp position:", self.get_tcp_pose())
                    inp = input("Enter target position as x,y,z (or type 'exit' to quit): ")
                    if inp.strip().lower() == 'exit':
                        break
                    try:
                        vals = [float(x) for x in inp.split(",")]
                        if len(vals) != 3:
                            print("Please provide exactly three comma-separated values.")
                            continue
                    except Exception as e:
                        print(f"Invalid input: {e}")
                        continue
                    target_position = vals
                    # Fixed target orientation (identity quaternion)
                    target_orientation = [0, 0.707, 0.707, 0]
                    # Use differential IK instead of PyBullet's IK
                    ik_solution = self.differential_ik(target_position, target_orientation)
                    # Update joint states (convert radians to degrees)
                    self.set_joint_states(np.degrees(ik_solution))
                    t_start = time.time()
                    while time.time() - t_start < 3.0:
                        self.step_simulation()
            except KeyboardInterrupt:
                logging.info("User input mode terminated by user")
            p.disconnect()
        elif simulation_mode == 'draw':
            logging.info("Starting simulation in draw mode (drawing a box with smooth interpolation)")
            self.init_simulation()
            # Define waypoints that form a rectangular box:
            # Corners: (0.05, -0.15, 0.15), (0.05, -0.25, 0.15), (-0.05, -0.25, 0.15), (-0.05, -0.15, 0.15)
            waypoints = [
                [0.05, -0.15, 0.15],
                [0.05, -0.25, 0.15],
                [-0.05, -0.25, 0.15],
                [-0.05, -0.15, 0.15]
            ]
            # Append the first point to close the loop
            waypoints.append(waypoints[0])
            # Fixed target orientation for drawing
            target_orientation = [0, 0.707, 0.707, 0]
            num_interp_steps = 10  # Number of interpolation steps between waypoints
            try:
                for _ in range(20):  # Repeat the drawing twice
                    for i in range(len(waypoints)-1):
                        start_pt = np.array(waypoints[i])
                        end_pt = np.array(waypoints[i+1])
                        for alpha in np.linspace(0, 1, num_interp_steps):
                            interp_target = (1 - alpha) * start_pt + alpha * end_pt
                            logging.info("Moving to interpolated target: %s", interp_target)
                            ik_solution = self.differential_ik(interp_target.tolist(), target_orientation, max_iters=20)
                            self.set_joint_states(np.degrees(ik_solution))
                            t_start = time.time()
                            while time.time() - t_start < 0.1:
                                self.step_simulation()
                    logging.info("Completed drawing the box.")
            except KeyboardInterrupt:
                logging.info("Draw mode terminated by user")
            p.disconnect()
        else:
            logging.error("Unknown simulation_mode: choose 'predefined', 'real', 'user', or 'draw'")

if __name__ == "__main__":
    # Provide the full path to the URDF file
    urdf_path = "/Users/denizbekleyisseven/workspace/SO100_Control/SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf"
    sim = RobotSimulation(urdf_path)
    # To use real robot angles simulation, update sim.external_real_angles from an external source and call run with 'real'
    # For example, you can test with:
    # sim.external_real_angles = [10, 20, 30, 40, 50, 60]
    # sim.run(simulation_mode='real')
    sim.run(simulation_mode='predefined')
