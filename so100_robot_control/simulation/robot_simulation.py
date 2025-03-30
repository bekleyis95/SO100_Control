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
        # Initialize camera 2 times closer to robot (adjust parameters accordingly)
        p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        # Store initial joint states
        num_joints = p.getNumJoints(self.robot)
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
                # info = p.getJointInfo(self.robot, i)
                # joint_name = info[1].decode() if isinstance(info[1], bytes) else info[1]
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
            logging.info("Set Joint '%s' (index %d) to %.5f rad", joint_name, i, target_rad)
    
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
        else:
            logging.error("Unknown simulation_mode: choose 'predefined' or 'real'")

if __name__ == "__main__":
    # Provide the full path to the URDF file
    urdf_path = "/Users/denizbekleyisseven/workspace/SO100_Control/SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf"
    sim = RobotSimulation(urdf_path)
    # To use real robot angles simulation, update sim.external_real_angles from an external source and call run with 'real'
    # For example, you can test with:
    # sim.external_real_angles = [10, 20, 30, 40, 50, 60]
    # sim.run(simulation_mode='real')
    sim.run(simulation_mode='predefined')
