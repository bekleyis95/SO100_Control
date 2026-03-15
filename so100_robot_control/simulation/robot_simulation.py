import numpy as np
import pybullet as p
import pybullet_data
import time
import logging
import math

logging.basicConfig(level=logging.INFO)


class RobotSimulation:
    """PyBullet-based 3-D visualisation of the SO-100 arm."""

    def __init__(self, urdf_path):
        self.urdf_path = urdf_path
        self.joint_angles = {}
        self.robot = None
        self.num_joints = 0
        # Actuated (non-fixed) joint indices — populated by init_simulation().
        self._actuated_joints: list[int] = []

    def init_simulation(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Disconnect the physics server cleanly if URDF loading fails so no
        # dangling GUI window / process is left behind.
        try:
            self.robot = p.loadURDF(self.urdf_path, useFixedBase=True)
        except Exception:
            p.disconnect()
            raise

        p.resetDebugVisualizerCamera(
            cameraDistance=1.0, cameraYaw=45, cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0],
        )
        self.num_joints = p.getNumJoints(self.robot)

        # Identify actuated joints — PyBullet's calculateJacobian requires
        # a position array sized to the number of non-fixed DOFs only.
        self._actuated_joints = [
            i for i in range(self.num_joints)
            if p.getJointInfo(self.robot, i)[2] != p.JOINT_FIXED
        ]

        for i in range(self.num_joints):
            state = p.getJointState(self.robot, i)
            self.joint_angles[i] = state[0]
        logging.info(
            "URDF loaded: %d total joints, %d actuated.",
            self.num_joints, len(self._actuated_joints),
        )

    def step_simulation(self):
        """Advance the physics simulation by one step.

        Timing is the caller's responsibility — no sleep is performed here.
        """
        p.stepSimulation()
        for i in range(p.getNumJoints(self.robot)):
            state = p.getJointState(self.robot, i)
            new_angle = state[0]
            if not math.isclose(self.joint_angles.get(i, 0.0), new_angle, rel_tol=1e-5):
                self.joint_angles[i] = new_angle

    def set_joint_states(self, target_angles_deg):
        """
        Set joint positions from a degree array.

        Iterates only up to min(num_joints, len(target_angles_deg)) so callers
        can safely pass a shorter array (e.g. 5-D or 6-D for an 8-joint URDF).
        """
        num = min(p.getNumJoints(self.robot), len(target_angles_deg))
        for i in range(num):
            target_rad = math.radians(target_angles_deg[i])
            p.resetJointState(self.robot, i, target_rad)
            self.joint_angles[i] = target_rad

    def get_tcp_pose(self):
        try:
            tcp_state = p.getLinkState(self.robot, self.num_joints - 1)
            return tcp_state[4], tcp_state[5]
        except Exception as e:
            logging.error("Error retrieving TCP pose: %s", e)
            return None, None

    def differential_ik(self, target_position, target_orientation,
                         max_iters=100, threshold=1e-3, damping=0.01):
        """6-D damped-least-squares IK using PyBullet Jacobians.

        Uses only the actuated (non-fixed) joints for the Jacobian so the
        DOF count matches what PyBullet expects.  Joint limits are read from
        the URDF rather than from a hardcoded table.

        Returns
        -------
        np.ndarray shape (num_joints,)
            Joint angles in radians for *all* URDF joints (fixed joints are
            preserved at their current values, actuated joints are updated).
            This matches the shape returned by ``p.getNumJoints`` so callers
            can pass the result directly to ``set_joint_states`` after
            converting to degrees.
        """

        def quat_inverse(q):
            return [-q[0], -q[1], -q[2], q[3]]

        def quat_multiply(q1, q2):
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            return [
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2,
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
            ]

        n_act = len(self._actuated_joints)

        # Current actuated joint angles (radians)
        q = np.array([self.joint_angles.get(i, 0.0) for i in self._actuated_joints])

        # Joint limits from URDF for actuated joints
        lo = np.full(n_act, -np.pi)
        hi = np.full(n_act, np.pi)
        for k, idx in enumerate(self._actuated_joints):
            info = p.getJointInfo(self.robot, idx)
            lo[k] = info[8]   # jointLowerLimit
            hi[k] = info[9]   # jointUpperLimit
            # PyBullet returns 0,0 for unlimited joints — keep ±π in that case
            if lo[k] == 0.0 and hi[k] == 0.0:
                lo[k], hi[k] = -np.pi, np.pi

        for _ in range(max_iters):
            tcp_state = p.getLinkState(self.robot, self.num_joints - 1,
                                       computeForwardKinematics=True)
            curr_pos = np.array(tcp_state[4])
            curr_ori = tcp_state[5]

            pos_err = np.array(target_position) - curr_pos
            q_err = quat_multiply(target_orientation, quat_inverse(curr_ori))
            ori_err = 2.0 * np.array(q_err[:3])
            err = np.concatenate((pos_err, ori_err))

            if np.linalg.norm(err) < threshold:
                break

            # PyBullet expects positions for actuated DOFs only
            jac_t, jac_r = p.calculateJacobian(
                self.robot, self.num_joints - 1, [0, 0, 0],
                q.tolist(),
                [0.0] * n_act,
                [0.0] * n_act,
            )
            J = np.vstack((np.array(jac_t), np.array(jac_r)))
            J_inv = J.T @ np.linalg.inv(J @ J.T + damping * np.eye(J.shape[0]))
            dq = J_inv @ err

            for i in range(n_act):
                candidate = q[i] + dq[i]
                dq[i] = np.clip(candidate, lo[i], hi[i]) - q[i]

            q += dq

            # Only update actuated joints — skip fixed joints entirely so
            # PyBullet is not asked to resetJointState on JOINT_FIXED indices.
            for k, idx in enumerate(self._actuated_joints):
                p.resetJointState(self.robot, idx, q[k])
                self.joint_angles[idx] = q[k]
            p.stepSimulation()

        # Return a full num_joints-D array so callers get a consistent shape
        # regardless of how many joints are fixed.
        full = np.array([self.joint_angles.get(i, 0.0) for i in range(self.num_joints)])
        return full

    def run(self, simulation_mode="predefined"):
        if simulation_mode == "predefined":
            logging.info("Starting PyBullet simulation with predefined joint states")
            self.init_simulation()
            target_states = [
                [0, 90, -90, -90, 90, 0],
                [-90, 0, 0, 0, 0, 90],
                [0, -90, 90, 0, 90, 0],
            ]
            idx, last_time = 0, time.time()
            try:
                while True:
                    if time.time() - last_time >= 3.0:
                        self.set_joint_states(target_states[idx])
                        idx = (idx + 1) % len(target_states)
                        last_time = time.time()
                    self.step_simulation()
                    time.sleep(0.01)  # only this standalone loop needs a sleep
            except KeyboardInterrupt:
                logging.info("Simulation terminated by user")
                p.disconnect()

        elif simulation_mode == "user":
            logging.info("Starting PyBullet in user-input mode")
            self.init_simulation()
            try:
                while True:
                    print("Current TCP:", self.get_tcp_pose())
                    inp = input("Enter target x,y,z (or 'exit'): ")
                    if inp.strip().lower() == "exit":
                        break
                    try:
                        vals = [float(x) for x in inp.split(",")]
                        if len(vals) != 3:
                            print("Need exactly 3 values.")
                            continue
                    except Exception as e:
                        print(f"Invalid input: {e}")
                        continue
                    ik_sol = self.differential_ik(vals, [0, 0.707, 0.707, 0])
                    self.set_joint_states(np.degrees(ik_sol))
                    t0 = time.time()
                    while time.time() - t0 < 3.0:
                        self.step_simulation()
                        time.sleep(0.01)
            except KeyboardInterrupt:
                logging.info("User-input mode terminated")
            p.disconnect()

        elif simulation_mode == "draw":
            logging.info("Starting PyBullet in draw mode")
            self.init_simulation()
            waypoints = [
                [0.05, -0.15, 0.15], [0.05, -0.25, 0.15],
                [-0.05, -0.25, 0.15], [-0.05, -0.15, 0.15],
            ]
            waypoints.append(waypoints[0])
            ori = [0, 0.707, 0.707, 0]
            try:
                for _ in range(2):
                    for i in range(len(waypoints) - 1):
                        start, end = np.array(waypoints[i]), np.array(waypoints[i + 1])
                        for alpha in np.linspace(0, 1, 10):
                            pt = (1 - alpha) * start + alpha * end
                            sol = self.differential_ik(pt.tolist(), ori, max_iters=20)
                            self.set_joint_states(np.degrees(sol))
                            t0 = time.time()
                            while time.time() - t0 < 0.1:
                                self.step_simulation()
                                time.sleep(0.01)
            except KeyboardInterrupt:
                logging.info("Draw mode terminated")
            p.disconnect()

        else:
            logging.error("Unknown simulation_mode: choose 'predefined', 'user', or 'draw'")
