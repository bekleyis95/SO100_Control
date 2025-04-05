from yourdfpy import URDF
import sympy as sp
import numpy as np
from scipy.spatial.transform import Rotation as R

class RobotKinematics:
    def __init__(self, urdf_path: str):
        self.robot = URDF.load(urdf_path)
        self.joint_names = self.robot.actuated_joint_names
        self.q_syms = sp.symbols(' '.join(self.joint_names))
        self.base_link = self.robot.base_link
        self.end_link = self.robot.robot.joints[-1].child
        self.chain = self._get_joint_chain()
        self.ee_pose_symbolic = self._build_fk_chain()
        self.jacobian_symbolic = self.ee_pose_symbolic[:3, 3].jacobian(self.q_syms)
        self.joint_limits = self._extract_joint_limits()

    def _get_joint_chain(self):
        chain = []
        current_link = self.end_link
        while current_link != self.base_link:
            joint = next(j for j in self.robot.robot.joints if j.child == current_link)
            chain.insert(0, joint)
            current_link = joint.parent
        return chain

    def _extract_joint_limits(self):
        limits = {}
        for joint in self.chain:
            if joint.limit:
                limits[joint.name] = (joint.limit.lower, joint.limit.upper)
            else:
                limits[joint.name] = (-np.inf, np.inf)
        return limits

    def _rot(self, axis, theta):
        x, y, z = axis
        c = sp.cos(theta)
        s = sp.sin(theta)
        C = 1 - c
        return sp.Matrix([
            [x*x*C + c,   x*y*C - z*s, x*z*C + y*s, 0],
            [y*x*C + z*s, y*y*C + c,   y*z*C - x*s, 0],
            [z*x*C - y*s, z*y*C + x*s, z*z*C + c,   0],
            [0,           0,           0,           1]
        ])

    def _trans(self, x, y, z):
        return sp.Matrix([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

    def _build_fk_chain(self):
        T = sp.eye(4)
        q_map = dict(zip(self.joint_names, self.q_syms))

        for joint in self.chain:
            T_origin = sp.Matrix(joint.origin)
            if joint.type == 'fixed':
                T *= T_origin
            elif joint.type == 'revolute' or joint.type == 'continuous':
                q = q_map[joint.name]
                axis = joint.axis
                R_sym = self._rot(axis, q)
                T *= T_origin * R_sym
        return T

    def fk(self, joint_values, real_robot=False):
        if real_robot:
            joint_values = self.sim_to_real(joint_values)
        subs = dict(zip(self.q_syms, joint_values))
        T_numeric = self.ee_pose_symbolic.evalf(subs=subs)
        T_np = np.array(T_numeric).astype(np.float64)
        position = T_np[:3, 3]
        rotation = R.from_matrix(T_np[:3, :3]).as_euler('xyz', degrees=True)
        return position, rotation

    def jacobian(self, joint_values):
        subs = dict(zip(self.q_syms, joint_values))
        return np.array(self.jacobian_symbolic.evalf(subs=subs)).astype(np.float64)

    def ik(self, target_pos, initial_guess=None, tol=1e-4, max_iters=100):
        if initial_guess is None:
            current_q = np.zeros(len(self.q_syms))
        else:
            current_q = np.array(initial_guess)

        for _ in range(max_iters):
            ee_pos, _ = self.fk(current_q)
            error = target_pos - ee_pos
            if np.linalg.norm(error) < tol:
                return current_q
            J = self.jacobian(current_q)
            dq = np.linalg.pinv(J) @ error
            current_q += dq

            for i, name in enumerate(self.joint_names):
                lower, upper = self.joint_limits[name]
                current_q[i] = np.clip(current_q[i], lower, upper)

        raise ValueError("IK did not converge")

    def show(self):
        self.robot.show()

    def sim_to_real(self, sim_angles):
        sim_angles = np.array(sim_angles)
        real = np.empty_like(sim_angles)
        real[0] = -sim_angles[0]
        real[1] = -sim_angles[1] + 90
        real[2] = sim_angles[2] + 90
        real[3] = sim_angles[3] + 90
        real[4] = -sim_angles[4] + 90
        real[5] = sim_angles[5]
        return real

    def real_to_sim(self, real_angles):
        real_angles = np.array(real_angles)
        sim = np.empty_like(real_angles)
        sim[0] = -real_angles[0]
        sim[1] = 90 - real_angles[1]
        sim[2] = real_angles[2] - 90
        sim[3] = real_angles[3] - 90
        sim[4] = 90 - real_angles[4]
        sim[5] = real_angles[5]
        return sim
