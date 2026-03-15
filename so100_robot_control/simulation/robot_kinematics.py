from yourdfpy import URDF
import sympy as sp
import numpy as np
from scipy.spatial.transform import Rotation as R

# The end-effector link used for FK / IK — the child of the last arm joint
# (Wrist_Roll) before the gripper jaw.  Named explicitly so that changes to
# URDF joint ordering don't silently pick the wrong link.
_END_LINK = "Fixed_Jaw"


class RobotKinematics:
    """
    FK / IK engine for a 5-DOF arm using a symbolic Jacobian.

    real space  — angles in degrees as the hardware reports them (6-D: 5 arm + gripper)
    sim space   — angles in radians in the URDF convention (5-D: arm joints only, no gripper)
    """

    def __init__(self, urdf_path: str, q_null_deg=None, null_gain=0.1):
        self.urdf_path = urdf_path
        self.robot = URDF.load(urdf_path)
        self.joint_names = list(self.robot.actuated_joint_names)
        self.joint_names.remove("Jaw")  # gripper is not part of IK chain
        self.q_syms = sp.symbols(" ".join(self.joint_names))
        self.base_link = self.robot.base_link
        self.end_link = _END_LINK
        self.chain = self._get_joint_chain()
        self.ee_pose_symbolic = self._build_fk_chain()
        self.jacobian_symbolic = self.ee_pose_symbolic[:3, 3].jacobian(self.q_syms)
        self.joint_limits = self._extract_joint_limits()

        n = len(self.joint_names)
        if q_null_deg is None:
            self.q_null = np.radians([0.0, 90.0, 90.0, 90.0, 90.0])
        else:
            arr = np.asarray(q_null_deg)
            if arr.shape[0] != n:
                raise ValueError(f"q_null_deg must have length {n}, got {arr.shape[0]}")
            self.q_null = np.radians(arr)
        self.null_gain = null_gain

        # Compile symbolic expressions to fast NumPy functions once at init.
        syms = list(self.q_syms)
        self._fk_fn = sp.lambdify(syms, self.ee_pose_symbolic, "numpy")
        self._jacobian_fn = sp.lambdify(syms, self.jacobian_symbolic, "numpy")

    # ------------------------------------------------------------------
    # Chain building
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Symbolic FK helpers
    # ------------------------------------------------------------------

    def _rot(self, axis, theta):
        x, y, z = axis
        c, s, C = sp.cos(theta), sp.sin(theta), 1 - sp.cos(theta)
        return sp.Matrix([
            [x*x*C + c,   x*y*C - z*s, x*z*C + y*s, 0],
            [y*x*C + z*s, y*y*C + c,   y*z*C - x*s, 0],
            [z*x*C - y*s, z*y*C + x*s, z*z*C + c,   0],
            [0,           0,           0,           1],
        ])

    def _trans(self, x, y, z):
        return sp.Matrix([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1],
        ])

    def _build_fk_chain(self):
        # 7 cm Y-axis offset applied before the kinematic chain.
        # This is an empirically-tuned correction — it accounts for the
        # distance from the URDF world frame origin to the physical base
        # mounting point that is not encoded in the URDF itself.
        # If you re-derive the URDF with the correct base transform, set this to 0.
        T = self._trans(0, 0.07, 0.0)
        q_map = dict(zip(self.joint_names, self.q_syms))
        for joint in self.chain:
            T_origin = sp.Matrix(joint.origin)
            if joint.type == "fixed":
                T *= T_origin
            elif joint.type in ("revolute", "continuous"):
                q = q_map[joint.name]
                T *= T_origin * self._rot(joint.axis, q)
        T *= self._get_tcp_offset_transform()
        return T

    def _get_tcp_offset_transform(self):
        # -7.3 cm offset along the Y-axis of the last link, applied after the
        # kinematic chain.  Empirically tuned to align the FK output with the
        # physical tool-centre point.  Set to 0 if your URDF already models
        # the full TCP offset.
        return sp.Matrix([
            [1, 0, 0,  0.0],
            [0, 1, 0, -0.073],
            [0, 0, 1,  0.0],
            [0, 0, 0,  1.0],
        ])

    # ------------------------------------------------------------------
    # FK / Jacobian
    # ------------------------------------------------------------------

    def fk(self, joint_values, real_robot=False):
        """
        Forward kinematics.

        Parameters
        ----------
        joint_values : array-like
            If real_robot=False: 5-D sim-space angles (radians).
            If real_robot=True:  5- or 6-D real-space angles (degrees).
        real_robot : bool
            Convert from real space before computing FK.

        Returns
        -------
        position : np.ndarray shape (3,)
        rotation : np.ndarray shape (3,)  (Euler XYZ degrees)
        """
        q = np.asarray(joint_values, dtype=float)
        if real_robot:
            q = self.real_to_sim(q)  # → 5-D sim radians
        q = q[:5]
        T = np.array(self._fk_fn(*q), dtype=np.float64)
        position = T[:3, 3]
        rotation = R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
        return position, rotation

    def jacobian(self, joint_values):
        """Position Jacobian (3×5) at the given 5-D sim-space joint angles."""
        q = np.asarray(joint_values, dtype=float)[:5]
        return np.array(self._jacobian_fn(*q), dtype=np.float64)

    # ------------------------------------------------------------------
    # IK
    # ------------------------------------------------------------------

    def ik(self, target_pos, initial_guess=None, tol=1e-4, max_iters=100, real_robot=False):
        """
        Iterative Jacobian pseudo-inverse IK with null-space posture control.

        Parameters
        ----------
        target_pos : array-like  shape (3,)
            Desired end-effector position in Cartesian space.
        initial_guess : array-like or None
            If real_robot=False: 5-D sim-space angles (radians).
            If real_robot=True:  5- or 6-D real-space angles (degrees).
        real_robot : bool
        tol, max_iters : convergence parameters

        Returns
        -------
        q : np.ndarray shape (5,)  — sim-space angles (radians)
        """
        n = len(self.joint_names)

        if initial_guess is None:
            q = np.zeros(n)
        elif real_robot:
            q = self.real_to_sim(initial_guess)
        else:
            q = np.asarray(initial_guess, dtype=float)[:n]

        target = np.asarray(target_pos, dtype=float)

        for _ in range(max_iters):
            ee_pos, _ = self.fk(q)
            err = target - ee_pos
            if np.linalg.norm(err) < tol:
                return q

            J = self.jacobian(q)           # (3×n)
            J_pinv = np.linalg.pinv(J)     # (n×3)

            dq_prim = J_pinv @ err
            N = np.eye(n) - J_pinv @ J     # null-space projector
            dq_null = self.null_gain * (N @ (self.q_null - q))

            q = q + dq_prim + dq_null

            for idx, name in enumerate(self.joint_names):
                lo, hi = self.joint_limits[name]
                q[idx] = np.clip(q[idx], lo, hi)

        raise ValueError(
            f"IK did not converge after {max_iters} iterations "
            f"(residual={np.linalg.norm(err):.4f})"
        )

    # ------------------------------------------------------------------
    # Angle space conversions
    # ------------------------------------------------------------------

    def real_to_sim(self, real_angles):
        """
        Convert real robot angles (degrees, 5- or 6-D) → sim angles (radians, 5-D).

        The gripper (index 5) is stripped; only the 5 arm joints are mapped.
        Mapping:
            sim[0] = -real[0]
            sim[1] =  90 - real[1]
            sim[2] =  real[2] - 90
            sim[3] =  real[3] - 90
            sim[4] =  90 - real[4]
        """
        r = np.asarray(real_angles, dtype=float)[:5]
        sim = np.array([
            -r[0],
            90.0 - r[1],
            r[2] - 90.0,
            r[3] - 90.0,
            90.0 - r[4],
        ])
        return np.radians(sim)

    def sim_to_real(self, sim_angles):
        """
        Convert sim angles (radians, 5-D) → real robot angles (degrees, 5-D).

        Inverse of real_to_sim (gripper not included):
            real[0] = -sim_deg[0]
            real[1] = -sim_deg[1] + 90
            real[2] =  sim_deg[2] + 90
            real[3] =  sim_deg[3] + 90
            real[4] = -sim_deg[4] + 90
        """
        s = np.degrees(np.asarray(sim_angles, dtype=float)[:5])
        real = np.array([
            -s[0],
            -s[1] + 90.0,
            s[2] + 90.0,
            s[3] + 90.0,
            -s[4] + 90.0,
        ])
        return real

    def show(self):
        self.robot.show()
