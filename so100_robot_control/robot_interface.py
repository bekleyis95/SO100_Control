import logging
from pathlib import Path
import numpy as np
import torch
import time

from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower
from so100_robot_control.configs.configs import SO100Config
from so100_robot_control.simulation.robot_kinematics import RobotKinematics

# Ordered joint names — must match the motor IDs in SOFollower.
_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Gripper angle constants — verified against physical hardware calibration.
# In degrees (use_degrees=True): 0° is the Feetech STS3215 centre/open position;
# 40° closes the jaw.
_GRIPPER_OPEN   = 40.0
_GRIPPER_CLOSED = 0.0


class RobotInterface:
    """
    Hardware abstraction layer for the SO-100 arm.

    Parameters
    ----------
    config : SO100Config, optional
        Robot configuration.  Defaults to SO100Config().
    mock : bool
        When True, the hardware connection is skipped (simulation-only mode).
    """

    def __init__(self, config=None, mock=False):
        self.logger = logging.getLogger(__name__)
        self.mock = mock

        if config is None:
            config = SO100Config()

        # In mock/simulate mode the hardware robot is never used.
        if mock:
            self.robot = None
        else:
            # Build lerobot 0.4+ SOFollowerRobotConfig from our SO100Config.
            lerobot_cfg = SOFollowerRobotConfig(
                id=config.robot_id,
                port=config.port,
                calibration_dir=Path(config.calibration_dir) if config.calibration_dir else None,
                max_relative_target=config.max_relative_target,
                use_degrees=config.use_degrees,
                cameras=config.cameras,
            )
            self.robot = SOFollower(lerobot_cfg)
        self._calibrate = getattr(config, "calibrate", False)

        self.kinematics = RobotKinematics(urdf_path=config.urdf_path)

        # Initialise state placeholders
        self.joint_angles = torch.zeros(6)
        self.tcp = np.zeros(3)

        if not mock:
            self._connect_and_home()

    def _connect_and_home(self):
        """Connect to hardware and move to home, blocking until arrival."""
        self.robot.connect(calibrate=self._calibrate)
        initial_angles, _ = self.get_observation()

        if initial_angles[1] < 0.0:
            raise ValueError(
                "Robot shoulder lift is in an invalid negative position. "
                "Rotate the arm past the singularity before starting."
            )

        home = torch.tensor([-24.1209, 148.2198, -27.4725, -137.5385, 40.2198, 18.4867])
        self.send_action(home)

        # Block until every joint is within tolerance of the home pose so the
        # control loop always starts from a known, settled position.
        _TOLERANCE_DEG = 8.0
        _TIMEOUT_S     = 15.0
        _POLL_S        = 0.05
        t0 = time.time()
        while time.time() - t0 < _TIMEOUT_S:
            current, _ = self.get_observation()
            if torch.max(torch.abs(current - home)).item() < _TOLERANCE_DEG:
                break
            time.sleep(_POLL_S)
        else:
            print("Warning: homing timed out — continuing anyway.")

    # ------------------------------------------------------------------
    # Core interface
    # ------------------------------------------------------------------

    def get_observation(self):
        """
        Read joint state from hardware (or return current state in mock mode).

        Returns
        -------
        joint_angles : torch.Tensor  shape (6,)  — degrees
        tcp          : np.ndarray    shape (3,)   — Cartesian position (m)
        """
        if self.mock:
            return self.joint_angles, self.tcp

        obs = self.robot.get_observation()
        if obs is None:
            self.logger.error("Failed to capture observation.")
            return self.joint_angles, self.tcp

        self.joint_angles = torch.tensor(
            [obs[f"{name}.pos"] for name in _JOINT_NAMES],
            dtype=torch.float32,
        )
        self.tcp, _ = self.kinematics.fk(self.joint_angles, real_robot=True)
        return self.joint_angles, self.tcp

    def send_action(self, action: torch.Tensor):
        """Send a joint position command (degrees, 6-D tensor)."""
        assert isinstance(action, torch.Tensor), "action must be a torch.Tensor"
        if not self.mock:
            action_dict = {
                f"{name}.pos": float(action[i])
                for i, name in enumerate(_JOINT_NAMES)
            }
            self.robot.send_action(action_dict)
        self.joint_angles = action

    def grasp(self, close: bool):
        """Open or close the gripper."""
        angles = self.joint_angles.clone()
        angles[-1] = _GRIPPER_CLOSED if close else _GRIPPER_OPEN
        self.send_action(angles)

    def stop_robot(self):
        """Disconnect from hardware (no-op in mock mode)."""
        if not self.mock:
            self.robot.disconnect()
        self.logger.info("Robot disconnected.")

    # ------------------------------------------------------------------
    # High-level TCP moves (useful from notebooks / scripts)
    # ------------------------------------------------------------------

    def move_tcp(self, delta: np.ndarray, interpolation_steps: int = 30):
        """Move TCP by a relative delta (metres)."""
        assert isinstance(delta, np.ndarray), "delta must be a numpy array"
        current_angles, current_tcp = self.get_observation()
        target_tcp = current_tcp + delta
        target_sim = self.kinematics.ik(target_tcp, current_angles, real_robot=True)
        target_real = self.kinematics.sim_to_real(target_sim)
        full_target = np.append(target_real, current_angles.numpy()[5])

        if np.linalg.norm(delta) < 0.01:
            self.send_action(torch.tensor(full_target, dtype=torch.float32))
        else:
            for a in np.linspace(0, 1, interpolation_steps):
                interp = (1 - a) * current_angles.numpy() + a * full_target
                self.send_action(torch.tensor(interp, dtype=torch.float32))
                time.sleep(1 / 30)

    def move_tcp_to(self, target_pos: np.ndarray, duration: float = 1.0):
        """Move TCP to an absolute Cartesian position over `duration` seconds."""
        assert isinstance(target_pos, np.ndarray), "target_pos must be a numpy array"
        current_angles, _ = self.get_observation()
        target_sim = self.kinematics.ik(target_pos, current_angles, real_robot=True)
        target_real = self.kinematics.sim_to_real(target_sim)
        full_target = np.append(target_real, current_angles.numpy()[5])

        steps = max(1, int(30 * duration))
        for a in np.linspace(0, 1, steps):
            interp = (1 - a) * current_angles.numpy() + a * full_target
            self.send_action(torch.tensor(interp, dtype=torch.float32))
            time.sleep(1 / 30)

    def dynamic_tcp_control(self, read_input_fn, dt=0.02, ik_period=0.2,
                             interp_alpha=None):
        """
        Continuously move toward a dynamically updated TCP target.

        Parameters
        ----------
        read_input_fn : callable
            Returns a small np.ndarray([dx, dy, dz]) delta each call, or None.
        dt : float
            Control loop period in seconds.
        ik_period : float
            How often to recompute IK.
        interp_alpha : float or None
            Step fraction per dt.  Defaults to dt / ik_period.
        """
        if interp_alpha is None:
            interp_alpha = dt / ik_period

        current_q, current_tcp = self.get_observation()
        target_tcp = current_tcp.copy()
        target_q = current_q.numpy()[:5]
        last_ik_time = time.time()

        try:
            while True:
                t0 = time.time()

                delta = read_input_fn()
                if delta is not None:
                    target_tcp = target_tcp + delta

                if t0 - last_ik_time >= ik_period:
                    target_sim = self.kinematics.ik(target_tcp, current_q, real_robot=True)
                    target_q = self.kinematics.sim_to_real(target_sim)
                    last_ik_time = t0

                current_5 = current_q.numpy()[:5]
                new_5 = (1 - interp_alpha) * current_5 + interp_alpha * target_q
                gripper = current_q.numpy()[5]
                new_q = np.append(new_5, gripper)

                self.send_action(torch.tensor(new_q, dtype=torch.float32))
                current_q = torch.tensor(new_q, dtype=torch.float32)

                to_sleep = dt - (time.time() - t0)
                if to_sleep > 0:
                    time.sleep(to_sleep)

        except KeyboardInterrupt:
            print("Dynamic control stopped.")


# Backward-compatible alias
SO100RobotInterface = RobotInterface
