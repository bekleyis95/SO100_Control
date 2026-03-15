"""
RobotController — top-level orchestrator that wires together:
  - hardware / mock robot interface
  - FK / IK kinematics
  - teleop input device (joystick or keyboard)
  - optional PyBullet visualisation
"""
from __future__ import annotations

import enum
import os
import threading
import time
import argparse

import numpy as np
import torch

from so100_robot_control.robot_interface import RobotInterface
from so100_robot_control.simulation.robot_simulation import RobotSimulation
from so100_robot_control.teleop_devices.joystick_listener import (
    JointJoystickController,
    CartesianJoystickController,
)
from so100_robot_control.teleop_devices.keyboard_listener import KeyboardController


class ControlDevice(enum.Enum):
    JOYSTICK = "joystick"
    KEYBOARD = "keyboard"


class ControlMode(enum.Enum):
    CARTESIAN = "cartesian"
    JOINT = "joint"


# How often to re-sync joint state from hardware to correct dead-reckoning drift.
# At 30 Hz this is a re-sync every ~333 ms.
_RESYNC_EVERY_N_LOOPS = 10


class RobotController:
    """
    Orchestrates a robot control session.

    Parameters
    ----------
    control_device : ControlDevice
    control_mode   : ControlMode
    simulate       : bool
        When True, hardware connection is skipped and PyBullet is shown.
    robot_config : SO100Config-compatible, optional
        Passed through to RobotInterface.  Uses SO100Config() default if None.
    control_rate   : float
        Control loop frequency in Hz (default 30).
    """

    def __init__(
        self,
        control_device: ControlDevice = ControlDevice.JOYSTICK,
        control_mode: ControlMode = ControlMode.JOINT,
        simulate: bool = False,
        robot_config=None,
        control_rate: float = 30.0,
    ):
        self.control_mode = control_mode
        self.simulation_mode = simulate
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate

        # ── Robot interface ─────────────────────────────────────────────
        print("Initialising robot interface...")
        self.sdk = RobotInterface(config=robot_config, mock=simulate)

        # ── Simulation visualisation ────────────────────────────────────
        if simulate:
            urdf_path = self.sdk.kinematics.urdf_path
            print(f"Initialising PyBullet simulation (URDF: {urdf_path}) ...")
            self.simulator = RobotSimulation(urdf_path)

        # ── Teleop device ───────────────────────────────────────────────
        self.control_device = control_device
        self.controller = self._build_controller(control_device, control_mode)

        # ── Callbacks ───────────────────────────────────────────────────
        self.controller.register_shutdown_callback(self.emergency_stop)
        self.controller.register_tensor_changed_callback(self.on_tensor_changed)
        self.controller.register_log_state_callback(self.get_robot_state)
        # Simulation ticks run on the main thread so all PyBullet / OpenGL
        # calls stay on the thread that owns the GL context (required on macOS).
        if simulate:
            self.controller.register_tick_callback(self._update_simulation)

        # ── State ───────────────────────────────────────────────────────
        self.running = True
        # target_joint_angles is the desired position, updated by input and
        # commanded to the robot every tick regardless of whether input changed.
        self.target_joint_angles: torch.Tensor | None = None
        self.current_tcp: np.ndarray | None = None
        self._loop_count = 0

        # Control thread is created here but started in start()
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)

    # ------------------------------------------------------------------
    # Teleop device factory
    # ------------------------------------------------------------------

    def _build_controller(self, device: ControlDevice, mode: ControlMode):
        if device == ControlDevice.JOYSTICK:
            print("Initialising joystick controller...")
            ctrl = (JointJoystickController() if mode == ControlMode.JOINT
                    else CartesianJoystickController())
            if ctrl.joystick is None:
                print("No joystick found — falling back to keyboard.")
                self.control_device = ControlDevice.KEYBOARD
                return KeyboardController()
            return ctrl
        print("Initialising keyboard controller...")
        return KeyboardController()

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _initialize_position(self) -> bool:
        try:
            angles, tcp = self.sdk.get_observation()
            self.target_joint_angles = angles.clone()
            self.current_tcp = tcp
            print(f"Initial joint angles: {self.target_joint_angles}")
            return True
        except Exception as e:
            print(f"Error initialising position: {e}")
            return False

    def _resync_from_hardware(self):
        """Periodically read TCP from hardware for IK state tracking.

        Only updates current_tcp (used as IK initial guess in Cartesian mode).
        target_joint_angles is intentionally NOT snapped to hardware — the
        control loop always commands the target, so snapping it would cause a
        jerk whenever the re-sync fires.
        """
        try:
            _, self.current_tcp = self.sdk.get_observation()
        except Exception as e:
            print(f"Re-sync warning: {e}")

    def _control_loop(self):
        if not self._initialize_position():
            print("Failed to initialise position — stopping.")
            self.running = False
            return

        while self.running:
            try:
                t0 = time.time()
                self._loop_count += 1

                # Input updates the target — decoupled from commanding.
                cmd = self.controller.get_control_tensor()
                if not torch.all(cmd == 0):
                    try:
                        if self.control_mode == ControlMode.CARTESIAN:
                            self._apply_cartesian_command(cmd)
                        else:
                            self._apply_joint_command(cmd)
                    except Exception as e:
                        print(f"Command error: {e}")

                # Always send the current target to the robot every tick.
                # This eliminates jerk caused by gaps in commanding: the motor
                # continuously receives its goal position even when no input is
                # active, giving smooth hold behaviour without relying solely on
                # the motor's internal PID.
                try:
                    self.sdk.send_action(self.target_joint_angles)
                except Exception as e:
                    print(f"Send action error: {e}")

                # Periodic TCP re-sync for IK state (real hardware only).
                if not self.simulation_mode and self._loop_count % _RESYNC_EVERY_N_LOOPS == 0:
                    self._resync_from_hardware()

                # Rate limiting
                elapsed = time.time() - t0
                sleep_time = self.control_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    print(f"Warning: loop running slower than {self.control_rate:.0f} Hz")

            except Exception as e:
                print(f"Outer control loop error: {e}")
                time.sleep(0.1)

    def _apply_joint_command(self, cmd: torch.Tensor):
        """Update joint target by delta. Actual send happens in the main tick."""
        self.target_joint_angles = self.target_joint_angles + cmd

    def _apply_cartesian_command(self, cmd: torch.Tensor):
        """
        Update joint target via IK. Actual send happens in the main tick.

        cmd[:3] is a Cartesian position delta (metres); gripper is preserved.
        """
        delta_xyz = cmd[:3].numpy()
        target_tcp = self.current_tcp + delta_xyz

        target_sim = self.sdk.kinematics.ik(
            target_tcp,
            initial_guess=self.target_joint_angles.numpy(),
            real_robot=True,
        )
        target_real_5 = self.sdk.kinematics.sim_to_real(target_sim)
        gripper = float(self.target_joint_angles[5])
        self.target_joint_angles = torch.tensor(
            np.append(target_real_5, gripper), dtype=torch.float32
        )
        self.current_tcp = target_tcp

    # ------------------------------------------------------------------
    # Simulation visualisation update
    # Called from the control thread so it works with both pyglet (joystick)
    # and pygame (keyboard) main-thread event loops.
    # ------------------------------------------------------------------

    def _update_simulation(self):
        if self.target_joint_angles is None:
            return
        try:
            # Convert real-space → sim-space degrees for PyBullet
            sim_rad = self.sdk.kinematics.real_to_sim(self.target_joint_angles.numpy())
            sim_deg = np.degrees(sim_rad)                          # 5-D
            gripper_deg = float(self.target_joint_angles[5])
            full_deg = np.append(sim_deg, gripper_deg).tolist()    # 6-D

            self.simulator.set_joint_states(full_deg)
            self.simulator.step_simulation()
        except Exception as e:
            print(f"Simulation update error: {e}")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def on_tensor_changed(self, tensor):
        pass  # control loop polls the tensor — nothing to do here

    def emergency_stop(self):
        print("EMERGENCY STOP ACTIVATED")
        self.running = False
        try:
            if (hasattr(self.sdk, "robot") and
                    hasattr(self.sdk.robot, "is_connected") and
                    self.sdk.robot.is_connected):
                self.sdk.stop_robot()
                print("Robot stopped.")
        except Exception as e:
            print(f"Error during emergency stop: {e}")
        os._exit(0)

    def get_robot_state(self):
        try:
            obs = self.sdk.get_observation()
            print(f"Robot state: {obs}")
            return obs
        except Exception as e:
            print(f"Error getting robot state: {e}")
            return None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        print(f"Starting RobotController  rate={self.control_rate:.0f}Hz  "
              f"device={self.control_device.value}  mode={self.control_mode.value}")

        try:
            if self.simulation_mode:
                # Init PyBullet before starting the control thread so the
                # physics server is ready when _update_simulation first runs.
                self.simulator.init_simulation()

            self.control_thread.start()
            self.controller.run()   # blocks on main thread (pyglet / pygame event loop)

        except KeyboardInterrupt:
            print("\nStopping due to keyboard interrupt")
            self.stop()
        except Exception as e:
            print(f"Error in start(): {e}")
            self.stop()

    def stop(self):
        print("Stopping RobotController")
        self.running = False
        try:
            self.sdk.stop_robot()
        except Exception as e:
            print(f"Error stopping robot: {e}")
        # Only disconnect PyBullet if init_simulation() was actually called.
        # If the URDF failed to load, self.simulator exists but the physics
        # server was never started — calling p.disconnect() would raise.
        if self.simulation_mode and hasattr(self, "simulator") and self.simulator.robot is not None:
            try:
                import pybullet as p
                p.disconnect()
            except Exception:
                pass
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)


# Backward-compatible alias
CombinedController = RobotController


# ------------------------------------------------------------------
# CLI entry point
# ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SO-100 Robot Controller")
    parser.add_argument(
        "--device", type=str, default="joystick",
        choices=[e.value for e in ControlDevice],
        help="Input device: joystick or keyboard (default: joystick)",
    )
    parser.add_argument(
        "--control-mode", type=str, default="joint",
        choices=[e.value for e in ControlMode],
        help="Control mode: joint or cartesian (default: joint)",
    )
    parser.add_argument(
        "--simulate", action="store_true",
        help="Skip hardware — run in PyBullet simulation only",
    )
    parser.add_argument(
        "--robot", type=str, default=None,
        help="Path to a YAML robot config file (see robots/so100.example.yaml)",
    )
    args = parser.parse_args()

    robot_config = None
    if args.robot:
        from so100_robot_control.configs.robot_loader import load_robot_config
        robot_config = load_robot_config(args.robot)

    controller = RobotController(
        control_device=ControlDevice(args.device),
        control_mode=ControlMode(args.control_mode),
        simulate=args.simulate,
        robot_config=robot_config,
    )

    try:
        controller.start()
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
