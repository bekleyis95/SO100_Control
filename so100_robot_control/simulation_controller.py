import threading
import time
import numpy as np
import pybullet as p
import pyglet

from simulation.robot_simulation import RobotSimulation
from teleop_devices.joystick_listener import JoystickController

# Define coefficient constant for target updates
COEFF = 0.001

class CartesianJoystickController(JoystickController):
    def __init__(self, target, lock):
        super().__init__()
        self.target = target
        self.lock = lock
        # Override axis event to update target position with locking
        @self.joystick.event
        def on_joyaxis_motion(_, axis, value):
            if abs(value) < 0.1:
                return
            with self.lock:
                if axis == "x":
                    self.target[0] += COEFF if value > 0 else -COEFF
                elif axis == "y":
                    self.target[1] += COEFF if value > 0 else -COEFF
                elif axis == "z":
                    self.target[2] += COEFF if value > 0 else -COEFF
            print(f"Updated target position: {self.target}")
        # You can keep or override button events as needed
        @self.joystick.event
        def on_joybutton_press(_, button):
            print(f"Button {button} pressed in cartesian mode")
            # ...existing button handling if needed...

# New simulation update function scheduled by pyglet
def simulation_update(dt, sim, target_position, lock, target_orientation):
    with lock:
        desired_target = target_position.copy()
    print(f"Current target: {desired_target}")
    ik_solution = sim.differential_ik(desired_target, target_orientation, max_iters=50)
    sim.set_joint_states(np.degrees(ik_solution))

# New simulation step function scheduled by pyglet
def simulation_step(dt, sim):
    sim.step_simulation()

def main():
    # Initialize shared target and lock.
    target_position = [0.1, 0.0, 0.1]
    lock = threading.Lock()

    # Create simulation instance.
    urdf_path = "/Users/denizbekleyisseven/workspace/SO100_Control/SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf"
    sim = RobotSimulation(urdf_path)
    sim.init_simulation()
    target_orientation = [0, 0.707, 0.707, 0]

    # Instantiate our customized joystick controller that updates the target.
    joystick_ctrl = CartesianJoystickController(target_position, lock)

    print("Starting simulation in Cartesian control mode. Use joystick axes to update target.")
    # Schedule simulation update and step functions using pyglet clock (both run in main thread)
    pyglet.clock.schedule_interval(lambda dt: simulation_update(dt, sim, target_position, lock, target_orientation), 0.5)
    pyglet.clock.schedule_interval(lambda dt: simulation_step(dt, sim), sim.time_step if hasattr(sim, 'time_step') else 0.01)
    
    try:
        # Run joystick app (pyglet.event loop) in the main thread.
        joystick_ctrl.run()
    except KeyboardInterrupt:
        print("Exiting simulation controller.")
        p.disconnect()

if __name__ == "__main__":
    main()
