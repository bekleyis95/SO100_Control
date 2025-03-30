import sys
import torch
import time
import threading
import argparse
from so100_robot_control.simulation.robot_simulation import RobotSimulation
from so100_robot_control.teleop_devices.joystick_listener import JoystickController
from so100_robot_control.teleop_devices.keyboard_listener import KeyboardController
from so100_robot_control.robot_interface import SO100RobotInterface

class CombinedController:
    """
    Wrapper class that allows choosing between joystick or keyboard control modes.
    Sets up the robot SDK interface and passes control to the selected controller.
    """
    def __init__(self, control_mode='joystick', simulate=False):
        # Initialize the SDK interface
        print("Initializing robot SDK interface...")
        self.sdk = SO100RobotInterface()
        
        # Simulation mode flag
        self.simulation_mode = simulate
        if self.simulation_mode:
            from so100_robot_control.simulation.robot_simulation import RobotSimulation
            urdf_path = "SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf"
            print("Initializing simulation instance...")
            self.simulator = RobotSimulation(urdf_path)
        
        # Select and initialize the controller based on mode
        self.control_mode = control_mode
        if control_mode == 'joystick':
            print("Initializing joystick controller...")
            self.controller = JoystickController()
            if self.controller.joystick is None:
                print("No joystick found. Falling back to keyboard control.")
                self.control_mode = 'keyboard'
                self.controller = KeyboardController()
        else:
            print("Initializing keyboard controller...")
            self.controller = KeyboardController()
        
        # Register callbacks
        self.controller.register_shutdown_callback(self.emergency_stop)
        self.controller.register_tensor_changed_callback(self.on_tensor_changed)
        self.controller.register_log_state_callback(self.get_robot_state)
        
        # Control settings
        self.control_rate = 30.0  # 30Hz
        self.control_interval = 1.0 / self.control_rate
        
        # Robot state tracking
        self.running = True
        self.origin_position = None
        self.current_position = None
        self.maintaining_position = False
        
        # Start the control loop in a separate thread
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        # Start controller interface in its own thread
        self.controller_thread = None
    
    def _initialize_position(self):
        """Get the initial robot position and set it as origin"""
        try:
            # Get initial observation
            observation = self.sdk.robot.capture_observation()["observation.state"]
            self.origin_position = observation.clone()
            self.current_position = observation.clone()
            print(f"Initial position set: {self.origin_position}")
            return True
        except Exception as e:
            print(f"Error initializing position: {e}")
            return False
    
    def _control_loop(self):
        """Main control loop that runs at the specified rate"""
        # Initialize position reference
        if not self._initialize_position():
            print("Failed to initialize position, stopping controller")
            self.running = False
            return
        
        while self.running:
            try:
                # Get start time for rate control
                loop_start_time = time.time()
                
                # Get the current control tensor
                current_tensor = self.controller.get_control_tensor()
                
                try:
                    # Update position with control tensor
                    if not torch.all(current_tensor == 0):
                        self.current_position = self.current_position + current_tensor
                        # Clip joint positions to limits:
                        min_limits = torch.tensor([-110, -20, -20, -110, -110, -10], device=self.current_position.device)
                        max_limits = torch.tensor([110, 200, 200, 110, 110, 110], device=self.current_position.device)
                        #self.current_position = torch.max(torch.min(self.current_position, max_limits), min_limits)
                        self.maintaining_position = False
                    
                    # Send the current target position to the robot
                    self.sdk.send_joint_positions(self.current_position)
                    
                    # Only print status message once when transitioning to idle
                    if torch.all(current_tensor == 0):
                        if not self.maintaining_position:
                            print("Maintaining current position")
                            self.maintaining_position = True
                    else:
                        print(f"New position: {self.current_position}")
                        
                except Exception as e:
                    print(f"Error updating robot position: {e}")
                
                # Maintain control rate
                elapsed_time = time.time() - loop_start_time
                sleep_time = max(0.0, self.control_interval - elapsed_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    print(f"Warning: Control loop running slower than {self.control_rate}Hz")
                    
            except Exception as e:
                print(f"Error in control loop: {e}")
                time.sleep(0.1)  # Still try to maintain timing even with errors
    
    def on_tensor_changed(self, tensor):
        """Callback function when the control tensor changes"""
        # This will be called by the controller when the tensor changes
        # We don't need to do anything here as the control loop handles the update
        pass
    
    def emergency_stop(self):
        """Emergency stop function triggered by controllers"""
        print("EMERGENCY STOP ACTIVATED")
        
        # Set running to false to stop the control loop
        self.running = False
        
        try:
            # Check if the robot is still connected before trying to control it
            if hasattr(self.sdk, 'robot') and hasattr(self.sdk.robot, 'is_connected') and self.sdk.robot.is_connected:
                # Try to stop the robot immediately
                try:
                    # Call stop_robot to ensure proper shutdown
                    self.sdk.stop_robot()
                    print("Robot stopped successfully")
                except Exception as e:
                    print(f"Error stopping robot: {e}")
            else:
                print("Robot already disconnected or not properly initialized")
        except Exception as e:
            print(f"Error during emergency stop: {e}")
        
        print("Exiting program due to emergency stop")
        import os
        os._exit(0)

    def get_robot_state(self):
        """Get the current state of the robot"""
        try:
            # Capture the current observation from the robot
            observation = self.sdk.robot.capture_observation()["observation.state"]
            
            print(f"Current robot state: {observation}")
            return observation
        except Exception as e:
            print(f"Error getting robot state: {e}")
            return None
    
    def reset_to_origin(self):
        """Reset the robot position to the original starting position"""
        if self.origin_position is not None:
            try:
                # Reset our tracked position to the origin
                self.current_position = self.origin_position.clone()
                # Send the reset command to the robot
                self.sdk.send_joint_positions(self.current_position)
                print("Reset to origin position")
            except Exception as e:
                print(f"Error resetting to origin: {e}")
    
    def _update_simulation(self):
        # This function is scheduled in pyglet's event loop
        if self.simulation_mode and self.current_position is not None:
            try:
                self.simulator.set_joint_states(RobotSimulation.real_to_sim(self.current_position).tolist())
                self.simulator.step_simulation()
                # Print TCP pose in simulation mode
                self.latest_tcp_pose = self.simulator.get_tcp_pose()
                print(f"TCP Pose in simulation: {self.latest_tcp_pose}")
            except Exception as e:
                print(f"Error in simulation update: {e}")

    def start(self):
        print(f"Starting Combined Robot Controller (Control rate: {self.control_rate}Hz)")
        print(f"Control mode: {self.control_mode}")
        print("Press Ctrl+C to stop")
        
        try:
            if self.simulation_mode:
                print("Initializing simulation on main thread...")
                self.simulator.init_simulation()
                import pyglet.clock
                # Schedule simulation update via pyglet; this runs in the main thread
                pyglet.clock.schedule_interval(lambda dt: self._update_simulation(), self.control_interval)
            
            # Start control thread (non-blocking)
            self.control_thread.start()
            
            # Run controller interface on main thread (pyglet.app.run is called inside)
            self.controller.run()  # This will block in the main thread
            
        except KeyboardInterrupt:
            print("\nStopping controller due to keyboard interrupt")
            self.stop()
        except Exception as e:
            print(f"Error starting controller: {e}")
            self.stop()
    
    def stop(self):
        """Stop the robot controller and release resources"""
        print("Stopping Robot Controller")
        self.running = False
        
        try:
            # Reset to origin position if possible
            if self.origin_position is not None:
                self.sdk.send_joint_positions(self.origin_position)
                print("Reset to origin position before stopping")
            
            # Stop the robot explicitly
            self.sdk.stop_robot()
            
            # If simulation mode is enabled, stop the simulation thread and disconnect simulation
            if self.simulation_mode:
                import pybullet as p
                p.disconnect()
        except Exception as e:
            print(f"Error during shutdown: {e}")
        
        # Wait for control thread to finish
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        if self.controller_thread and self.controller_thread.is_alive():
            self.controller_thread.join(timeout=1.0)


def main():
    """Main entry point for the combined controller"""
    parser = argparse.ArgumentParser(description='SO100 Robot Controller')
    parser.add_argument('--mode', type=str, default='joystick', 
                        choices=['joystick', 'keyboard'],
                        help='Control mode: joystick or keyboard (default: joystick)')
    parser.add_argument('--simulate', action='store_true',
                        help='Enable simulation mode instead of using real robot')
    args = parser.parse_args()
    
    controller = CombinedController(control_mode=args.mode, simulate=args.simulate)
    
    try:
        controller.start()
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
