import sys
import torch
import time
import threading
import argparse
from so100_robot_control.teleop_devices.joystick_listener import JoystickController
from so100_robot_control.teleop_devices.keyboard_listener import KeyboardController
from so100_robot_control.robot_interface import SO100RobotInterface

class CombinedController:
    """
    Wrapper class that allows choosing between joystick or keyboard control modes.
    Sets up the robot SDK interface and passes control to the selected controller.
    """
    def __init__(self, control_mode='joystick'):
        # Initialize the SDK interface
        print("Initializing robot SDK interface...")
        self.sdk = SO100RobotInterface()
        
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
                        # Add control tensor to current position
                        self.current_position = self.current_position + current_tensor
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
    
    def start(self):
        """Start the robot controller with the selected input method"""
        print(f"Starting Combined Robot Controller (Control rate: {self.control_rate}Hz)")
        print(f"Control mode: {self.control_mode}")
        print("Press Ctrl+C to stop")
        
        try:
            # Start the control thread
            self.control_thread.start()
            
            # Start the appropriate controller (this runs in the main thread)
            print(f"Starting {self.control_mode} controller interface")
            self.controller.run()  # This blocks until exited
                
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
        except Exception as e:
            print(f"Error during shutdown: {e}")
        
        # Wait for control thread to finish
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)


def main():
    """Main entry point for the combined controller"""
    parser = argparse.ArgumentParser(description='SO100 Robot Controller')
    parser.add_argument('--mode', type=str, default='joystick', 
                        choices=['joystick', 'keyboard'],
                        help='Control mode: joystick or keyboard (default: joystick)')
    args = parser.parse_args()
    
    controller = CombinedController(control_mode=args.mode)
    
    try:
        controller.start()
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
