import torch
import time
import threading
from robot_control.joystick_listener import JoystickController
from robot_control.sdk_interface import SDKInterface

class RobotController:
    """
    Integrates joystick control with the robot SDK interface.
    Converts joystick movements to robot joint commands.
    """
    def __init__(self):
        # Initialize the SDK interface
        self.sdk = SDKInterface()
        
        # Initialize the joystick controller
        self.joystick = JoystickController()
        
        # Save the last control tensor for comparison
        self.last_tensor = torch.zeros(6)
        
        # Flag for running the control loop
        self.running = True
        
        # Current joint positions from the robot
        self.current_joint_positions = None
        
        # Start the control loop in a separate thread
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
    
    def _control_loop(self):
        """
        Main control loop that checks for tensor changes and updates robot joints.
        """
        while self.running:
            try:
                # Get the current control tensor from joystick
                current_tensor = self.joystick.get_control_tensor()
                
                # Check if the tensor has changed and is not all zeros
                if not torch.all(current_tensor == 0) and not torch.all(current_tensor == self.last_tensor):
                    # Get current joint positions from robot (through SDK)
                    try:
                        observation = self.sdk.robot.capture_observation()["observation.state"]
                        
                        # Send updated positions to the robot
                        self.sdk.send_joint_positions(observation + current_tensor)

                    except Exception as e:
                        print(f"Error updating robot position: {e}")
                
                # Update the last tensor for next comparison
                self.last_tensor = current_tensor.clone()
                
                # Sleep to avoid maxing out CPU
                time.sleep(0.05)
            except Exception as e:
                print(f"Error in control loop: {e}")
                time.sleep(0.5)  # Add delay to prevent tight error loops
    
    def _apply_tensor_to_joints(self, tensor):
        """
        Apply control tensor values to current joint positions.
        Scale tensor values appropriately for joint movement.
        """
        if self.current_joint_positions is None:
            print("No current joint positions available")
            return None
        
        # Convert tensor to a movement delta (scale as needed)
        # This scaling factor needs tuning for your specific robot
        scaling_factor = 0.01
        delta = tensor * scaling_factor
        
        # Add delta to current joint positions
        # Make sure tensor dimensions match joint positions
        new_positions = []
        for i, joint_pos in enumerate(self.current_joint_positions):
            if i < len(delta):
                new_positions.append(joint_pos + delta[i].item())
            else:
                new_positions.append(joint_pos)
        
        return torch.tensor(new_positions)
    
    def start(self):
        """
        Start the robot controller, including the control loop and joystick monitoring.
        """
        print("Starting Robot Controller")
        print("Press Ctrl+C to stop")
        
        try:
            # Start the control thread
            self.control_thread.start()
            
            # Start the joystick controller (this runs in the main thread)
            if self.joystick.joystick:
                print("Joystick found, starting controller")
                self.joystick.run()  # This blocks until exited
            else:
                print("No joystick found, cannot start controller")
                self.stop()  # Clean up if no joystick
        except KeyboardInterrupt:
            print("\nStopping robot controller due to keyboard interrupt")
            self.stop()
        except Exception as e:
            print(f"Error starting robot controller: {e}")
            self.stop()
    
    def stop(self):
        """
        Stop the robot controller and release resources.
        """
        print("Stopping Robot Controller")
        self.running = False
        
        try:
            # Stop the robot movement - send zero tensor to ensure robot stops
            zero_tensor = torch.zeros(6)
            observation = self.sdk.robot.capture_observation()["observation.state"]
            self.sdk.send_joint_positions(observation)  # Send current position to stop movement
            
            # Stop the robot explicitly
            self.sdk.stop_robot()
        except Exception as e:
            print(f"Error during shutdown: {e}")
        
        # Wait for control thread to finish
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)


if __name__ == "__main__":
    controller = RobotController()
    try:
        controller.start()
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        controller.stop()
