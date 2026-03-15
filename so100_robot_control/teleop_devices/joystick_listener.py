from abc import abstractmethod
import pyglet
import torch
import sys
import os
from so100_robot_control.teleop_devices.base_teleop_device import BaseTeleopDevice

class JoystickController(BaseTeleopDevice):
    def __init__(self):
        # Initialize base class
        super().__init__()
        
        # Initialize joystick
        joysticks = pyglet.input.get_joysticks()
        if not joysticks:
            print("No joystick detected!")
            self.joystick = None
            return
        
        self.joystick = joysticks[0]
        self.joystick.open()
        print(f"Using joystick: {self.joystick}")
        
        # Set up event handlers for joystick
        @self.joystick.event
        def on_joyaxis_motion(_, axis, value):
            self._handle_axis_motion(axis, value)
        
        @self.joystick.event
        def on_joybutton_press(_, button):
            self._handle_button_press(button)
    
    def _handle_axis_motion(self, axis, value):
        """Handle joystick axis movements"""
        print(f"Axis {axis} moved to {value}")
        self._apply_axis_change(axis, value) 
    
    def _handle_button_press(self, button):
        """Handle joystick button presses"""
        print(f"Button {button} pressed")
        self._apply_button_press(button)
    
    @abstractmethod
    def _apply_axis_change(self, axis_name, value):
        """Apply axis value to the control tensor"""
        raise NotImplementedError("This method should be implemented by subclasses")
    
    @abstractmethod
    def _apply_button_press(self, button):
        """Apply button press to the control tensor"""
        raise NotImplementedError("This method should be implemented by subclasses")
    
    def run(self):
        """Start the pyglet event loop"""
        # Print initial state
        print("Controller initialized. Move joystick to test controls.")
        print("Press buttons 0/2/4 to change active element, button 11 to switch modes")
        print("Press button 10 for emergency stop")
        print("Press Ctrl+C to exit")
        self._print_tensor_state()

        if self.tick_callback:
            pyglet.clock.schedule_interval(lambda dt: self.tick_callback(), 1 / 30)

        try:
            pyglet.app.run()
        except KeyboardInterrupt:
            print("\nJoystick controller stopped by user")
            # Reset control tensor to zeros before exiting
            self.control_tensor = torch.zeros(6)
            if self.tensor_changed_callback:
                self.tensor_changed_callback(self.control_tensor)
            return

class JointJoystickController(JoystickController):
    """
    Controller for joint control of the robot.
    This controller uses a 6D tensor to represent the robot's state.
    """
    def __init__(self):
        super().__init__()
        # Initialize control tensor with zeros
        self.control_tensor = torch.zeros(6)
    
    def _apply_axis_change(self, axis_name, value):
        """Apply axis value to the control tensor"""
        
        """
        Apply a value from an axis to the control tensor
        :param axis_name: 'x' or 'y'
        :param value: float value between -1.0 and 1.0
        """
        # Store previous tensor for change detection
        old_tensor = self.control_tensor.clone()
        
        # Apply deadzone to prevent drift when control is near center
        if abs(value) < self.deadzone:
            value = 0.0
        
        # Reset all values to ensure only one motor moves at a time
        self.control_tensor = torch.zeros(6)
        
        # First check mode, then handle axis
        if self.mode == "lower":
            if axis_name == "y":  # Y axis controls current element
                self.control_tensor[self.current_element] = self.coefficient * value
            elif axis_name == "x":  # X axis controls element 0
                self.control_tensor[0] = self.coefficient * value
        elif self.mode == "upper":
            if axis_name == "x":  # X-axis controls element 4 in upper mode
                self.control_tensor[4] = self.coefficient * value
            elif axis_name == "y":  # Y-axis controls element 5 in upper mode
                self.control_tensor[5] = self.coefficient * value
        
        # Check if the tensor has changed, notify callback if registered
        if self.tensor_changed_callback and not torch.all(old_tensor == self.control_tensor):
            self.tensor_changed_callback(self.control_tensor)

    
    def _apply_button_press(self, button):
        # Button 10 triggers system shutdown
        if button == 10:
            print("EMERGENCY STOP BUTTON PRESSED - SHUTTING DOWN")
            self._trigger_shutdown()
            return
        elif button == 5:
            self._trigger_log_state()
        # Reset control tensor whenever button is pressed to avoid lingering values
        self.control_tensor = torch.zeros(6)
        
        # Button presses change which element is controlled
        if button == 1:
            self._change_control_element(1)
        elif button == 2:
            self._change_control_element(2)
        elif button == 3:
            self._change_control_element(3)
        
        # Button 11 switches modes
        elif button == 11:
            self._toggle_mode()


class CartesianJoystickController(JoystickController):
    """
    Controller for Cartesian control of the robot.
    This controller uses a 6D tensor to represent the robot's state.
    """
    def __init__(self):
        super().__init__()
        # Initialize control tensor with zeros
        self.control_tensor = torch.zeros(6)
    
    def _apply_axis_change(self, axis_name, value):
        """Apply axis value to the control tensor"""
        
        """
        Apply a value from an axis to the control tensor
        :param axis_name: 'x' or 'y'
        :param value: float value between -1.0 and 1.0
        """
        # Store previous tensor for change detection
        old_tensor = self.control_tensor.clone()
        
        # Apply deadzone to prevent drift when control is near center
        if abs(value) < self.deadzone:
            value = 0.0
        
        # Reset all values to ensure only one motor moves at a time
        self.control_tensor = torch.zeros(6)
        
        # First check mode, then handle axis
        if self.mode == "lower":
            if axis_name == "y":  # Y axis controls current element
                self.control_tensor[self.current_element] = self.coefficient * value
            elif axis_name == "x":  # X axis controls element 0
                self.control_tensor[0] = self.coefficient * value
        elif self.mode == "upper":
            if axis_name == "x":  # X-axis controls element 4 in upper mode
                self.control_tensor[4] = self.coefficient * value
            elif axis_name == "y":  # Y-axis controls element 5 in upper mode
                self.control_tensor[5] = self.coefficient * value
        
        # Display the current state
        self._print_tensor_state()
        
        # Check if the tensor has changed, notify callback if registered
        if self.tensor_changed_callback and not torch.all(old_tensor == self.control_tensor):
            self.tensor_changed_callback(self.control_tensor)
    
    def _apply_button_press(self, button):
        # Button 10 triggers system shutdown
        if button == 10:
            print("EMERGENCY STOP BUTTON PRESSED - SHUTTING DOWN")
            self._trigger_shutdown()
            return
        elif button == 5:
            self._trigger_log_state()
        # Reset control tensor whenever button is pressed to avoid lingering values
        self.control_tensor = torch.zeros(6)
        
        # Button presses change which element is controlled
        if button == 1:
            self._change_control_element(1)
        elif button == 2:
            self._change_control_element(2)
        elif button == 3:
            self._change_control_element(3)
        
        # Button 11 switches modes
        elif button == 11:
            self._toggle_mode()