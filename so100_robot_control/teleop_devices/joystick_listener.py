import pyglet
import torch
import sys
import os
from so100_robot_control.base_controller import BaseController

class JoystickController(BaseController):
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
        
        # Convert axis to our standard format and apply
        if axis == "z":
            self._apply_axis_value("x", value)  # Z-axis maps to X in our system
        elif axis == "y":
            self._apply_axis_value("y", value)  # Y-axis maps directly
        else:
            print(f"Additional axis {axis} detected with value {value}")
    
    def _handle_button_press(self, button):
        """Handle joystick button presses"""
        print(f"Button {button} pressed")
        
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
        if button == 0:
            self._change_control_element(2)
        elif button == 2:
            self._change_control_element(1)
        elif button == 4:
            self._change_control_element(3)
        
        # Button 11 switches modes
        elif button == 11:
            self._toggle_mode()
    
    def run(self):
        """Start the pyglet event loop"""
        # Print initial state
        print("Controller initialized. Move joystick to test controls.")
        print("Press buttons 0/2/4 to change active element, button 11 to switch modes")
        print("Press button 10 for emergency stop")
        print("Press Ctrl+C to exit")
        self._print_tensor_state()
        
        try:
            pyglet.app.run()
        except KeyboardInterrupt:
            print("\nJoystick controller stopped by user")
            # Reset control tensor to zeros before exiting
            self.control_tensor = torch.zeros(6)
            if self.tensor_changed_callback:
                self.tensor_changed_callback(self.control_tensor)
            return


if __name__ == "__main__":
    controller = JoystickController()
    if controller.joystick:
        controller.run()
