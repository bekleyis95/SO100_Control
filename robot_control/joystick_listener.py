import pyglet
import torch

class JoystickController:
    def __init__(self):
        # Initialize joystick
        joysticks = pyglet.input.get_joysticks()
        if not joysticks:
            print("No joystick detected!")
            self.joystick = None
            return
        
        self.joystick = joysticks[0]
        self.joystick.open()
        print(f"Using joystick: {self.joystick}")
        
        # Control mode (lower or upper)
        self.mode = "lower"  # Start with lower control mode
        
        # Store the current active element (defaults to 1)
        self.current_element = 1
        
        # Control coefficient (magnitude of change)
        self.coefficient = 15.0
        
        # Initialize the 6-dimensional tensor with zeros
        self.control_tensor = torch.zeros(6)
        
        # Debug variables
        self.debug_mode = True
        
        # Add a callback that will be called whenever the tensor changes
        self.tensor_changed_callback = None
        
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
        
        # Store previous tensor for change detection
        old_tensor = self.control_tensor.clone()
        
        # First check mode, then handle axis
        if self.mode == "lower":
            if axis == "y":  # In lower mode, Y axis controls current element (swapped from X)
                if value >= 0.5:
                    self.control_tensor[self.current_element] = self.coefficient
                    self._print_tensor_state()
                elif value <= -0.5:
                    self.control_tensor[self.current_element] = -self.coefficient
                    self._print_tensor_state()
                elif -0.1 <= value <= 0.1:
                    self.control_tensor[self.current_element] = 0.0
                    self._print_tensor_state()
            
            elif axis == "x":  # In lower mode, X axis controls element 0 (swapped from Y)
                if value >= 0.5:
                    self.control_tensor[0] = self.coefficient
                    self._print_tensor_state()
                elif value <= -0.5:
                    self.control_tensor[0] = -self.coefficient
                    self._print_tensor_state()
                elif -0.1 <= value <= 0.1:
                    self.control_tensor[0] = 0.0
                    self._print_tensor_state()
            
            else:
                print(f"Additional axis {axis} detected with value {value} in lower mode")
                
        elif self.mode == "upper":
            if axis == "x":  # X-axis controls element 4 in upper mode
                if value >= 0.5:
                    self.control_tensor[4] = self.coefficient
                    self._print_tensor_state()
                elif value <= -0.5:
                    self.control_tensor[4] = -self.coefficient
                    self._print_tensor_state()
                elif -0.1 <= value <= 0.1:
                    self.control_tensor[4] = 0.0
                    self._print_tensor_state()
            
            elif axis == "y":  # Y-axis controls element 5 in upper mode
                if value >= 0.5:
                    self.control_tensor[5] = self.coefficient
                    self._print_tensor_state()
                elif value <= -0.5:
                    self.control_tensor[5] = -self.coefficient
                    self._print_tensor_state()
                elif -0.1 <= value <= 0.1:
                    self.control_tensor[5] = 0.0
                    self._print_tensor_state()
            
            else:
                print(f"Additional axis {axis} detected with value {value} in upper mode")
        
        else:
            print(f"Unknown mode: {self.mode}")
        
        # Check if the tensor has changed, notify callback if registered
        if self.tensor_changed_callback and not torch.all(old_tensor == self.control_tensor):
            self.tensor_changed_callback(self.control_tensor)
    
    def _handle_button_press(self, button):
        """Handle joystick button presses"""
        print(f"Button {button} pressed")
        
        # Button presses change which element is controlled
        if button == 0:
            self.current_element = 2
            print(f"Now controlling element 2")
        elif button == 2:
            self.current_element = 1
            print(f"Now controlling element 1")
        elif button == 4:
            self.current_element = 3
            print(f"Now controlling element 3")
        
        # Button 11 switches modes
        elif button == 11:
            self.mode = "upper" if self.mode == "lower" else "lower"
            print(f"Switched to {self.mode} control mode")
        
        # Print the current state after button press
        self._print_tensor_state()
    
    def _print_tensor_state(self):
        """Print the current state of the control tensor in a clear format"""
        tensor_values = [f"{val:.1f}" for val in self.control_tensor]
        print(f"Mode: {self.mode}, Active Element: {self.current_element}")
        print(f"Control tensor: [{', '.join(tensor_values)}]")
        
        if self.debug_mode:
            # Print the actual tensor object for detailed inspection
            print(f"Raw tensor: {self.control_tensor}")
    
    def get_control_tensor(self):
        """Return the current control tensor"""
        return self.control_tensor
    
    def register_tensor_changed_callback(self, callback):
        """
        Register a callback function that will be called whenever the tensor changes.
        """
        self.tensor_changed_callback = callback
    
    def run(self):
        """Start the pyglet event loop"""
        # Print initial state
        print("Controller initialized. Move joystick to test controls.")
        print("Press buttons 0/2/4 to change active element, button 11 to switch modes")
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
