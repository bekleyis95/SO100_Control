import torch
from abc import ABC, abstractmethod

class BaseController(ABC):
    """
    Abstract base class for robot controllers.
    Defines common functionality for both joystick and keyboard controllers.
    """
    def __init__(self):
        # Control mode (lower or upper)
        self.mode = "lower"  # Start with lower control mode
        
        # Store the current active element (defaults to 1)
        self.current_element = 1
        
        # Control coefficient (magnitude of change)
        self.coefficient = 1.0
        
        # Add a small deadzone to prevent drift
        self.deadzone = 0.05
        
        # Initialize the 6-dimensional tensor with zeros
        self.control_tensor = torch.zeros(6)
        
        # Debug variables
        self.debug_mode = True
        
        # Add callbacks
        self.tensor_changed_callback = None
        self.shutdown_callback = None
        self.log_state_callback = None
    
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
    
    def register_shutdown_callback(self, callback):
        """
        Register a callback function that will be called when shutdown is requested.
        """
        self.shutdown_callback = callback
    
    def register_log_state_callback(self, callback):
        """
        Register a callback function that will be called to log the current state.
        """
        self.log_state_callback = callback

    def _change_control_element(self, element_idx):
        """Change which element is being controlled"""
        if 0 <= element_idx <= 5:  # Valid range for control tensor
            self.current_element = element_idx
            print(f"Now controlling element {element_idx}")
            self._print_tensor_state()
    
    def _toggle_mode(self):
        """Toggle between upper and lower control modes"""
        self.mode = "upper" if self.mode == "lower" else "lower"
        print(f"Switched to {self.mode} control mode")
        self._print_tensor_state()
    
    def _trigger_shutdown(self):
        """Trigger emergency shutdown through callback"""
        print("EMERGENCY STOP TRIGGERED")
        if self.shutdown_callback:
            try:
                self.shutdown_callback()
            except Exception as e:
                print(f"Error in shutdown callback: {e}")
                import os
                os._exit(1)
        else:
            print("No shutdown callback registered")
            import os
            os._exit(0)
    
    def _trigger_log_state(self):
        """Log the current state of the control tensor"""
        if self.log_state_callback:
            try:
                self.log_state_callback()
            except Exception as e:
                print(f"Error in log state callback: {e}")
    
    def _apply_axis_value(self, axis_name, value):
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
    
    @abstractmethod
    def run(self):
        """Start the controller event loop - must be implemented by subclasses"""
        pass
