import torch
from abc import ABC, abstractmethod


class BaseTeleopDevice(ABC):
    """
    Abstract base class for teleoperation input devices (keyboard, joystick).

    Subclasses implement the device-specific event loop in ``run()`` and
    translate raw hardware events into a 6-D control tensor that the
    TeleopSession control loop reads every tick.
    """

    def __init__(self):
        self.mode = "lower"
        self.current_element = 1
        self.coefficient = 1.0
        self.deadzone = 0.05

        # 6-D tensor: each element maps to one robot joint
        self.control_tensor = torch.zeros(6)

        self.debug_mode = True

        # Callbacks registered by TeleopSession
        self.tensor_changed_callback = None
        self.shutdown_callback = None
        self.log_state_callback = None
        # Called once per frame on the main thread — used to tick the simulation
        # visualisation so all PyBullet calls stay on the main thread (required
        # on macOS where the OpenGL context is thread-affine).
        self.tick_callback = None

    def _print_tensor_state(self):
        tensor_values = [f"{val:.1f}" for val in self.control_tensor]
        print(f"Mode: {self.mode}, Active Element: {self.current_element}")
        print(f"Control tensor: [{', '.join(tensor_values)}]")
        if self.debug_mode:
            print(f"Raw tensor: {self.control_tensor}")

    def get_control_tensor(self):
        """Return the current control tensor."""
        return self.control_tensor

    def register_tensor_changed_callback(self, callback):
        self.tensor_changed_callback = callback

    def register_shutdown_callback(self, callback):
        self.shutdown_callback = callback

    def register_log_state_callback(self, callback):
        self.log_state_callback = callback

    def register_tick_callback(self, callback):
        """Register a callback invoked once per frame on the main thread."""
        self.tick_callback = callback

    def _change_control_element(self, element_idx):
        if 0 <= element_idx <= 5:
            self.current_element = element_idx
            print(f"Now controlling element {element_idx}")
            self._print_tensor_state()

    def _toggle_mode(self):
        self.mode = "upper" if self.mode == "lower" else "lower"
        print(f"Switched to {self.mode} control mode")
        self._print_tensor_state()

    def _trigger_shutdown(self):
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
        if self.log_state_callback:
            try:
                self.log_state_callback()
            except Exception as e:
                print(f"Error in log state callback: {e}")

    @abstractmethod
    def _apply_axis_change(self, axis_name, value):
        raise NotImplementedError

    @abstractmethod
    def _apply_button_press(self, button):
        raise NotImplementedError

    @abstractmethod
    def run(self):
        """Start the device event loop (blocks until stopped)."""
        pass
