import sys
import os
import pygame
import time
import threading
import torch
from so100_robot_control.teleop_devices.base_teleop_device import BaseTeleopDevice


class KeyboardController(BaseTeleopDevice):
    def __init__(self):
        super().__init__()

        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("Keyboard Robot Controller")
        self.font = pygame.font.Font(None, 36)

        # Disable key-repeat: movement is polled via keys_pressed each frame,
        # so repeat is not needed for motion and would cause action-selection
        # keys (1–5, M) to fire repeatedly when held.
        pygame.key.set_repeat(0, 0)

        self.running = True
        self.keys_pressed = set()
        self.x_value = 0.0
        self.y_value = 0.0

        self.key_map = {
            pygame.K_0: lambda: self._change_control_element(0),  # shoulder pan
            pygame.K_1: lambda: self._change_control_element(1),
            pygame.K_2: lambda: self._change_control_element(2),
            pygame.K_3: lambda: self._change_control_element(3),
            pygame.K_4: lambda: self._change_control_element(4),
            pygame.K_5: lambda: self._change_control_element(5),
            pygame.K_m: self._toggle_mode,
            pygame.K_ESCAPE: self._trigger_shutdown,
        }

        print("Keyboard Controller Instructions:")
        print("  Arrow Keys: Control X/Y axis movement")
        print("  0-5: Select control element (0=shoulder pan, 1-5=others)")
        print("  M: Toggle upper/lower control mode")
        print("  ESC: Emergency stop  |  Q: Quit")

    def _apply_axis_change(self, axis_name, value):
        old_tensor = self.control_tensor.clone()

        if abs(value) < self.deadzone:
            value = 0.0

        self.control_tensor = torch.zeros(6)

        if self.mode == "lower":
            if axis_name == "y":
                self.control_tensor[self.current_element] = self.coefficient * value
            elif axis_name == "x":
                self.control_tensor[0] = self.coefficient * value
        elif self.mode == "upper":
            if axis_name == "x":
                self.control_tensor[4] = self.coefficient * value
            elif axis_name == "y":
                self.control_tensor[5] = self.coefficient * value

        if self.tensor_changed_callback and not torch.all(old_tensor == self.control_tensor):
            self.tensor_changed_callback(self.control_tensor)

    def _apply_button_press(self, button):
        # Keyboard uses key_map instead of hardware buttons — no-op here.
        pass

    def _handle_key_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.running = False
                    return

                self.keys_pressed.add(event.key)

                if event.key in self.key_map:
                    self.key_map[event.key]()

            elif event.type == pygame.KEYUP:
                if event.key in self.keys_pressed:
                    self.keys_pressed.remove(event.key)

    def _process_pressed_keys(self):
        self.x_value = 0.0
        self.y_value = 0.0

        if pygame.K_UP in self.keys_pressed:
            self.y_value = 1.0
        if pygame.K_DOWN in self.keys_pressed:
            self.y_value = -1.0
        if pygame.K_RIGHT in self.keys_pressed:
            self.x_value = 1.0
        if pygame.K_LEFT in self.keys_pressed:
            self.x_value = -1.0

        if self.x_value != 0:
            self._apply_axis_change("x", self.x_value)
        if self.y_value != 0:
            self._apply_axis_change("y", self.y_value)

        if self.x_value == 0 and self.y_value == 0 and self.control_tensor.abs().sum() > 0:
            self.control_tensor = torch.zeros(6)
            if self.tensor_changed_callback:
                self.tensor_changed_callback(self.control_tensor)

    def _draw_interface(self):
        self.screen.fill((30, 30, 30))

        mode_text = self.font.render(f"Mode: {self.mode}", True, (255, 255, 255))
        element_text = self.font.render(f"Active Element: {self.current_element}", True, (255, 255, 255))
        tensor_values = [f"{val:.2f}" for val in self.control_tensor]
        tensor_text = self.font.render(f"Control: [{', '.join(tensor_values)}]", True, (100, 220, 100))
        axis_text = self.font.render(f"X: {self.x_value:.1f}, Y: {self.y_value:.1f}", True, (200, 200, 255))
        instr1 = self.font.render("Arrows: Move | 0-5: Select | M: Mode", True, (160, 160, 160))
        instr2 = self.font.render("ESC: E-Stop | Q: Quit", True, (160, 160, 160))

        self.screen.blit(mode_text, (20, 20))
        self.screen.blit(element_text, (20, 60))
        self.screen.blit(tensor_text, (20, 100))
        self.screen.blit(axis_text, (20, 140))
        self.screen.blit(instr1, (20, 400))
        self.screen.blit(instr2, (20, 440))
        pygame.display.flip()

    def run(self):
        try:
            while self.running:
                self._handle_key_events()
                self._process_pressed_keys()
                self._draw_interface()
                if self.tick_callback:
                    self.tick_callback()
                time.sleep(1 / 30)
        except KeyboardInterrupt:
            print("Keyboard controller stopped by user")
        finally:
            pygame.quit()
            self.control_tensor = torch.zeros(6)
            if self.tensor_changed_callback:
                self.tensor_changed_callback(self.control_tensor)


if __name__ == "__main__":
    controller = KeyboardController()
    controller.run()
