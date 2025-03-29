import sys
import os
import pygame
import time
import threading
import torch
from so100_robot_control.base_controller import BaseController

class KeyboardController(BaseController):
    def __init__(self):
        # Initialize base class
        super().__init__()
        
        # Initialize pygame for keyboard input
        pygame.init()
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("Keyboard Robot Controller")
        self.font = pygame.font.Font(None, 36)
        
        # Set key repeat behavior (delay, interval in milliseconds)
        pygame.key.set_repeat(200, 50)
        
        # Running flag
        self.running = True
        
        # For continuous key press handling
        self.keys_pressed = set()
        
        # Current control values
        self.x_value = 0.0
        self.y_value = 0.0
        
        # Key to function mapping
        self.key_map = {
            pygame.K_1: lambda: self._change_control_element(1),
            pygame.K_2: lambda: self._change_control_element(2),
            pygame.K_3: lambda: self._change_control_element(3),
            pygame.K_4: lambda: self._change_control_element(4),
            pygame.K_5: lambda: self._change_control_element(5),
            pygame.K_m: self._toggle_mode,
            pygame.K_ESCAPE: self._trigger_shutdown,
        }
        
        # Print instructions
        print("Keyboard Controller Instructions:")
        print("  Arrow Keys: Control X/Y axis movement")
        print("  1-5: Select control element")
        print("  M: Toggle control mode")
        print("  ESC: Emergency stop")
        print("  Q/CTRL+C: Exit program")
    
    def _handle_key_events(self):
        """Handle keyboard events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return
                
            elif event.type == pygame.KEYDOWN:
                # Check for exit with Q
                if event.key == pygame.K_q:
                    self.running = False
                    return
                    
                # Track pressed keys for continuous movement
                self.keys_pressed.add(event.key)
                
                # Check for mapped function keys
                if event.key in self.key_map:
                    self.key_map[event.key]()
            
            elif event.type == pygame.KEYUP:
                # Remove from pressed keys
                if event.key in self.keys_pressed:
                    self.keys_pressed.remove(event.key)
    
    def _process_pressed_keys(self):
        """Process currently pressed keys for continuous movement"""
        # Reset control values
        self.x_value = 0.0
        self.y_value = 0.0
        
        # Check arrow keys for movement
        if pygame.K_UP in self.keys_pressed:
            self.y_value = 1.0
        if pygame.K_DOWN in self.keys_pressed:
            self.y_value = -1.0
        if pygame.K_RIGHT in self.keys_pressed:
            self.x_value = 1.0
        if pygame.K_LEFT in self.keys_pressed:
            self.x_value = -1.0
        
        # Apply axis values if they've changed
        if self.x_value != 0:
            self._apply_axis_value("x", self.x_value)
        if self.y_value != 0:
            self._apply_axis_value("y", self.y_value)
        
        # If both x and y are zero but keys were just released, reset control tensor
        if self.x_value == 0 and self.y_value == 0 and self.control_tensor.abs().sum() > 0:
            self.control_tensor = torch.zeros(6)
            if self.tensor_changed_callback:
                self.tensor_changed_callback(self.control_tensor)
    
    def _draw_interface(self):
        """Draw the control interface on the pygame window"""
        # Clear screen
        self.screen.fill((50, 50, 50))
        
        # Draw status text
        mode_text = self.font.render(f"Mode: {self.mode}", True, (255, 255, 255))
        element_text = self.font.render(f"Active Element: {self.current_element}", True, (255, 255, 255))
        
        # Draw tensor values
        tensor_values = [f"{val:.1f}" for val in self.control_tensor]
        tensor_text = self.font.render(f"Control: [{', '.join(tensor_values)}]", True, (255, 255, 255))
        
        # Draw axis values
        axis_text = self.font.render(f"X: {self.x_value:.1f}, Y: {self.y_value:.1f}", True, (255, 255, 255))
        
        # Instructions
        instr1 = self.font.render("Arrow Keys: Move, 1-5: Select Element", True, (200, 200, 200))
        instr2 = self.font.render("M: Switch Mode, ESC: Emergency Stop, Q: Quit", True, (200, 200, 200))
        
        # Position text
        self.screen.blit(mode_text, (20, 20))
        self.screen.blit(element_text, (20, 60))
        self.screen.blit(tensor_text, (20, 100))
        self.screen.blit(axis_text, (20, 140))
        self.screen.blit(instr1, (20, 400))
        self.screen.blit(instr2, (20, 440))
        
        # Update display
        pygame.display.flip()
    
    def run(self):
        """Run the keyboard controller main loop"""
        try:
            # Control loop
            while self.running:
                # Handle events
                self._handle_key_events()
                
                # Process pressed keys for movement
                self._process_pressed_keys()
                
                # Update the display
                self._draw_interface()
                
                # Sleep to limit update rate
                time.sleep(1/30)  # 30 FPS
                
        except KeyboardInterrupt:
            print("Keyboard controller stopped by user")
        finally:
            # Clean up
            pygame.quit()
            
            # Reset control tensor
            self.control_tensor = torch.zeros(6)
            if self.tensor_changed_callback:
                self.tensor_changed_callback(self.control_tensor)


if __name__ == "__main__":
    controller = KeyboardController()
    controller.run()
