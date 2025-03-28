import pygame
class JoystickListener:
    def __init__(self):
        # Initialize joystick
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Joystick initialized: {self.joystick.get_name()}")
        else:
            print("No joystick detected!")


    def get_input(self):
        # Get joystick input
        pygame.event.pump()
        if self.joystick:
            axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
            buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
            return {"axes": axes, "buttons": buttons}
        return {"axes": [], "buttons": []}

    def get_frame(self):
        # Get webcam frame
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to capture frame from webcam!")
        return frame

    def close(self):
        # Close joystick and camera
        pygame.joystick.quit()
        pygame.quit()

if __name__ == "__main__":
    listener = JoystickListener()
    try:
        while True:
            inputs = listener.get_input()
            # print(inputs)  # Replace with robot control logic

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        listener.close()
