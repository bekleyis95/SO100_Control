from robot_control.robot_controller import RobotController

def main():
    """
    Main entry point for the SO100 Control application.
    Initializes and runs the robot controller.
    """
    controller = RobotController()
    
    try:
        # Start the controller - this blocks until the user exits
        controller.start()
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        # Make sure we always clean up
        controller.stop()
        print("Robot controller stopped")

if __name__ == "__main__":
    main()
