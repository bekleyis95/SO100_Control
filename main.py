import argparse
from so100_robot_control.combined_controller import CombinedController

def main():
    """
    Main entry point for the SO100 Control application.
    Parses command-line arguments and starts the appropriate controller.
    """
    parser = argparse.ArgumentParser(description='SO100 Robot Controller')
    parser.add_argument('--mode', type=str, default='joystick', 
                        choices=['joystick', 'keyboard'],
                        help='Control mode: joystick or keyboard (default: joystick)')
    parser.add_argument('--simulate', action='store_true')
    args = parser.parse_args()
    
    controller = CombinedController(control_mode=args.mode, simulate=True if args.simulate else False)
    
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
