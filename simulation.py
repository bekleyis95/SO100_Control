import sys
from so100_robot_control.simulation.robot_simulation import RobotSimulation

def main():
    # Set the URDF file path (adjust as necessary)
    urdf_path = "/Users/denizbekleyisseven/workspace/SO100_Control/SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf"
    sim = RobotSimulation(urdf_path)
    
    print("Available simulation modes: predefined, real, user")
    mode = input("Enter simulation mode: ").strip().lower()
    if mode not in ["predefined", "real", "user", "draw"]:
        print("Invalid mode. Defaulting to 'predefined'.")
        mode = "predefined"
    
    # Optionally, set external real angles for real mode testing
    if mode == "real":
        sim.external_real_angles = [30, 45, 60, 30, 45, 60]
    
    sim.run(simulation_mode=mode)

if __name__ == "__main__":
    main()
