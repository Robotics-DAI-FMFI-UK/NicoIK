import argparse
import sys
from grasper import Grasper



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Nico Robot Grasping Control")
    parser.add_argument("--urdf", type=str, default="./urdf/nico_grasper.urdf", help="Path to the robot URDF file.")
    parser.add_argument("--config", type=str, default="./nico_humanoid_upper_rh7d_ukba.json", help="Path to the motor config JSON.")
    parser.add_argument("--side", type=str, default="right", help="Which arm and gripper to execute")
    parser.add_argument("--real_robot", action="store_true", help="Execute actions on the real robot (requires hardware connection).")
    parser.add_argument("--pos", nargs=3, type=float, default=[0.4, -0.3, 0.11], help="Target position [x y z] for IK test/move.")
    parser.add_argument( "--ori", nargs=3, type=float, default=[0, 0, 0], help="Target orientation [r p y] for IK test/move.")
    args = parser.parse_args()

    connect_hw = args.real_robot
    grasper = None  # Initialize grasper to None


    print("Initializing Grasper...")
    try:
        grasper = Grasper(
            urdf_path=args.urdf,
            motor_config=args.config,
            connect_robot=args.real_robot,     # Connect to the real robot hardware
        )
        print("Grasper initialized successfully for real robot.")
    except Exception as e:
        print(f"Error initializing Grasper for real robot: {e}")

    ik_solution_rad = None
    ik_solution_nico_deg = None # Store IK result in Nico degrees

    if grasper.is_pybullet_connected:
        print(f"\n--- Calculating IK ---")
        print(f"Target Position: {args.pos}")
        print(f"Target Orientation (Euler): {args.ori}")
        ik_solution_rad = grasper.calculate_ik(args.pos, args.ori)
        if ik_solution_rad:
            #print(f"IK Solution (radians): {ik_solution_rad}")
            # Ensure joint_names are populated before converting
            if grasper.joint_names:
                # Convert to Nico degrees dictionary
                ik_solution_nico_deg = grasper.rad2nicodeg(
                    grasper.joint_names, ik_solution_rad
                )
                # Filter out None values if any conversion failed
                ik_solution_nico_deg = {k: v for k, v in ik_solution_nico_deg.items() if v is not None}
                if ik_solution_nico_deg:
                    print("IK Solution (Nico degrees dict):")
                    for joint, angle in ik_solution_nico_deg.items():
                        print(f"{joint}: {angle}")
            else:
                print("Could not convert to Nico degrees: Joint names not available.")
        else:
            print("IK calculation failed.")
        print("--- IK Calculation Finished ---\n")
    else:
        print("Cannot perform IK calculation: PyBullet not connected.")

    # Execute actions on the real robot if requested and connected
    if args.real_robot:
        if grasper.is_robot_connected:
            if ik_solution_nico_deg: # Check if IK calculation was successful and converted
                print("\n--- Executing Sequence with IK Move ---")
                grasper.open_gripper(args.side)
                #Init pose
                grasper.move_arm([0, -0.3, 0.5], [0,-1.57,0], args.side)
                # Pick object
                grasper.move_arm([args.pos[0],args.pos[1],args.pos[2]+0.2], args.ori, args.side)
                grasper.move_arm(args.pos, args.ori, args.side)
                grasper.close_gripper(args.side) # Close right gripper
                #Lift                
                grasper.move_arm([args.pos[0],args.pos[1],args.pos[2]+0.2], args.ori, args.side)
                #Move to drop pose
                grasper.move_arm([0.35,-0.4,0.2], args.ori, args.side)
                # Drop object
                grasper.open_gripper(args.side)
                grasper.move_arm([0.2,-0.4,0.2], args.ori, args.side)
                grasper.move_arm([0.2,-0.4,0.11], args.ori, args.side)
                grasper.close_gripper(args.side)
                grasper.move_arm([0.2,-0.4,0.3], args.ori, args.side)
                grasper.move_arm([args.pos[0],args.pos[1],args.pos[2]+0.2], args.ori, args.side)
                grasper.move_arm(args.pos, args.ori, args.side)
                grasper.open_gripper(args.side)
                #Init pose
                grasper.move_arm([0, -0.3, 0.5], [0,-1.57,0], args.side)

                print("--- Sequence Finished ---\n")
            else:
                print("Cannot execute sequence: IK calculation failed or did not produce valid angles.")
        else:
            print("Cannot execute on real robot: Hardware not connected.")

    else:
        print("\nNo action performed (use --real_robot to execute on hardware.)")

    # Cleanup
    #if grasper:
    #    grasper.disconnect()