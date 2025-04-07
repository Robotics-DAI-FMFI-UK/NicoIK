import pybullet as p
import pybullet_data # Added for GUI assets
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, round, any, mean
import argparse
import os
import csv
# Attempt to import Motion, but handle failure gracefully if not simulating
try:
    from nicomotion.Motion import Motion
    NICOMOTION_AVAILABLE = True
except ImportError:
    Motion = None # Define Motion as None if import fails
    NICOMOTION_AVAILABLE = False
    print("Warning: nicomotion library not found. Hardware control disabled.")

class Grasper:
    # Constants
    SPEED = 0.03
    SPEEDF = 0.09
    DELAY = 2
    REPEAT = 1

    # Predefined poses (consider making these configurable or loading from file)
    RESET_POSE = [0,0,0,90,90,90,0,0,-180,-180,-180,-180,0.22,12.88,11.03,100.97,-24.13,-91.91,-180.0,-180.0,-180.0,-174.81]
    DROP_POSE = [0,0,-20,27,40,90,125,100,180,20,20,20,0,13,11,100,-24,-91,-180.0,-180.0,-180.0,-175]
    GRASP_POSE = [0,0,-2,34,17,122,161,-13,180,-180,-180,-180,0,13,11,100,-24,-91,-180.0,-180.0,-180.0,-175]
    INIT_POS = {  # standard position
        'head_z': 0.0, 'head_y': 0.0, 'r_shoulder_z': 1, 'r_shoulder_y': 87,
        'r_arm_x': 88, 'r_elbow_y': 87, 'r_wrist_z': 2, 'r_wrist_x': -29,
        'r_thumb_z': -1, 'r_thumb_x': 44, 'r_indexfinger_x': -90, 'r_middlefingers_x': 100.0,
        'l_shoulder_z': -24.0, 'l_shoulder_y': 13.0, 'l_arm_x': 0.0, 'l_elbow_y': 104.0,
        'l_wrist_z': -4.0, 'l_wrist_x': -55.0, 'l_thumb_z': -62.0, 'l_thumb_x': -180.0,
        'l_indexfinger_x': -170.0, 'l_middlefingers_x': -180.0
    }

    def __init__(self, urdf_path="./nico_upper_rh6d_r.urdf", motor_config='./nico_humanoid_upper_rh7d_ukba.json', connect_pybullet=True, connect_robot=True, use_gui=False):
        """
        Initializes the Grasper class.

        Args:
            urdf_path (str): Path to the robot URDF file.
            motor_config (str): Path to the motor configuration JSON file for nicomotion.
            connect_pybullet (bool): Whether to connect to PyBullet simulation.
            connect_robot (bool): Whether to attempt connection to the physical robot.
            use_gui (bool): If connecting to PyBullet, whether to use GUI or DIRECT mode.
        """
        set_printoptions(precision=3, suppress=True)

        self.robot_id = None
        self.num_joints = 0
        self.joints_limits_l = []
        self.joints_limits_u = []
        self.joints_ranges = []
        self.joints_rest_poses = []
        self.joint_names = []
        self.link_names = []
        self.joint_indices = [] # List of movable joint indices
        self.joint_name_to_index = {} # Map from joint name to index
        self.end_effector_index = -1
        self.robot = None # For nicomotion hardware interface
        self.is_pybullet_connected = False
        self.is_robot_connected = False

        if connect_pybullet:
            try:
                # Connect to PyBullet (GUI or DIRECT)
                connection_mode = p.GUI if use_gui else p.DIRECT
                self.physics_client = p.connect(connection_mode)
                self.is_pybullet_connected = True
                print(f"Connected to PyBullet ({'GUI' if use_gui else 'DIRECT'} mode).")

                if use_gui:
                    p.setAdditionalSearchPath(pybullet_data.getDataPath())
                    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
                    p.setGravity(0, 0, -9.81)
                    # Load ground plane only in GUI mode for visualization
                    p.loadURDF("plane.urdf")
                # Load robot slightly above ground in GUI mode
                start_pos = [0, 0, 0.5] if use_gui else [0, 0, 0]
                self.robot_id = p.loadURDF(urdf_path, start_pos, useFixedBase=use_gui) # Fix base in GUI
                self.num_joints = p.getNumJoints(self.robot_id)
                self._get_joints_limits()
                print(f"Loaded URDF: {urdf_path}")
                print(f"Found {self.num_joints} joints.")
                print(f"End effector index: {self.end_effector_index}")
            except Exception as e:
                print(f"Error connecting to PyBullet or loading URDF: {e}")
                self.is_pybullet_connected = False


        if connect_robot and NICOMOTION_AVAILABLE and Motion is not None:
            try:
                self.robot = Motion(motorConfig=motor_config)
                self.is_robot_connected = True
                print(f"Robot hardware initialized using config: {motor_config}")
            except Exception as e:
                print(f"Could not initialize robot hardware: {e}")
                print("Proceeding without hardware connection.")
                self.is_robot_connected = False
        elif connect_robot and not NICOMOTION_AVAILABLE:
             print("Hardware connection requested, but nicomotion library is not available.")
             self.is_robot_connected = False
        else:
             # Not attempting to connect robot
             self.is_robot_connected = False

    def _get_joints_limits(self):
        """
        Internal method to retrieve joint limits and info from PyBullet model.
        """
        if not self.is_pybullet_connected:
            print("PyBullet not connected. Cannot get joint limits.")
            return

        joints_limits_l, joints_limits_u, joints_ranges, joints_rest_poses = [], [], [], []
        joint_names, link_names, joint_indices, joint_name_to_index = [], [], [], {}
        end_effector_index = -1

        for jid in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, jid)
            q_index = joint_info[3]
            joint_name_bytes = joint_info[1]
            link_name_bytes = joint_info[12]

            if q_index > -1:  # Fixed joints have q_index -1
                # Decode names once
                decoded_joint_name = joint_name_bytes.decode("utf-8")
                decoded_link_name = link_name_bytes.decode("utf-8")
                joint_id = joint_info[0]

                # Append the decoded name *once*
                joint_names.append(decoded_joint_name)
                link_names.append(decoded_link_name)
                joint_indices.append(joint_id)
                joint_name_to_index[decoded_joint_name] = joint_id # Populate map

                joints_limits_l.append(joint_info[8])
                joints_limits_u.append(joint_info[9])
                joints_ranges.append(joint_info[9] - joint_info[8])
                joints_rest_poses.append((joint_info[9] + joint_info[8]) / 2)

            if link_name_bytes.decode("utf-8") == 'endeffector': # Make sure your URDF defines this link name
                end_effector_index = jid

        self.joints_limits_l = joints_limits_l
        self.joints_limits_u = joints_limits_u
        self.joints_ranges = joints_ranges
        self.joints_rest_poses = joints_rest_poses
        self.joint_names = joint_names
        self.link_names = link_names
        self.joint_indices = joint_indices # List of movable joint indices
        self.joint_name_to_index = joint_name_to_index # Map name -> index
        self.end_effector_index = end_effector_index


    def calculate_ik(self, pos, ori_euler):
        """
        Calculates Inverse Kinematics for a given end-effector pose.
        Assumes the URDF defines the correct kinematic chain leading to end_effector_index.

        Args:
            pos (list/tuple): Target position [x, y, z].
            ori_euler (list/tuple): Target orientation in Euler angles [roll, pitch, yaw].

        Returns:
            list: The calculated joint angles (in radians) for the movable joints
                  as returned by PyBullet, or None if calculation fails. The length
                  should match the number of movable joints (DoFs).
        """
        if not self.is_pybullet_connected:
            print("PyBullet not connected. Cannot calculate IK.")
            return None
        if self.end_effector_index < 0:
            print("End effector index not found. Cannot calculate IK.")
            return None
        # Ensure internal lists are populated and consistent
        if not all([self.joints_limits_l, self.joints_limits_u, self.joints_ranges, self.joints_rest_poses, self.joint_indices]):
             print("Joint information not fully available. Cannot calculate IK.")
             return None

        try:
            # calculateInverseKinematics returns a list whose length is the number of
            # movable joints (DoFs) identified by PyBullet for the robot.
            ik_solution = p.calculateInverseKinematics(self.robot_id,
                                                     self.end_effector_index,
                                                     pos,
                                                     targetOrientation=p.getQuaternionFromEuler(ori_euler),
                                                     lowerLimits=self.joints_limits_l,
                                                     upperLimits=self.joints_limits_u,
                                                     jointRanges=self.joints_ranges,
                                                     restPoses=self.joints_rest_poses,
                                                     maxNumIterations=300,
                                                     residualThreshold=0.0001)

            # Basic check if solution is valid
            if ik_solution is None:
                 print("IK calculation returned None.")
                 return None

            # Verify the length matches the number of movable joints found earlier
            # This is the crucial check based on the actual DoFs from the URDF
            if len(ik_solution) != len(self.joint_indices):
                 print(f"Error: IK solution length ({len(ik_solution)}) doesn't match number of movable joints ({len(self.joint_indices)}).")
                 return None

            return list(ik_solution) # Ensure it's a list

        except Exception as e:
            import traceback
            print(f"IK calculation failed with exception:")
            traceback.print_exc()
            return None

    @staticmethod
    def nicodeg2rad(nicojoints, nicodegrees):
        """Converts Nico-specific degrees to radians."""
        if isinstance(nicojoints, str):
            nicojoints = [nicojoints]
        if isinstance(nicodegrees, (int, float)):
            nicodegrees = [nicodegrees]

        rads = []
        for nicojoint, nicodegree in zip(nicojoints, nicodegrees):
            if nicojoint == 'r_wrist_z' or nicojoint == 'l_wrist_z':
                rad = deg2rad(nicodegree / 2)
            elif nicojoint == 'r_wrist_x' or nicojoint == 'l_wrist_x':
                rad = deg2rad(nicodegree / 4)
            else:
                rad = deg2rad(nicodegree)
            rads.append(rad)

        return rads[0] if len(rads) == 1 else rads

    @staticmethod
    def rad2nicodeg(nicojoints, rads):
        """Converts radians to Nico-specific degrees."""
        if isinstance(nicojoints, str):
            nicojoints = [nicojoints]
        if isinstance(rads, (int, float)):
            rads = [rads]

        nicodegrees = []
        for nicojoint, rad in zip(nicojoints, rads):
            # Ensure rad is a number before applying rad2deg
            if isinstance(rad, (int, float)):
                if nicojoint == 'r_wrist_z':
                    nicodegree = rad2deg(rad) * 2
                elif nicojoint == 'r_wrist_x':
                    nicodegree = rad2deg(rad) * 4
                else:
                    nicodegree = rad2deg(rad)
                nicodegrees.append(nicodegree)
            else:
                 # Handle cases where IK might return non-numeric values or None
                 print(f"Warning: Non-numeric radian value '{rad}' for joint '{nicojoint}'. Skipping conversion.")
                 nicodegrees.append(None) # Or some other placeholder

        # Filter out None values if you only want valid degrees
        valid_nicodegrees = [deg for deg in nicodegrees if deg is not None]

        if len(nicojoints) == 1: # Check based on input length
             return nicodegrees[0] if nicodegrees else None
        return nicodegrees # Return the list potentially containing None


    def _set_pose_hardware(self, pose_values):
        """Internal: Sends a pose command to the physical robot."""
        if not self.is_robot_connected:
            print("Robot hardware not connected. Skipping hardware command.")
            return False

        index = 0
        # Ensure pose_values has the correct number of elements corresponding to INIT_POS keys
        keys = list(self.INIT_POS.keys())
        if len(pose_values) != len(keys):
             print(f"Error: Pose values length ({len(pose_values)}) does not match INIT_POS keys length ({len(keys)}).")
             return False

        try:
            for k in keys:
                # Make sure the joint name k exists in the robot's configuration
                # This check might depend on the specifics of the nicomotion library
                # if hasattr(self.robot, 'setAngle'): # Basic check
                self.robot.setAngle(k, float(pose_values[index]), self.SPEED)
                # else:
                #    print(f"Warning: Joint '{k}' not found or setAngle not available.")
                index += 1
            return True
        except Exception as e:
            print(f"Error setting hardware pose: {e}")
            return False

    def _set_hand_angles_hardware(self, thumb_z, thumb_x, index_x, middle_x):
         """Internal: Sets individual finger angles on the hardware."""
         if not self.is_robot_connected:
             print("Robot hardware not connected. Skipping hand command.")
             return False
         try:
             # Assuming right hand joints based on original script
             self.robot.setAngle('r_thumb_z', thumb_z, self.SPEEDF)
             self.robot.setAngle('r_thumb_x', thumb_x, self.SPEEDF)
             self.robot.setAngle('r_indexfinger_x', index_x, self.SPEEDF)
             self.robot.setAngle('r_middlefingers_x', middle_x, self.SPEEDF)
             return True
         except Exception as e:
             print(f"Error setting hand angles: {e}")
             return False


    def close_hand(self):
        """Closes the robot's hand (hardware)."""
        print("Closing hand...")
        if not self.is_robot_connected:
            print("Robot hardware not connected. Cannot close hand.")
            return
        # Original logic used a loop, simplified here for clarity
        # Final closed position angles might need adjustment
        success = self._set_hand_angles_hardware(thumb_z=180, thumb_x=20, index_x=20, middle_x=20)
        if success:
            print("Hand closed.")
        else:
            print("Failed to close hand.")
        time.sleep(0.5) # Short delay after command


    def open_hand(self):
        """Opens the robot's hand (hardware)."""
        print("Opening hand...")
        if not self.is_robot_connected:
            print("Robot hardware not connected. Cannot open hand.")
            return
        # Final open position angles might need adjustment
        success = self._set_hand_angles_hardware(thumb_z=180, thumb_x=-180, index_x=-180, middle_x=-180)
        if success:
            print("Hand opened.")
        else:
            print("Failed to open hand.")
        time.sleep(0.5) # Short delay after command

    def move_to_pose(self, pose_name):
        """Moves the robot to a predefined pose (e.g., 'reset', 'grasp', 'drop')."""
        if not self.is_robot_connected:
             print("Robot hardware not connected. Cannot move to pose.")
             return

        pose_map = {
            'reset': self.RESET_POSE,
            'grasp': self.GRASP_POSE,
            'drop': self.DROP_POSE
        }
        if pose_name.lower() in pose_map:
            print(f"Moving to {pose_name} pose...")
            success = self._set_pose_hardware(pose_map[pose_name.lower()])
            if success:
                print(f"Moved to {pose_name} pose.")
            else:
                print(f"Failed to move to {pose_name} pose.")
            time.sleep(self.DELAY)
        else:
            print(f"Error: Unknown pose name '{pose_name}'. Available: {list(pose_map.keys())}")


    def perform_grasp_sequence(self):
        """Executes the predefined grasp sequence by calling individual steps."""
        if not self.is_robot_connected:
            print("Robot hardware not connected. Cannot perform grasp sequence.")
            return

        print("\n--- Starting Grasp Sequence ---")
        self.perform_move()
        self.perform_grasp()
        # Optional: Lift arm after grasp - moved to perform_grasp
        self.perform_drop()
        print("--- Grasp Sequence Finished ---\n")

    def perform_move(self):
        """Moves the robot to the grasp pose."""
        print("Moving to grasp pose...")
        self.move_to_pose('grasp')
        print("Moved to grasp pose.")
        time.sleep(self.DELAY)

    def perform_grasp(self):
        """Closes the robot's hand and optionally lifts the arm."""
        print("Closing hand...")
        self.close_hand()
        print("Hand closed.")
        # Optional: Lift arm after grasp
        print("Lifting arm slightly...")
        try:
            self.robot.setAngle('r_elbow_y', 90, self.SPEED) # Example joint adjustment
            print("Arm lifted.")
            time.sleep(self.DELAY)
        except Exception as e:
             print(f"Could not lift arm: {e}")

    def perform_drop(self):
        """Moves the robot to the drop pose and opens the hand."""
        print("Moving to drop pose...")
        self.move_to_pose('drop')
        print("Moved to drop pose.")
        self.open_hand()
        self.move_to_pose('reset')
        time.sleep(self.DELAY)

    def get_real_joint_angles(self):
        """Reads current joint angles from the hardware."""
        if not self.is_robot_connected:
            print("Robot hardware not connected. Cannot read joint angles.")
            return None

        last_position = {}
        try:
            # Assuming getAngle works for reading too, or use a specific read method if available
            for k in self.INIT_POS.keys():
                 # Check if the robot object has the getAngle method and the joint exists
                 # This check depends heavily on the nicomotion library's API
                 if hasattr(self.robot, 'getAngle'): # Basic check
                     actual = self.robot.getAngle(k)
                     last_position[k] = actual
                 # else:
                 #    print(f"Warning: Joint '{k}' not found or getAngle not available.")
            print("Current hardware joint angles:", last_position)
            return last_position
        except Exception as e:
            print(f"Error reading hardware joint angles: {e}")
            return None

    def set_pose_sim(self, target_angles_rad):
        """Sets the robot's pose in the PyBullet simulation using radian values."""
        if not self.is_pybullet_connected:
            print("PyBullet not connected. Cannot set simulation pose.")
            return
        # Check if the number of target angles matches the number of movable joints
        if len(self.joint_indices) != len(target_angles_rad):
             print(f"Error setting sim pose: Mismatch between movable joint indices ({len(self.joint_indices)}) and target angles ({len(target_angles_rad)})")
             return

        print("Setting robot pose in simulation...")
        # Iterate through the movable joint indices and set their state
        for i, joint_index in enumerate(self.joint_indices):
            target_angle = target_angles_rad[i]
            # Use resetJointState for immediate effect in visualization
            p.resetJointState(self.robot_id, joint_index, target_angle)
            # Optional: Print joint name if available
            # joint_name = self.joint_names[i] # Assuming self.joint_names is correctly populated and ordered
            # print(f"  Joint index {joint_index} set to {target_angle:.4f} rad")
        print("Simulation pose set.")

    def disconnect(self):
        """Disconnects from PyBullet and potentially cleans up hardware resources."""
        print("Disconnecting...")
        if self.is_pybullet_connected:
            try:
                p.disconnect(self.physics_client)
                self.is_pybullet_connected = False
                print("Disconnected from PyBullet.")
            except Exception as e:
                print(f"Error disconnecting PyBullet: {e}")

        if self.is_robot_connected and hasattr(self.robot, 'stop'):
             try:
                 # Add any necessary hardware cleanup/shutdown commands here
                 # e.g., self.robot.stop() or similar, depending on nicomotion API
                 print("Stopped robot hardware interface (if applicable).")
             except Exception as e:
                 print(f"Error stopping robot hardware: {e}")
        print("Cleanup complete.")


# Main execution block
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Nico Robot Grasping Control")
    parser.add_argument(
        "--urdf",
        type=str,
        default="./nico_upper_rh6d_r.urdf",
        help="Path to the robot URDF file.",
    )
    parser.add_argument(
        "--config",
        type=str,
        default="./nico_humanoid_upper_rh7d_ukba.json",
        help="Path to the motor config JSON.",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Run only PyBullet simulation, don't connect to hardware.",
    )
    parser.add_argument(
        "--test-ik", action="store_true", help="Perform an IK calculation test."
    )
    parser.add_argument(
        "--pos",
        nargs=3,
        type=float,
        default=[0.3, -0.2, 0.1],
        help="Target position [x y z] for IK test.",
    )
    parser.add_argument(
        "--ori",
        nargs=3,
        type=float,
        default=[0, 0, 3.14],
        help="Target orientation [r p y] for IK test.",
    )

    args = parser.parse_args()

    connect_hw = not args.simulate
    grasper = None  # Initialize grasper to None

    # Determine if GUI is needed
    use_gui_for_pybullet = args.simulate and args.test_ik

    try:
        print("Initializing Grasper...")
        grasper = Grasper(
            urdf_path=args.urdf,
            motor_config=args.config,
            connect_pybullet=True,  # Always connect pybullet if simulating or testing IK
            connect_robot=connect_hw,
            use_gui=use_gui_for_pybullet,
        )

        ik_calculated_pose = None  # Store IK result if successful

        if args.test_ik:
            if grasper.is_pybullet_connected:
                print(f"\n--- Testing IK Calculation ---")
                print(f"Target Position: {args.pos}")
                print(f"Target Orientation (Euler): {args.ori}")
                ik_solution_rad = grasper.calculate_ik(args.pos, args.ori)
                if ik_solution_rad:
                    print(f"IK Solution (radians): {ik_solution_rad}")
                    ik_calculated_pose = ik_solution_rad  # Store for potential visualization
                    # Ensure joint_names are populated before converting
                    if grasper.joint_names:
                        ik_solution_nico_deg = grasper.rad2nicodeg(
                            grasper.joint_names, ik_solution_rad
                        )
                        print(f"IK Solution (Nico degrees): {ik_solution_nico_deg}")
                    else:
                        print(
                            "Could not convert to Nico degrees: Joint names not available."
                        )
                else:
                    print("IK calculation failed.")
                print("--- IK Test Finished ---\n")
            else:
                print("Cannot perform IK test: PyBullet not connected.")
        else:
            # Execute grasp sequence steps individually
            print("\n--- Executing Grasp Sequence Steps Individually ---")
            grasper.perform_move()
            grasper.perform_grasp()
            grasper.perform_drop()
            print("--- Grasp Sequence Steps Finished ---\n")