import pybullet as p
import pybullet_data
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, round, any, mean
import argparse
import os
import csv

# Constants for simulation control
SIM_TIME_STEP = 1./240.
SIM_DELAY_STEPS = 900 # Number of simulation steps for delays (approx 0.5 sec)
SIM_HAND_STEPS = 48 # Number of simulation steps for hand movements (approx 0.2 sec)

# Predefined poses in Nico Degrees
# Note: These lists must match the order of joints defined in INIT_POS_KEYS below
RESET_POSE_DEG = [0,0,0,90,90,90,0,0,-180,-180,-180,-180,0.22,12.88,11.03,100.97,-24.13,-91.91,-180.0,-180.0,-180.0,-174.81]
DROP_POSE_DEG = [0,0,-20,27,40,90,125,100,180,20,20,20,0,13,11,100,-24,-91,-180.0,-180.0,-180.0,-175]
GRASP_POSE_DEG = [0,0,-2,34,17,122,161,-13,180,-180,-180,-180,0,13,11,100,-24,-91,-180.0,-180.0,-180.0,-175]

# Define the order of joints corresponding to the pose lists above
# This should match the joints controlled by the original script's poses
INIT_POS_KEYS = [
    'head_z', 'head_y',
    'r_shoulder_z', 'r_shoulder_y', 'r_arm_x', 'r_elbow_y', 'r_wrist_z', 'r_wrist_x',
    'r_thumb_z', 'r_thumb_x', 'r_indexfinger_x', 'r_middlefingers_x',
    'l_shoulder_z', 'l_shoulder_y', 'l_arm_x', 'l_elbow_y', 'l_wrist_z', 'l_wrist_x',
    'l_thumb_z', 'l_thumb_x', 'l_indexfinger_x', 'l_middlefingers_x'
] # Derived from the keys order in original grasp.py init_pos dictionary
# Ensure the length matches the pose lists
assert len(INIT_POS_KEYS) == len(RESET_POSE_DEG), "Mismatch between INIT_POS_KEYS and pose list length"


# --- Utility Functions ---

def nicodeg2rad(nicojoints, nicodegrees):
    """Converts Nico-specific degrees to radians."""
    if isinstance(nicojoints, str):
        nicojoints = [nicojoints]
    if isinstance(nicodegrees, (int, float)):
        nicodegrees = [nicodegrees]

    rads = []
    for nicojoint, nicodegree in zip(nicojoints, nicodegrees):
        # Ensure nicodegree is numeric before conversion
        if isinstance(nicodegree, (int, float)):
            if nicojoint == 'r_wrist_z' or nicojoint == 'l_wrist_z':
                rad = deg2rad(nicodegree / 2)
            elif nicojoint == 'r_wrist_x' or nicojoint == 'l_wrist_x':
                rad = deg2rad(nicodegree / 4)
            else:
                rad = deg2rad(nicodegree)
            rads.append(rad)
        else:
            print(f"Warning: Non-numeric degree value '{nicodegree}' for joint '{nicojoint}'. Skipping conversion.")
            rads.append(None) # Or handle as error

    # Filter out None values if necessary, or return list with potential Nones
    valid_rads = [r for r in rads if r is not None]
    if len(nicojoints) == 1:
        return rads[0] if rads else None
    return rads # Return list potentially containing None

def get_joint_info(robot_id):
    """Gets joint names, indices, limits, etc. from the PyBullet model."""
    num_joints = p.getNumJoints(robot_id)
    joint_name_to_index = {}
    joint_indices = []
    joints_limits_l, joints_limits_u, joints_ranges, joints_rest_poses = [], [], [], []
    link_names = []
    end_effector_index = -1

    print("\nRobot Joints:")
    for jid in range(num_joints):
        joint_info = p.getJointInfo(robot_id, jid)
        jid = joint_info[0]
        joint_name = joint_info[1].decode("utf-8")
        joint_type = joint_info[2]
        q_index = joint_info[3] # Velocity index in state vector, -1 for fixed
        link_name = joint_info[12].decode("utf-8")

        print(f"  ID: {jid}, Name: {joint_name}, Type: {joint_type}, Link: {link_name}, q_idx: {q_index}")

        # Store info only for movable joints (REVOLUTE, PRISMATIC, SPHERICAL, PLANAR)
        if q_index > -1:
            joint_name_to_index[joint_name] = jid
            joint_indices.append(jid)
            joints_limits_l.append(joint_info[8])
            joints_limits_u.append(joint_info[9])
            joints_ranges.append(joint_info[9] - joint_info[8])
            joints_rest_poses.append((joint_info[9] + joint_info[8]) / 2) # Midpoint as rest pose
            link_names.append(link_name)

            if link_name == 'endeffector': # Check if your URDF defines this link
                end_effector_index = jid
                print(f"    -> Found end effector link at joint index: {jid}")

    print(f"\nFound {len(joint_name_to_index)} movable joints.")
    if end_effector_index == -1:
         print("Warning: Could not find a link named 'endeffector'. IK might not work as expected.")

    return joint_name_to_index, joint_indices, [joints_limits_l, joints_limits_u], joints_ranges, joints_rest_poses, end_effector_index


# --- Simulation Control Functions ---

def set_pose_sim(robot_id, joint_name_to_index, pose_keys, pose_values_deg):
    """Sets the robot to a specific pose in the simulation using degrees."""
    print(f"Setting pose with {len(pose_values_deg)} values...")
    if len(pose_keys) != len(pose_values_deg):
        print(f"Error: Mismatch between pose_keys ({len(pose_keys)}) and pose_values_deg ({len(pose_values_deg)})")
        return

    target_positions_rad = nicodeg2rad(pose_keys, pose_values_deg)

    for joint_name, target_pos_rad in zip(pose_keys, target_positions_rad):
        if joint_name in joint_name_to_index and target_pos_rad is not None:
            joint_index = joint_name_to_index[joint_name]
            p.setJointMotorControl2(bodyUniqueId=robot_id,
                                    jointIndex=joint_index,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=target_pos_rad,
                                    force=500) # Apply some force
            # print(f"  Setting {joint_name} (idx {joint_index}) to {target_pos_rad:.3f} rad")
        # else:
            # print(f"  Skipping {joint_name}: Not found in model or invalid target value.")

def set_hand_angles_sim(robot_id, joint_name_to_index, thumb_z_deg, thumb_x_deg, index_x_deg, middle_x_deg):
    """Sets individual finger angles in the simulation using degrees."""
    hand_joints = {
        'r_thumb_z': thumb_z_deg,
        'r_thumb_x': thumb_x_deg,
        'r_indexfinger_x': index_x_deg,
        'r_middlefingers_x': middle_x_deg
    }
    # print(f"Setting hand angles (deg): {hand_joints}")

    rad_angles = nicodeg2rad(list(hand_joints.keys()), list(hand_joints.values()))

    for joint_name, target_pos_rad in zip(hand_joints.keys(), rad_angles):
         if joint_name in joint_name_to_index and target_pos_rad is not None:
             joint_index = joint_name_to_index[joint_name]
             p.setJointMotorControl2(bodyUniqueId=robot_id,
                                     jointIndex=joint_index,
                                     controlMode=p.POSITION_CONTROL,
                                     targetPosition=target_pos_rad,
                                     force=100) # Lower force for fingers maybe
             # print(f"  Setting {joint_name} (idx {joint_index}) to {target_pos_rad:.3f} rad")
         # else:
             # print(f"  Skipping {joint_name}: Not found in model or invalid target value.")


def close_hand_sim(robot_id, joint_name_to_index):
    """Simulates closing the hand."""
    print("Closing hand (sim)...")
    # Simplified: Go directly to closed state based on original script's final state
    # Original loop went up to angle=20
    set_hand_angles_sim(robot_id, joint_name_to_index,
                        thumb_z_deg=180, # From original loop
                        thumb_x_deg=20,
                        index_x_deg=20,
                        middle_x_deg=20)
    # Simulate for a short duration
    for _ in range(SIM_HAND_STEPS):
        p.stepSimulation()
        # time.sleep(SIM_TIME_STEP) # Optional small sleep for smoother viz

def open_hand_sim(robot_id, joint_name_to_index):
    """Simulates opening the hand."""
    print("Opening hand (sim)...")
    # Simplified: Go directly to open state based on original script's final state
    # Original loop went down to angle=-180
    set_hand_angles_sim(robot_id, joint_name_to_index,
                        thumb_z_deg=180, # From original loop
                        thumb_x_deg=-180,
                        index_x_deg=-180,
                        middle_x_deg=-180)
    # Simulate for a short duration
    for _ in range(SIM_HAND_STEPS):
        p.stepSimulation()
        # time.sleep(SIM_TIME_STEP) # Optional small sleep for smoother viz

def wait_sim(steps):
    """Runs the simulation for a number of steps."""
    # print(f"Simulating for {steps} steps...")
    for _ in range(steps):
        p.stepSimulation()
        # time.sleep(SIM_TIME_STEP) # Optional small sleep for smoother viz


# --- Main Execution ---

def main():
    parser = argparse.ArgumentParser(description="Nico Robot Grasping Simulation")
    parser.add_argument("--urdf", type=str, default="./nico_upper_rh6d_r.urdf", help="Path to the robot URDF file.")
    args = parser.parse_args()

    print("Starting PyBullet Grasping Simulation...")
    # Connect to PyBullet with GUI
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # For plane.urdf etc.
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0) # We will step manually
    p.setTimeStep(SIM_TIME_STEP)

    # Load ground plane
    planeId = p.loadURDF("plane.urdf")

    # Load robot
    print(f"Loading URDF: {args.urdf}")
    try:
        robot_id = p.loadURDF(args.urdf, [0, 0, 0.5], useFixedBase=True) # Start slightly above ground
    except Exception as e:
        print(f"Error loading URDF: {e}")
        p.disconnect()
        return

    # Get joint information
    joint_name_to_index, joint_indices, _, _, _, _ = get_joint_info(robot_id)

    # --- Grasp Sequence ---
    print("\n--- Starting Grasp Sequence (Simulation) ---")
    input("Press Enter to start the sequence...") # Pause for user

    # 1. Move to Reset Pose
    print("\n1. Moving to Reset Pose")
    set_pose_sim(robot_id, joint_name_to_index, INIT_POS_KEYS, RESET_POSE_DEG)
    input("Press Enter to start the sequence...")

    # 2. Move to Grasp Pose
    print("\n2. Moving to Grasp Pose")
    set_pose_sim(robot_id, joint_name_to_index, INIT_POS_KEYS, GRASP_POSE_DEG)
    input("Press Enter to start the sequence...")

    # 3. Close Hand
    print("\n3. Closing Hand")
    close_hand_sim(robot_id, joint_name_to_index)
    input("Press Enter to start the sequence...") # Shorter wait after hand action

    # 4. Lift Arm (Move Elbow)
    print("\n4. Lifting Arm (Elbow)")
    # Find elbow joint index
    elbow_joint_name = 'r_elbow_y'
    if elbow_joint_name in joint_name_to_index:
        elbow_joint_index = joint_name_to_index[elbow_joint_name]
        target_elbow_rad = nicodeg2rad(elbow_joint_name, 90) # Target 90 degrees
        if target_elbow_rad is not None:
             p.setJointMotorControl2(bodyUniqueId=robot_id,
                                     jointIndex=elbow_joint_index,
                                     controlMode=p.POSITION_CONTROL,
                                     targetPosition=target_elbow_rad,
                                     force=500)
             wait_sim(SIM_DELAY_STEPS)
        else:
             print(f"Could not convert target angle for {elbow_joint_name}")
    else:
        print(f"Could not find joint '{elbow_joint_name}'")


    # 5. Move to Drop Pose
    print("\n5. Moving to Drop Pose")
    set_pose_sim(robot_id, joint_name_to_index, INIT_POS_KEYS, DROP_POSE_DEG)
    wait_sim(SIM_DELAY_STEPS)

    # 6. Open Hand
    print("\n6. Opening Hand")
    open_hand_sim(robot_id, joint_name_to_index)
    wait_sim(SIM_DELAY_STEPS // 2) # Shorter wait

    # 7. Move back to Reset Pose
    print("\n7. Moving back to Reset Pose")
    set_pose_sim(robot_id, joint_name_to_index, INIT_POS_KEYS, RESET_POSE_DEG)
    wait_sim(SIM_DELAY_STEPS * 2) # Longer wait at the end

    print("\n--- Grasp Sequence Finished ---")
    print("Simulation running. Close the PyBullet window to exit.")

    # Keep simulation running until closed
    while p.isConnected():
        # Allow interaction with GUI (camera, etc.)
        # p.stepSimulation() # Not needed if just viewing final state
        time.sleep(0.1)


    p.disconnect()
    print("PyBullet disconnected.")


if __name__ == "__main__":
    set_printoptions(precision=3, suppress=True)
    main()