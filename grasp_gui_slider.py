import pybullet as p
import pybullet_data
import argparse
import time
import numpy as np

def calculate_ik(robot_id, end_effector_index, box_pos, orientation):   
    """Calculate inverse kinematics solution for the robot arm."""
    ik_solution = p.calculateInverseKinematics(
        robot_id,
        end_effector_index,
        box_pos,
        targetOrientation=orientation,
        maxNumIterations=100,
        residualThreshold=0.001
    )
    return ik_solution

def apply_ik_solution(robot_id, ik_solution, joint_idxs):
    """Apply the IK solution to the robot joints."""
    joint_values = []
    for index, joint_idx in enumerate(joint_idxs):
        joint_pos = ik_solution[index]
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_pos
        )
        joint_values.append(f"{p.getJointInfo(robot_id, joint_idx)[1].decode('utf-8')}: {joint_pos:.4f} rad ({joint_pos*57.2958:.2f}°)")
    return joint_values

def main():
    parser = argparse.ArgumentParser(description='URDF Visualizer with Joint Sliders')
    parser.add_argument('--urdf', type=str, default='nico_grasp.urdf', help='Path to URDF file (default: nico_grasp.urdf)')
    args = parser.parse_args()

    # Initialize PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Fixed orientation for IK (quaternion)
    #orientation = p.getQuaternionFromEuler([-3.14, 0, 0])  # Fixed downward orientation

    # Create ground plane (90x60x3 cm)
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.30, .45, 0.025],
                                                           rgbaColor=[0.0, 0.6, 0.6, 1]),
                      baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[.30, .45, 0.025]),
                      baseMass=0, basePosition=[0.26, 0, 0.029])
    # Create tablet mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.165, .267, 0.001],
                                                           rgbaColor=[0, 0, 0.0, .5]),
                      baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                                    halfExtents=[.165, .267, 0.001]), baseMass=0,
                      basePosition=[0.395, 0, 0.054])

    # Load URDF
    try:
        robot_id = p.loadURDF(args.urdf, useFixedBase=True)
    except:
        print(f"Error: Failed to load URDF file '{args.urdf}'")
        return

    # Find end effector link
    num_joints = p.getNumJoints(robot_id)
    end_effector_index = -1
    for joint_idx in range(num_joints):
        joint_info = p.getJointInfo(robot_id, joint_idx)
        link_name = joint_info[12].decode("utf-8")
        if link_name == 'endeffector':
            end_effector_index = joint_idx
            break
    if end_effector_index == -1:
        print("Error: Could not find end effector link in URDF")
        return

    # Get joint information
    joint_idxs = []
    for joint_idx in range(num_joints):
        joint_info = p.getJointInfo(robot_id, joint_idx)
        joint_name = joint_info[1].decode("utf-8")
        joint_type = joint_info[2]
        
        if joint_type != p.JOINT_FIXED:
            lower = joint_info[8]
            upper = joint_info[9]
            
            if lower >= upper:
                lower, upper = -180, 180
                
            joint_idxs.append(joint_idx)

    # Box control variables
    box_size = 0.03
    box_id = p.createMultiBody(
        baseMass=0, # Set mass to 0 if it's only visual
        baseCollisionShapeIndex=-1, # No collision shape
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[box_size/2]*3, rgbaColor=[1, 0.0, 0.0, 0.8]), # Visual shape only
        basePosition=[0.35, 0, 0.25]
    )

    # Second box control variables
    box2_size = 0.03
    box2_initial_pos = [0.0, -0.3, 0.5]
    box2_id = p.createMultiBody(
        baseMass=0, # Visual only
        baseCollisionShapeIndex=-1, # No collision
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[box2_size/2]*3, rgbaColor=[0, 0, 1, 0.7]), # Black color
        basePosition=box2_initial_pos
    )


    # Create sliders for box position control
    x_slider = p.addUserDebugParameter("Goal X", 0.25, 0.5, 0.3)
    y_slider = p.addUserDebugParameter("Goal Y", -0.25, 0.25, -0.2)
    z_slider = p.addUserDebugParameter("Goal Z", 0.07, 0.7, 0.07)
    roll_slider = p.addUserDebugParameter("Goal_Roll", -np.pi, np.pi, 0)
    pitch_slider = p.addUserDebugParameter("Goal_Pitch", -np.pi, np.pi, 0)
    yaw_slider = p.addUserDebugParameter("Goal_Yaw", -np.pi, np.pi, 0) # Default to downward

    # Create sliders for second box position control
    x2_slider = p.addUserDebugParameter("Init X", -0.2, 0.2, box2_initial_pos[0])
    y2_slider = p.addUserDebugParameter("Init Y", -0.6, 0.2, box2_initial_pos[1])
    z2_slider = p.addUserDebugParameter("Init Z", 0.2, 0.7, box2_initial_pos[2])

    # Create additional init sliders for Yaw, Pitch, Roll
    init_roll_slider = p.addUserDebugParameter("Init Roll", -np.pi, np.pi, 0)
    init_pitch_slider = p.addUserDebugParameter("Init Pitch", -np.pi, np.pi, -np.pi/2)
    init_yaw_slider = p.addUserDebugParameter("Init Yaw", -np.pi, np.pi, 0)


    # Main simulation loop
    p.setRealTimeSimulation(1)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.95,
        cameraYaw=90,
        cameraPitch=-15,
        cameraTargetPosition=[0, 0, 0.3]
    )
 
    orientation_line_id = None # Initialize desired orientation line ID tracker
    actual_orientation_line_id = None # Initialize actual orientation line ID tracker
    orientation_diff_text_id = None # Initialize debug text ID tracker
    position_diff_text_id = None # Initialize position difference text ID tracker
    
    use_second_target = False # Flag to switch between target boxes/orientations
    
    try:
        while True:
            # Check keyboard events first to potentially switch targets
            keys = p.getKeyboardEvents()
            if ord('x') in keys and keys[ord('x')] & p.KEY_WAS_TRIGGERED:
                use_second_target = not use_second_target
                print(f"Switched target source: {'Initial state' if use_second_target else 'Goal State'}")

            # --- Determine Target Position and Orientation based on flag ---
            if use_second_target:
                # Use second box sliders for position
                target_pos = [
                    p.readUserDebugParameter(x2_slider),
                    p.readUserDebugParameter(y2_slider),
                    p.readUserDebugParameter(z2_slider)
                ]
                # Use initial sliders for orientation
                roll = p.readUserDebugParameter(init_roll_slider)
                pitch = p.readUserDebugParameter(init_pitch_slider)
                yaw = p.readUserDebugParameter(init_yaw_slider)
                target_orientation_quat = p.getQuaternionFromEuler([roll, pitch, yaw])
                target_orientation_euler = [roll, pitch, yaw]
                # Update the visual position of the second box (black)
                p.resetBasePositionAndOrientation(box2_id, target_pos, [0,0,0,1])
                # Keep the first box (red) at its slider position for reference
                box1_pos_ref = [ p.readUserDebugParameter(x_slider), p.readUserDebugParameter(y_slider), p.readUserDebugParameter(z_slider)]
                p.resetBasePositionAndOrientation(box_id, box1_pos_ref, [0,0,0,1])

            else:
                # Use first box sliders for position
                target_pos = [
                    p.readUserDebugParameter(x_slider),
                    p.readUserDebugParameter(y_slider), 
                    p.readUserDebugParameter(z_slider)
                ]
                # Use main orientation sliders
                roll = p.readUserDebugParameter(roll_slider)
                pitch = p.readUserDebugParameter(pitch_slider)
                yaw = p.readUserDebugParameter(yaw_slider)
                target_orientation_quat = p.getQuaternionFromEuler([roll, pitch, yaw])
                target_orientation_euler = [roll, pitch, yaw]
                # Update the visual position of the first box (red)
                p.resetBasePositionAndOrientation(box_id, target_pos, [0,0,0,1])
                # Keep the second box (black) at its slider position for reference
                box2_pos_ref = [ p.readUserDebugParameter(x2_slider), p.readUserDebugParameter(y2_slider), p.readUserDebugParameter(z2_slider)]
                p.resetBasePositionAndOrientation(box2_id, box2_pos_ref, [0,0,0,1])


            # Apply IK first using the determined target
            ik_solution = calculate_ik(robot_id, end_effector_index, target_pos, target_orientation_quat)
            apply_ik_solution(robot_id, ik_solution, joint_idxs)

            # Get current state of the end effector AFTER applying IK
            link_state = p.getLinkState(robot_id, end_effector_index)
            ee_pos = link_state[0] # World position
            actual_orientation_quat = link_state[1] # Actual world orientation (quaternion)
            actual_orientation_euler = p.getEulerFromQuaternion(actual_orientation_quat)

            line_length = 0.05 # Length of visualization lines

            # --- Visualize Desired Orientation (Red Line) ---
            # Remove previous red line
            if orientation_line_id is not None:
                p.removeUserDebugItem(orientation_line_id)

            # Calculate end point for the desired orientation line (originating from current ee_pos)
            rot_matrix_desired = p.getMatrixFromQuaternion(target_orientation_quat) # Use active target orientation
            z_axis_direction_desired = [rot_matrix_desired[2], rot_matrix_desired[5], rot_matrix_desired[8]]
            # Invert the z-axis direction to point upside down
            line_end_desired = [ee_pos[0] - z_axis_direction_desired[0] * line_length,
                                ee_pos[1] - z_axis_direction_desired[1] * line_length,
                                ee_pos[2] - z_axis_direction_desired[2] * line_length]

            # Draw the new red line (Desired Orientation)
            orientation_line_id = p.addUserDebugLine(ee_pos, line_end_desired, [1, 0, 0], 5) # Red line

            # --- Calculate and Display Orientation Difference ---
            # Quaternion difference
            quat_diff = p.getDifferenceQuaternion(target_orientation_quat, actual_orientation_quat)
            # Convert quaternion difference to Euler angles (axis-angle representation essentially)
            axis, angle = p.getAxisAngleFromQuaternion(quat_diff)
            
            # Euler angle difference (direct subtraction, might wrap around pi)
            euler_diff = [d - a for d, a in zip(target_orientation_euler, actual_orientation_euler)]
            # Normalize Euler differences to [-pi, pi] for better readability (optional)
            euler_diff_norm = [(diff + np.pi) % (2 * np.pi) - np.pi for diff in euler_diff]


            # Remove previous orientation debug text
            if orientation_diff_text_id is not None:
                p.removeUserDebugItem(orientation_diff_text_id)

            # Format orientation difference text
            orientation_diff_text = (
                f"  Quat Diff Angle: {angle:.3f} rad\n"
            )
            
            # Add new orientation debug text
            if angle> 0.1:
                color=[1,0,0]
            else:
                color = [0, 0, 1]
            
            orientation_diff_text_id = p.addUserDebugText(
                orientation_diff_text, 
                textPosition=[0.05, 0.0, 0.8], # Position the text in the GUI
                textColorRGB=color, 
                textSize=1.0
            )

            # --- Calculate and Display Position Difference ---
            # Calculate Euclidean distance using the active target position
            pos_diff_vec = np.array(target_pos) - np.array(ee_pos)
            pos_distance = np.linalg.norm(pos_diff_vec)

            # Remove previous position debug text
            if position_diff_text_id is not None:
                p.removeUserDebugItem(position_diff_text_id)

            # Format position difference text
            position_diff_text = f"Position Distance: {pos_distance:.4f} m"

            if pos_distance> 0.1:
                color2 = [1,0,0]
            else:
                color2 = [0, 0, 1]

            # Add new position debug text
            position_diff_text_id = p.addUserDebugText(
                position_diff_text,
                textPosition=[0.05, 0, 0.7], # Position above orientation text
                textColorRGB=color2, 
                textSize=1.0
            )
            
            # Check keyboard events (IK already applied above)
            # keys = p.getKeyboardEvents() # Already got keys at the start of the loop
            if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                # Re-apply IK if needed (though it's already applied each loop with current target)
                ik_solution = calculate_ik(robot_id, end_effector_index, target_pos, target_orientation_quat)
                apply_ik_solution(robot_id, ik_solution, joint_idxs)

            time.sleep(0.01)
    except KeyboardInterrupt:
        p.disconnect()

if __name__ == "__main__":
    main()