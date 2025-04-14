import pybullet as p
import pybullet_data
import argparse
import time
import numpy as np

def calculate_ik(robot_id, end_effector_index, box_pos, fixed_orientation):
    """Calculate inverse kinematics solution for the robot arm."""
    ik_solution = p.calculateInverseKinematics(
        robot_id,
        end_effector_index,
        box_pos,
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
    parser.add_argument("-ct", "--control", type=str, default='auto', help="Calculate IK automatically or by keypress (default: auto)")
    args = parser.parse_args()

    # Initialize PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Fixed orientation for IK (quaternion)
    fixed_orientation = p.getQuaternionFromEuler([0, 0, 3.14])  # Fixed downward orientation

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
        baseMass=0.1,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_size/2]*3),
        basePosition=[0.35, 0, 0.25]
    )

    # Create sliders for box position control
    x_slider = p.addUserDebugParameter("Box X", 0.25, 0.5, 0.35)
    y_slider = p.addUserDebugParameter("Box Y", -0.25, 0.25, 0)
    z_slider = p.addUserDebugParameter("Box Z", 0.07, 0.4, 0.07)

    # Main simulation loop
    p.setRealTimeSimulation(1)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.95,
        cameraYaw=90,
        cameraPitch=-15,
        cameraTargetPosition=[0, 0, 0.3]
    )

    try:
        while True:
            # Update box position from sliders
            box_pos = [
                p.readUserDebugParameter(x_slider),
                p.readUserDebugParameter(y_slider), 
                p.readUserDebugParameter(z_slider)
            ]
            p.resetBasePositionAndOrientation(box_id, box_pos, [0,0,0,1])
            
            # Check keyboard events
            if args.control == 'auto':
                ik_solution = calculate_ik(robot_id, end_effector_index, box_pos, fixed_orientation)
                apply_ik_solution(robot_id, ik_solution, joint_idxs)
            else:
                keys = p.getKeyboardEvents()
                if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                    ik_solution = calculate_ik(robot_id, end_effector_index, box_pos, fixed_orientation)
                    apply_ik_solution(robot_id, ik_solution, joint_idxs)

            time.sleep(0.01)
    except KeyboardInterrupt:
        p.disconnect()

if __name__ == "__main__":
    main()