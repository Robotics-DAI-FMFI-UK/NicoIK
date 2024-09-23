import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, round, any, mean 
import argparse
import os
import calibration_matrices


def nicodeg2rad(nicojoints, nicodegrees):
    if isinstance(nicojoints, str):
        nicojoints = [nicojoints]
    if isinstance(nicodegrees, (int, float)):
        nicodegrees = [nicodegrees]

    rads = []

    for nicojoint, nicodegree in zip(nicojoints, nicodegrees):
        if nicojoint == 'r_wrist_z':
            rad = deg2rad(nicodegree/2)
        elif nicojoint == 'r_wrist_x':
            rad = deg2rad(nicodegree/4)
        else:
            rad = deg2rad(nicodegree)
        rads.append(rad)

    if len(rads) == 1:
        return rads[0]
    return rads


def rad2nicodeg(nicojoints, rads):
    if isinstance(nicojoints, str):
        nicojoints = [nicojoints]
    if isinstance(rads, (int, float)):
        rads = [rads]

    nicodegrees = []

    for nicojoint, rad in zip(nicojoints, rads):
        if nicojoint == 'r_wrist_z':
            nicodegree = rad2deg(rad) * 2
        elif nicojoint == 'r_wrist_x':
            nicodegree = rad2deg(rad) * 4
        else:
            nicodegree = rad2deg(rad)
        nicodegrees.append(nicodegree)

    if len(nicodegrees) == 1:
        return nicodegrees[0]
    return nicodegrees

set_printoptions(precision=3)
set_printoptions(suppress=True)

def get_joints_limits(robot_id, num_joints, arg_dict):
    """
    Identify limits, ranges and rest poses of individual robot joints. Uses data from robot model.

    Returns:
        :return [joints_limits_l, joints_limits_u]: (list) Lower and upper limits of all joints
        :return joints_ranges: (list) Ranges of movement of all joints
        :return joints_rest_poses: (list) Rest poses of all joints
    """
    joints_limits_l, joints_limits_u, joints_ranges, joints_rest_poses, joint_names, link_names, joint_indices = [], [], [], [], [], [], []
    for jid in range(num_joints):
        joint_info = p.getJointInfo(robot_id, jid)
        q_index = joint_info[3]
        joint_name = joint_info[1]
        link_name = joint_info[12]
        if q_index > -1:  # Fixed joints have q_index -1
            joint_names.append(joint_info[1].decode("utf-8"))
            link_names.append(joint_info[12].decode("utf-8"))
            joint_indices.append(joint_info[0])
            joints_limits_l.append(joint_info[8])
            joints_limits_u.append(joint_info[9])
            joints_ranges.append(joint_info[9] - joint_info[8])
            joints_rest_poses.append((joint_info[9] + joint_info[8]) / 2)
        if arg_dict["left"]:
            if link_name.decode("utf-8") == 'endeffectol':
                end_effector_index = jid
        else:
            if link_name.decode("utf-8") == 'endeffector':
                # if link_name.decode("utf-8") == 'endeffector':
                end_effector_index = jid

    return [joints_limits_l,
            joints_limits_u], joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices


def get_real_joints(robot, joints):
    last_position = []

    for k in joints:
        actual = robot.getAngle(k)
        # print("{} : {}, ".format(k,actual),end="")
        last_position.append(actual)
    # print("")

    return last_position


def match_joints(init_pos, joint_names):
    actuated_joint_names = []
    actuated_joint_init_pos = []
    for name in init_pos.keys():
        if name in joint_names:
            actuated_joint_names.append((name))
            actuated_joint_init_pos.append(init_pos[name])

    return actuated_joint_names, actuated_joint_init_pos


def reset_robot(robot, init_pos):
    for k in init_pos.keys():
        robot.setAngle(k, init_pos[k], RESET_SPEED)

    return robot


def reset_actuated(robot, actuated_joints, actuated_initpos):
    for joint, initpos in zip(actuated_joints, actuated_initpos):
        robot.setAngle(joint, initpos, RESET_SPEED)
    return robot


def speed_control(initial, target, duration):
    speed_to_reach = (abs((float(initial) - float(target)) / float(1260 * duration)))
    return speed_to_reach


def sim_speed_control(initial, target, duration):
    return abs(float(initial) - float(target)) / float(0.425 * duration)


def spin_simulation(steps):
    for i in range(steps):
        p.stepSimulation()
        time.sleep(0.01)




def to_formatted_str(number):
    number = str(number)

    decimal_point_index = number.find('.')
    if decimal_point_index >= 0:
        number += '0' * (DECIMALS - (len(number) - decimal_point_index - 1))

    return number


def main():
    parser = argparse.ArgumentParser() 
    parser.add_argument("-p", "--position", nargs=3, type=float, help="Target position for the robot end effector as a list of three floats.")
    parser.add_argument("-rr", "--real_robot", action="store_true", help="If set, execute action on real robot.")
    arg_dict = vars(parser.parse_args())

    # TouchAgent()
    # TouchAgent.clean()

    # GUI initialization
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

    robot_id = p.loadURDF("./nico_upper_rh6d.urdf", [0, 0, 0])
    # Create table mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.3, .45, 0.02],
                                                               rgbaColor=[0.6, 0.6, 0.6, 1]),
                      baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[.3, .45, 0.02]),
                      baseMass=0, basePosition=[0.27, 0, -0.005])
    # Create tablet mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.16, .26, 0.01],
                                                               rgbaColor=[0, 0, 0.0, 1]),
                      baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                                     halfExtents=[.16, .26, 0.01]), baseMass=0,
                      basePosition=[0.41, 0, 0.008])

    num_joints = p.getNumJoints(robot_id)
    joints_limits, joints_ranges, joints_rest_poses, end_effector_index, joint_names, link_names, joint_indices = get_joints_limits(
        robot_id, num_joints, arg_dict)
    # Custom intital position

    # joints_rest_poses = deg2rad([-15, 68, 2.8, 56.4, 0.0, 11.0, -70.0])

    actuated_joints, actuated_initpos = match_joints(init_pos, joint_names)

    # Real robot initialization and setting all joints
    if arg_dict["real_robot"]:
        from nicomotion.Motion import Motion
        motorConfig = './nico_humanoid_upper_rh7d_ukba.json'
        try:
            robot = Motion(motorConfig=motorConfig)
            print('Robot initialized')
        except:
            print('Motors are not operational')
            exit()
        # robot = init_robot()

    # IK paramenters
    max_iterations = 300
    residual_threshold = 0.0001

    target_position = arg_dict["position"]

        # Create goal dot
    p.createMultiBody(
        baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.007, rgbaColor=[0, 0, 1, .5]),
        baseCollisionShapeIndex=-1, baseMass=0, basePosition=target_position)


            


            
    
 
    ik_solution = p.calculateInverseKinematics(robot_id,
                                                end_effector_index,
                                                target_position,
                                                maxNumIterations=max_iterations,
                                            residualThreshold=residual_threshold)





for j in range(len(joint_indices)):
    p.resetJointState(robot_id, joint_indices[j], ik_solution[j])

            
     
            
if arg_dict["real_robot"]:

    targetdeg = []
    joint_values = get_real_joints(robot, actuated_joints)
    tic = time.time()

    print('movement_duration: {}'.format(movement_duration))

    for j, realjoint in enumerate(actuated_joints):
        
        
        speed = speed_control(actual_position[j], nicodeg_ik[j], movement_duration)
        
        #CORRECT SPEED FOR WRIST JOINTS
        if realjoint == 'r_wrist_z' or realjoint == 'r_wrist_x':
            speed = speed/10

        #print('Joint: {} , Angle: {}Speed: {}'.format(realjoint, nicodeg_ik[j],speed))
        #while True:
        #    angle = float(input("Enter angle: "))
        #    speed = float(input("Enter speed: "))
        #    robot.setAngle('r_elbow_y', angle, speed)
        robot.setAngle(realjoint, nicodeg_ik[j], speed)
        targetdeg.append(nicodeg_ik[j])
    nicodeg_ik = []
            
    p.disconnect()


if __name__ == "__main__":
    main()
