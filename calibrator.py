import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, round, any, mean 
import argparse
import os
#import keyboard



init_pos = {  # standard position
    'l_shoulder_z': -24.0,
    'l_shoulder_y': 13.0,
    'l_arm_x': 0.0,
    'l_elbow_y': 104.0,
    'l_wrist_z': -4.0,
    'l_wrist_x': -55.0,
    'l_thumb_z': -62.0,
    'l_thumb_x': -180.0,
    'l_indexfinger_x': -170.0,
    'l_middlefingers_x': -180.0,
    'r_shoulder_z': -25,
    'r_shoulder_y': 84,
    'r_arm_x': 47,
    'r_elbow_y': 94,
    'r_wrist_z': -59,
    'r_wrist_x': 114,
    'r_thumb_z': -1,
    'r_thumb_x': 44,
    'r_indexfinger_x': -90,
    'r_middlefingers_x': 38.0,
    'head_z': 0.0,
    'head_y': 0.0
}

def nicodeg2rad(nicojoints, nicodegrees):
    if isinstance(nicojoints, str):
        nicojoints = [nicojoints]
    if isinstance(nicodegrees, (int, float)):
        nicodegrees = [nicodegrees]

    rads = []

    for nicojoint, nicodegree in zip(nicojoints, nicodegrees):
        if nicojoint == 'r_wrist_z' or nicojoint == 'l_wrist_z':
            rad = deg2rad(nicodegree/2)
        elif nicojoint == 'r_wrist_x' or nicojoint == 'l_wrist_x':
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

def get_joints_limits(robot_id, num_joints):
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


    return [joints_limits_l,
            joints_limits_u], joints_ranges, joints_rest_poses, joint_names, link_names, joint_indices


def get_real_joints(robot, joints):
    last_position = []

    for k in joints:
        actual = robot.getAngle(k)
        # print("{} : {}, ".format(k,actual),end="")
        last_position.append(actual)
    # print("")

    return last_position

def spin_simulation(steps):
    for i in range(steps):
        p.stepSimulation()
        time.sleep(0.01)


def main():
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

    robot_id = p.loadURDF("./calibration.urdf", [0, 0, 0])
    # Create table mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.31, .45, 0.025],
                                                               rgbaColor=[0.6, 0.6, 0.6, 1]),
                      baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[.31, .45, 0.025]),
                      baseMass=0, basePosition=[0.27, 0, 0.029])
    # Create tablet mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.16, .26, 0.02],
                                                               rgbaColor=[0, 0, 0.0, 1]),
                      baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                                     halfExtents=[.16, .26, 0.02]), baseMass=0,
                      basePosition=[0.41, 0, 0.036])

    num_joints = p.getNumJoints(robot_id)
    joints_limits, joints_ranges, joints_rest_poses, joint_names, link_names, joint_indices = get_joints_limits(
        robot_id, num_joints)
    # Custom intital position

    # joints_rest_poses = deg2rad([-15, 68, 2.8, 56.4, 0.0, 11.0, -70.0])

    #actuated_joints, actuated_initpos = match_joints(init_pos, joint_names)

    # Real robot initialization and setting all joints
    

    from nicomotion.Motion import Motion
    motorConfig = './nico_humanoid_upper_rh7d_ukba.json'
    try:
        robot = Motion(motorConfig=motorConfig)
        print('Robot initialized')
    except:
        print('Motors are not operational')
        exit()
        # robot = init_robot()

    while True:
        actual_position = get_real_joints(robot, joint_names)
        for i in range(len(joint_indices)):
            p.resetJointState(robot_id, joint_indices[i], nicodeg2rad(joint_names[i],actual_position[i]))

        keypress = p.getKeyboardEvents()
        if ord('d') in keypress:
                robot.disableTorqueAll()
                #print("Torque disabled in all joints")
        if ord('f') in keypress:
                robot.enableTorqueAll()
                #print("Torque enabled in all joints")
        if ord('q') in keypress:
                break
        spin_simulation(1)

    p.disconnect()


if __name__ == "__main__":
    main()
