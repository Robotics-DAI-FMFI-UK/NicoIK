import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, round, any, mean 
import argparse
import os
import csv

SPEED = 0.03
SPEEDF = 0.09
DELAY= 2
REPEAT = 1

reset_pose = [0,0,0,90,90,90,0,0,-180,-180,-180,-180,0.22,12.88,11.03,100.97,-24.13,-91.91,-180.0,-180.0,-180.0,-174.81]
drop_pose = [0,0,-20,27,40,90,125,100,180,20,20,20,0,13,11,100,-24,-91,-180.0,-180.0,-180.0,-175]
grasp_pose= [0,0,-2,34,17,122,161,-13,180,-180,-180,-180,0,13,11,100,-24,-91,-180.0,-180.0,-180.0,-175]
init_pos = {  # standard position
    'head_z': 0.0,
    'head_y': 0.0,
    'r_shoulder_z': 1,
    'r_shoulder_y': 87,
    'r_arm_x': 88,
    'r_elbow_y': 87,
    'r_wrist_z': 2,
    'r_wrist_x': -29,
    'r_thumb_z': -1,
    'r_thumb_x': 44,
    'r_indexfinger_x': -90,
    'r_middlefingers_x': 100.0,
    'l_shoulder_z': -24.0,
    'l_shoulder_y': 13.0,
    'l_arm_x': 0.0,
    'l_elbow_y': 104.0,
    'l_wrist_z': -4.0,
    'l_wrist_x': -55.0,
    'l_thumb_z': -62.0,
    'l_thumb_x': -180.0,
    'l_indexfinger_x': -170.0,
    'l_middlefingers_x': -180.0
}

def calculate_ik(robot_id,end_effector_index,pos,ori):
    ik_solution = p.calculateInverseKinematics(robot_id,
                                                end_effector_index,
                                                pos,
                                                targetOrientation=p.getQuaternionFromEuler(ori),
                                                maxNumIterations=300,
                                            residualThreshold=0.0001)
    return ik_solution



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

def reset_robot(robot, init_pos, values):
    index=0
    for k in init_pos.keys():
        robot.setAngle(k, float(values[index]), SPEED)
        index += 1
    return robot

def close_hand(robot):
    closing = True
    for angle in range(-180, 20, 20):
        robot.setAngle('r_thumb_z', 180, SPEEDF)
        robot.setAngle('r_thumb_x', angle, SPEEDF)
        robot.setAngle('r_indexfinger_x', angle, SPEEDF)
        robot.setAngle('r_middlefingers_x', angle, SPEEDF)
        time.sleep(0.2)

def open_hand(robot):
    closing = True
    for angle in range(20, -180, -20):
        robot.setAngle('r_thumb_z', 180, SPEEDF)
        robot.setAngle('r_thumb_x', angle, SPEEDF)
        robot.setAngle('r_indexfinger_x', angle, SPEEDF)
        robot.setAngle('r_middlefingers_x', angle, SPEEDF)
        time.sleep(0.2)

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
        if link_name.decode("utf-8") == 'endeffector':
                end_effector_index = jid


    return [joints_limits_l,
            joints_limits_u], joints_ranges, joints_rest_poses, joint_names, link_names, joint_indices, end_effector_index


def get_real_joints(robot, joints):
    last_position = []

    for k in joints:
        actual = robot.getAngle(k)
        # print("{} : {}, ".format(k,actual),end="")
        last_position.append(actual)
    # print("")

    return last_position


def main():
    p.connect(p.DIRECT)
    robot_id = p.loadURDF("./nico_upper_rh6d_r.urdf", [0, 0, 0])
    num_joints = p.getNumJoints(robot_id)
    joints_limits, joints_ranges, joints_rest_poses, joint_names, link_names, joint_indices, end_effector_index = get_joints_limits(
        robot_id, num_joints)

    from nicomotion.Motion import Motion
    motorConfig = './nico_humanoid_upper_rh7d_ukba.json'
    #try:
    #    robot = Motion(motorConfig=motorConfig)
    #    print('Robot initialized')
    #except:
    #    print('Motors are not operational')
    #    exit()
    position = [0.3,-0.2,0.1]
    orientation = [0, 0, 3.14]
    ik = calculate_ik(robot_id, end_effector_index,position, orientation)
    print(end_effector_index)
    print(ik)
    print(joint_names)

    real_ik = rad2nicodeg(joint_names, ik)
    print(real_ik)
    print (grasp_pose[2:8])
    #print (real_ik[2:7])
    input("wait")
    reset_robot(robot, init_pos, reset_pose)
    time.sleep(DELAY)    
    reset_robot(robot, init_pos, grasp_pose)
    time.sleep(DELAY)
    close_hand(robot)
    time.sleep(DELAY)
    robot.setAngle('r_elbow_y', 90, SPEED)
    time.sleep(DELAY)
    reset_robot(robot, init_pos, drop_pose)                    
    time.sleep(DELAY)
    open_hand(robot)
    time.sleep(DELAY)
    reset_robot(robot, init_pos, reset_pose)
    time.sleep(DELAY)
    p.disconnect()



if __name__ == "__main__":
    main()
