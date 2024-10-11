import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, round, any, mean 
import argparse
import os
import csv

mode = 'calibrate' # anything else is exploratory mode

SPEED = 0.05
DELAY= 3

calibration_matrix = [[0, 0, 0, 20, 0, 50, 0,0, -180, -180, -180.0, -180.0, 0, 20, 0, 50, 0, 0, -180, -180.0, -180, -180.0],
                    [-0.4, -1.45, 9.98, 129.89, -8.92, 139.56, 18.15, -19.65, -180.0, -180.0, -180.0, -180.0, 10.15, 135.69, -18.86, 160.22, 29.05, -50.95, -179.47, -180.0, -180.0, -180.0],
                    [-0.31, -1.45, 8.66, 93.41, -4.44, 141.93, 25.71, -18.51, -180.0, -180.0, -180.0, -180.0, 2.15, 94.37, -20.62, 160.22, 31.16, -92.62, -179.47, -180.0, -180.0, -180.0],
                    [-4.53, 14.29, 73.89, 0.22, 74.51, 49.629999999999995, -118.81, 163.56, -180.0, -179.47, -180.0, -180.0, -5.05, 3.74, 26.15, 92.62, -121.63, -9.01, -180.0, -180.0, -180.0, -180.0],
                    [-4.53, 14.29, -0.22, 19.74, 31.78, 117.85, -70.64, 163.56, -180.0, -180.0, -180.0, -180.0, 76.26, -3.91, 81.54, 51.56, -131.3, 130.77, -180.0, -180.0, -180.0, -180.0],
                    [-4.62, 14.37, -0.13, 19.74, 31.25, 117.41, -68.26, 164.18, -180.0, -180.0, -180.0, -180.0, 77.41, -14.29, 52.88, 51.650000000000006, -122.24, 133.05, -180.0, -180.0, -180.0, -180.0],
                    [-4.53, 14.29, -0.13, 19.74, 30.99, 117.41, -67.74, 162.42, -180.0, -180.0, -180.0, -180.0, 89.71, 67.38, 40.31, 61.05, -79.16, 133.05, -180.0, -180.0, -180.0, -180.0],
                    [-4.35, 14.29, 81.89, 75.65, 39.34, 61.58, -81.8, 180.0, -180.0, -180.0, -180.0, -180.0, 10.59, 10.33, 22.55, 103.52000000000001, -46.81, 133.05, -180.0, -180.0, -180.0, -180.0],
                    [-0.66, -18.59, 75.38, -2.15, 86.29, 51.56, -132.97, 180.0, -180.0, -179.65, -180.0, -180.0, 11.21, 20.0, 5.49, 112.48, -11.12, -123.91, -178.33, -180.0, -180.0, -180.0],
                    [0, 0, -34.51, -62.9, 118.46, 127.08, -127.08, 118.37, -180, -180, -180.0, -180.0, -27.03, -62.29, 113.1, 132.18, -97.8, 145.8, -180, -180.0, -180, -180.0]]

reset_pose = [0, 0, 0, 40, 0, 50, 0,0, -180, -180, -180.0, -180.0, 0, 40, 0, 50, 0, 0, -180, -180.0, -180, -180.0]

calib_pose = ['Resting','Touchscreen corner','Touchscreen center','Left body','Left thumb','Right body','Eyes','Ears','Middlepoint','Boh arms',
                'Right eye','Top head','Left eye','Left body','LF-right eye','LF-left eye','LF-right shoulder','LF-right thumb','LF-right index',]

init_pos = {  # standard position
    'head_z': 0.0,
    'head_y': 0.0,
    'r_shoulder_z': 84,
    'r_shoulder_y': 84,
    'r_arm_x': 47,
    'r_elbow_y': 94,
    'r_wrist_z': -59,
    'r_wrist_x': 114,
    'r_thumb_z': -1,
    'r_thumb_x': 44,
    'r_indexfinger_x': -90,
    'r_middlefingers_x': 38.0,
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

def check_execution(robot, joints, target, accuracy, verbose):
    tic = time.time()
    distance = 100
    step = 0
    while distance > accuracy:
        actual = get_real_joints(robot, joints)
        # print(timestamp)
        diff = array(target) - array(actual)
        distance = linalg.norm(diff)
        if verbose:
            print('RealNICO Step: {}, Time: {:.2f}, JointDeg: {}'.format(step, time.time() - tic, ['{:.2f}'.format(act) for act in actual]))
        else:
            print('Duration: {:.2f}, Error: {:.2f}'.format(time.time() - tic, distance), end='\r')
        time.sleep(0.01)
        step += 1
    toc = time.time()
    print("\n")
    return toc - tic

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

    robot_id = p.loadURDF("./nico_alljoints.urdf", [0, 0, 0])
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
    index = 0
    if mode ==  'calibrate':
        actual_position = get_real_joints(robot, joint_names)
        for i in range(len(joint_indices)):
            p.resetJointState(robot_id, joint_indices[i], nicodeg2rad(joint_names[i],actual_position[i]))
    
        with open('calibration.csv', mode='r') as csvfile:
            csvreader = csv.reader(csvfile)
            for row in csvreader:  # Iterate through each row in the CSV file
                row = array(row, dtype=float)
                reset_robot(robot, init_pos, row)
                check_execution(robot, init_pos, row, 3, False)
                actual_position = get_real_joints(robot, joint_names)
                for i in range(len(joint_indices)):
                    p.resetJointState(robot_id, joint_indices[i], nicodeg2rad(joint_names[i],actual_position[i]))
                spin_simulation(1)   

                print("{}.{} ".format(index+1, calib_pose[index]))
                time.sleep(DELAY)
                #input("Press Enter to continue...")
                reset_robot(robot, init_pos, reset_pose)
                time.sleep(DELAY)
                index += 1

    else:
        processed_keys = set()
        while True:
            actual_position = get_real_joints(robot, joint_names)
            for i in range(len(joint_indices)):
                p.resetJointState(robot_id, joint_indices[i], nicodeg2rad(joint_names[i],actual_position[i]))
            print("Actual position: ", actual_position,  end='\r')
            keypress = p.getKeyboardEvents()
            if ord('d') in keypress:
                robot.disableTorqueAll()
                #print("Torque disabled in all joints")
            if ord('f') in keypress:
                robot.enableTorqueAll()
                #print("Torque enabled in all joints")
            if ord('a') in keypress:
                all_position = get_real_joints(robot, init_pos.keys())
                #print(init_pos.keys())
                print("Actual position: ", all_position)
                with open('calibration.csv', 'a', newline='') as csvfile:
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerow(all_position)
                keypress.clear()
                time.sleep(1)
                processed_keys.add(ord('a'))
            elif ord('a') not in keypress and ord('a') in processed_keys:
                processed_keys.remove(ord('a'))            

            if ord('q') in keypress:
                break
            keypress.clear()
            spin_simulation(1)

    p.disconnect()



if __name__ == "__main__":
    main()
