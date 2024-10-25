import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, round, any, mean 
import argparse
import os
import csv

mode = 'calibrate' # anything else is exploratory mode

SPEED = 0.05
DELAY= 3
REPEAT = 20

reset_pose = [0, 0, 0, 40, 0, 50, 0,0, -180, -180, -180.0, -180.0, 0, 40, 0, 50, 0, 0, -180, -180.0, -180, -180.0]


marker_positionr = [[0.23,-0.267,0.055], #right lower touchscreen
                [0.30,-0.267,0.055], #right touchscreen edge 1
                [0.36,-0.267,0.055], #right touchscreen edge 2
                [0.465,-0.267,0.055], #right touchscreen edge 3
                [0.465,-0.107,0.055], #touchscreen place 1
                [0.44,0.023,0.055], #touchscreen place 2
                [0.23,0.0,0.055], #middle lower touchscreen
                [0.06,0.075,0.235], #top left front body corner
                [0.1,0.025,0.40], #left eye
                [0.1,0.0,0.36], # nose
                [0.1,-0.025,0.40], #right eye
                [-0.04,-0.045,0.45], #right head
                [-0.04,-0.0,0.45], #top head
                [0.06,0.075,0.055], #bottom left front body corner
                [0.25,-0.003,0.055], #mark sign middle low touchscreen
                ]

marker_position = [[0.23,0.267,0.055], #left lower touchscreen
                [0.33,0.267,0.055], #left touchscreen edge 1
                [0.43,0.267,0.055], #left touchscreen edge 2
                [0.23,0.167,0.055], #botton touchscreen edge 1
                [0.23,0.067,0.055], #bottom touchscreen edge 2
                [0.23,0.0,0.055], #middle lower touchscreen
                [0.23,-0.133,0.055], #bottom touchscreen edge 3
                [0.06,-0.075,0.235], #top right front body corner
                [0.06,-0.075,0.055], #bottom right front body corner
                [0.16,0.105,0.055], #left inner hole corner
                [0.16,-0.105,0.055], #right inner hole corner
                [0.25,-0.003,0.055], #mark sign middle low touchscreen
        ]
#calib_pose = ['Resting','Touchscreen corner','Touchscreen center','Left body','Left thumb','Right body','Eyes','Ears','Middlepoint','Boh arms',
#                'Right eye','Top head','Left eye','Left body','LF-right eye','LF-left eye','LF-right shoulder','LF-right thumb','LF-right index',]

calib_poser = ['right lower touchscreen','right touchscreen edge 1','right touchscreen edge 2','right touchscreen edge 3','touchscreen place 1',
            'touchscreen place 1','middle lower touchscreen','top left front body corner','left eye','nose','right eye','right head','top head',
                'Right eye','Top head','Left eye','Left body','LF-right eye','LF-left eye','LF-right shoulder','LF-right thumb','LF-right index',]

calib_pose = ['left lower touchscreen','left touchscreen edge 1','left touchscreen edge 2','bottom touchscreen edge 1','bottom touchscreen edge ',
            'middle lower touchscreen','bottom touchscreen edge 3', 'top right body corner',
            'bottom right body corner', 'left inner hole corner','right inner hole corner','mark sign middle low touchscreen']


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

def set_sim_robot(robot, robot_id,joint_names, joint_indices):
    actual_position = get_real_joints(robot, joint_names)
    for i in range(len(joint_indices)):
        p.resetJointState(robot_id, joint_indices[i], nicodeg2rad(joint_names[i],actual_position[i]))
    spin_simulation(5) 

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

def create_marker (position):
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.006,
                                                               rgbaColor=[1, 0, 0, .8]),
                      basePosition=position)

def main():
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

    robot_id = p.loadURDF("./nico_alljoints.urdf", [0, 0, 0])
    # Create table mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.30, .45, 0.025],
                                                               rgbaColor=[0.6, 0.6, 0.6, 1]),
                      baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[.30, .45, 0.025]),
                      baseMass=0, basePosition=[0.26, 0, 0.029])
    # Create tablet mesh
    p.createMultiBody(baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[.165, .267, 0.02],
                                                               rgbaColor=[0, 0, 0.0, 1]),
                      baseCollisionShapeIndex=p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                                     halfExtents=[.165, .267, 0.02]), baseMass=0,
                      basePosition=[0.395, 0, 0.036])

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
    results= []
    if mode ==  'calibrate':
        reset_robot(robot, init_pos, reset_pose)
        time.sleep(DELAY)    
        with open('calib_lh_short.csv', mode='r') as csvfile:
            csvreader = csv.reader(csvfile)
            for row in csvreader:  # Iterate through each row in the CSV file
                for iter in range(REPEAT):
                
                    row = array(row, dtype=float)
                    reset_robot(robot, init_pos, row)
                    create_marker(marker_position[index])
                    #check_execution(robot, init_pos, row, 3, False)
                    time.sleep(DELAY)
                    set_sim_robot(robot, robot_id, joint_names, joint_indices)
                    #sim_pos = p.getLinkState(robot_id,10) # right end effector
                    sim_pos = p.getLinkState(robot_id,20) # left end effector
                    print("{}.{} error: {} ".format(index+1, calib_pose[index], array(marker_position[index]) - array(sim_pos[0])))
                    p.addUserDebugText(f"Calibrating {calib_pose[index]}",[.0, -0.3, .60], textSize=2, lifeTime=4, textColorRGB=[1, 0, 0])
                    p.addUserDebugText(f"X,Y,Z error: {array(marker_position[index]) - array(sim_pos[0])}",[.0, -0.3, .55], textSize=2, lifeTime=4, textColorRGB=[1, 0, 0])
                    results.append(tuple(array(marker_position[index]) - array(sim_pos[0])))
                    reset_robot(robot, init_pos, reset_pose)
                    time.sleep(DELAY)
                    set_sim_robot(robot, robot_id, joint_names, joint_indices)                
                index += 1
            
        with open('lhcalib_output.csv', 'w', newline='') as csvoutfile:
            # Create a csv.writer object for this file
            csvwriter = csv.writer(csvoutfile)
    
            # Iterate over the results, writing each step as a row in the CSV file
            for step in results:
                csvwriter.writerow(step)

        print(results)

    else:
        processed_keys = set()
        while True:
            actual_position = get_real_joints(robot, joint_names)
            for i in range(len(joint_indices)):
                p.resetJointState(robot_id, joint_indices[i], nicodeg2rad(joint_names[i],actual_position[i]))
            #print("Actual position: ", actual_position,  end='\r')
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
