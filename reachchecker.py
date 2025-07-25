import pybullet as p
import time
from numpy import random, rad2deg, deg2rad, set_printoptions, array, linalg, round, any, mean, pi
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import os
from sim_height_calculation import calculate_z
import calibration_matrices

# Graspchecker config
#GOALORI = [0, 0, 1]
EEACCURACY = 0.05
JOINTACCURACY = 0.05
OPENGRIPPER = 0.04
CLOSEDGRIPPER = 0.03
Z_OFFSET = 0.15
FORCE = 500
# Set speed to reseti to initial position
RESET_SPEED = 0.02
# Acccuracy (vector distance) to consider the target positon reached
ACCURACY = 3


# Delay between simulation steps
SIM_STEP_DELAY = 0.01
# Amount of decimals to round to when writing trajectory to file
DECIMALS = 10


init_pos = {  # standard position
    'arm_right_1_rjoint': -1.17,
    'arm_right_2_rjoint': -2.2,
    'arm_right_3_rjoint': -0.7,
    'arm_right_4_rjoint': -1.3,
    'arm_right_5_rjoint': 0.8,
    'arm_right_6_rjoint': -1.4,
    'arm_right_7_rjoint': 1.0,
    'gripper_right_right_finger_gjoint': 0.04,
    'gripper_right_left_finger_gjoint': 0.04
}



set_printoptions(precision=3)
set_printoptions(suppress=True)

def save_image(file_name):
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(1920, 1080, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgbImg = rgbImg[:, :, :3]
    plt.imsave(file_name, rgbImg)


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
        #print(joint_info)
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


def check_response(robot, joints, joints_state_before, tic):
    while True:
        actual = get_real_joints(robot, joints)
        diff = array(joints_state_before) - array(actual)
        if any(diff):
            break
        print('Duration: {:.2f}'.format(time.time() - tic), end='\r')
        time.sleep(0.01)
    toc = time.time()
    return toc - tic


def check_execution(robot, joints, target,accuracy, verbose):
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
    return toc - tic


def to_formatted_str(number):
    number = str(number)

    decimal_point_index = number.find('.')
    if decimal_point_index >= 0:
        number += '0' * (DECIMALS - (len(number) - decimal_point_index - 1))

    return number


def write_first_line(file):
    offset1, offset2 = 7 * (DECIMALS + 6), DECIMALS + 20
    first_line = "JOINTS ANGLES (DEGREES)" + " " * (offset1 - 23) + "DURATION (SECONDS)" + " " * (
                offset2 - 18) + "END EFFECTOR POSITION (CARTEZIAN)"
    file.write(first_line + "\n")


def write_line(file, joint_angles, duration, end_effector_coords):
    offset1, offset2 = 7 * (DECIMALS + 6), DECIMALS + 20

    length = file.write("%s " % ','.join(list(map(to_formatted_str, round(joint_angles, DECIMALS)))))
    file.write(" " * (offset1 - length))
    length = file.write("%s " % to_formatted_str(round(duration, DECIMALS)))
    file.write(" " * (offset2 - length))
    file.write("%s\n" % ','.join(list(map(to_formatted_str, round(end_effector_coords, DECIMALS)))))

def calculate_ik_position(robot_id, end_effector_index, target_position, max_iterations, residual_threshold):
    return p.calculateInverseKinematics(robot_id, end_effector_index, target_position,
                                        maxNumIterations=max_iterations, residualThreshold=residual_threshold)

def calculate_ik_orientation(robot_id, end_effector_index, target_position, target_orientation, max_iterations, residual_threshold):
    return p.calculateInverseKinematics(robot_id, end_effector_index, target_position, targetOrientation=target_orientation,
                                        maxNumIterations=max_iterations, residualThreshold=residual_threshold)
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true", help="Show detail informtion about robot position in terminal")
    parser.add_argument("-p", "--position", nargs=3, type=float, help="Target position for the robot end effector as a list of three floats.")
    parser.add_argument("-to", "--target_object", type=str, help="Target object")
    parser.add_argument("-o", "--orientation", nargs=3, type=float, help="Target orientation for the robot end effector as a list of four floats.")
    parser.add_argument("-rr", "--real_robot", action="store_true", help="If set, execute action on real robot.")
    parser.add_argument("-a", "--animate", action="store_true", help="If set, the animation of motion is shown.")
    parser.add_argument("-g", "--gui", action="store_true", help="If set, turn the GUI on")
    parser.add_argument("-gr", "--grasp", action="store_true", help="If set, test grasp")
    parser.add_argument("-rp", "--robot_pos", nargs=3, default = [0,0,0], type=float, help="Target orientation for the robot end effector as a list of four floats.")
    parser.add_argument("-ro", "--robot_ori", nargs=3, default = [0,0,0], type=float, help="Target orientation for the robot end effector as a list of four floats.")
    parser.add_argument("-r", "--robot", type=str, default="./urdf/tiago_dual_mygym.urdf", help="Duration of movement in si/real robot")
    parser.add_argument("-en", "--environment", type=str, default="./urdf/table_tiago.urdf", help="Duration of movement in si/real robot")
    parser.add_argument("-l", "--left", action="store_true", help="If set, use left hand IK")
    parser.add_argument("-i", "--initial", action="store_true", help="If set, reset the robot to the initial position after each postion")
    parser.add_argument("-ip", "--iposition", nargs=3, default = [-0.2, -0.5, 1.2], type=float, help="Initial position for the robot end effector as a list of three floats.")
    parser.add_argument("-io", "--iorientation", nargs=3, type=float, help="Initial orientation for the robot end effector as a list of four floats.")
    parser.add_argument("-t", "--trajectory", type=str, help="If set, execute trajectory positions from the text file with corresponding path")
    parser.add_argument("-of", "--output_file", type=str,  default = 'test', help="If set, execute trajectory positions from the text file with corresponding path")
    parser.add_argument("-c", "--calibration", nargs=9, type = float, default=[0.2, 0.8, 0.3,-0.7, 0.7, 0.1, 0.77, 0.77, 0.2], help="Duration of movement in si/real robot")
    parser.add_argument("-e", "--experiment", action="store_true", help="If set, execute experiments positions")
    parser.add_argument("-s", "--speed", type=float, default=1, help="Speed of arm movement in simulator")
    parser.add_argument("-d", "--duration", type=float, default=2, help="Duration of movement in si/real robot")
    parser.add_argument("-ts", "--trajectory_steps", type=int, default=5, help="Number of steps in each trajectory")
    parser.add_argument("-st", "--save_trajectory", action="store_true", help="Store coordinates from simulation into text file")
    parser.add_argument("-cz", "--calculate_z_value", action="store_true", help="If set, the z value is calculated from calibration grid")
    parser.add_argument("-ss", "--save_statistics", action="store_true", help="If set, joint difference and time statistics are saved into text files")
    arg_dict = vars(parser.parse_args())

    # TouchAgent()
    # TouchAgent.clean()

    # GUI initialization
    if arg_dict["gui"]:
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-70, cameraTargetPosition=[0, 0, 0.7])
    else:
        p.connect(p.DIRECT)

    # Load the URDF robot a create scene
    floor = p.loadURDF("./urdf/plane.urdf")
    floortex = p.loadTexture("./textures/parquet1.jpg")
    p.changeVisualShape(floor, -1, textureUniqueId=floortex)

    robot_id = p.loadURDF(arg_dict["robot"], arg_dict["robot_pos"],p.getQuaternionFromEuler(arg_dict["robot_ori"]), flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
    
    table_id = p.loadURDF(arg_dict["environment"],basePosition = [0,0,0], baseOrientation = p.getQuaternionFromEuler([0,0,-pi/2]), useFixedBase=True)

    tabletex = p.loadTexture("./textures/table.jpg")

    p.changeVisualShape(table_id, -1, rgbaColor=[0.8, 0.7, 0.5, 1.0])
    p.changeVisualShape(table_id, -1, textureUniqueId=tabletex)

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
        robot = reset_robot(robot, init_pos)
        time_res = check_execution(robot, init_pos.keys(), list(init_pos.values()), 10, arg_dict["verbose"])
        print('Robot reset in {:.2f} seconds.'.format(time_res))
        actual_position = get_real_joints(robot, actuated_joints)

    #spin_simulation(50)

    # IK paramenters
    max_iterations = 300
    residual_threshold = 0.0001

    # Statistics
    TargetPstat = []        # target position x, y
    TimeRstat = []          # response time
    IKposstat = []
    IKoristat = []
    IKjointstat = []
    JointIstat = []
    TimeIstat = []
    JointGstat = []
    TimeGstat = []
    state = []
    sim_time_errors = []
    finished = False
    failed = 0
    failpos = 0
    failori = 0
    failjoint = 0   
    #poserror = []
    #orierror = []
    #jointerror = []
    reacherror = []

    movement_duration = arg_dict["duration"]

    # if arg_dict["file"]:
    #    open(arg_dict["file"]) as data

    # Reads trajectory points from file
    trajectory_joints, trajectory_durations, trajectory_end_effector_positions = [], [], []
    if arg_dict["trajectory"] is not None:
        file_name = arg_dict['trajectory']
        with open(file_name, 'r') as f:
            content = f.read().split('\n')[1:-1]

        if content:
            for pos in content:
                joints, duration, end_effector_position = pos.split()
                trajectory_joints.append(list(map(float, joints.split(','))))
                trajectory_durations.append(float(duration))
                trajectory_end_effector_positions.append(list(map(float, end_effector_position.split(','))))
            print("Movement duration is being overwritten by a value in trajectory file.")
        else:
            print("File empty")

    # Creates needed directories if not created yet
    if arg_dict["save_trajectory"]:
        if not os.path.exists("trajectories"):
            os.mkdir("trajectories")
        if not os.path.exists("linear_trajectories"):
            os.mkdir("linear_trajectories")
        if not os.path.exists("test_trajectories"):
            os.mkdir("test_trajectories")
    
    if arg_dict["save_statistics"]:
        if not os.path.exists("statistics"):
            os.mkdir("statistics")

    elif arg_dict["calibration"]:
        grid = calibration_matrices.TargetGridTiagoTable(*arg_dict["calibration"])
    
    if arg_dict["iorientation"]:
        ik_init = calculate_ik_orientation(robot_id, end_effector_index, arg_dict["iposition"],p.getQuaternionFromEuler(arg_dict["iorientation"]), max_iterations, residual_threshold)
    else:
        ik_init = calculate_ik_position(robot_id, end_effector_index, arg_dict["iposition"], max_iterations, residual_threshold)
    if not arg_dict["initial"]:
        for i in range(len(joint_indices)):
            p.resetJointState(robot_id, joint_indices[i], ik_init[i])
            time.sleep(0.2)
    try:
        for i in range(1000):
            # ik_solution = tuple()
            # while True:
            # Target position
            if arg_dict["position"]:
                target_position = arg_dict["position"]
            elif arg_dict["calibration"]:
                target_position = next(grid)
                if i ==0:
                    target_pos_relative = array([0,0,0])
                    target_first = target_position
                else:
                    target_pos_relative = array(target_position) - array(target_first)
                #print(target_position)
            elif arg_dict["experiment"]:
                target_position = calibration_matrices.target_experiment(i)
            elif arg_dict["trajectory"] is not None:
                index = i % len(trajectory_end_effector_positions)
                target_position = trajectory_end_effector_positions[index]
                # ik_solution = deg2rad(trajectory_joints[index])
                movement_duration = trajectory_durations[index]
            # elif arg_dict["file"]:
            #    target_position = data[i]
            elif arg_dict["joints"]:
                target_position = target_joints(i)
            else:
                target_position = calibration_matrices.target_random()

            #print (target_position)
            if arg_dict["calculate_z_value"]:
                target_position[2] = calculate_z(target_position[0], target_position[1])
            
            TargetPstat.append(target_position[:2])

            # Create goal dot
            if arg_dict["target_object"]:
                object_id = p.loadURDF('./ycb/'+ arg_dict["target_object"],basePosition = target_position, useFixedBase=False)
                p.changeVisualShape(object_id, -1, rgbaColor=[1, 0, 0, 1.0])
                spin_simulation(100)
            else:
                p.createMultiBody(
                    baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.007, rgbaColor=[0, 0, 1, .5]),
                    baseCollisionShapeIndex=-1, baseMass=0, basePosition=target_position)

            if arg_dict["grasp"]:
                #Find 'gjoint' in joint_names and store corresponding joint_indices values in gjoint_indices
                gjoint_indices = [joint_indices[i] for i, joint in enumerate(joint_names) if 'gjoint' in joint]

                print(gjoint_indices)  # Output: [1, 3]
                target_position = [target_position[0], target_position[1], target_position[2] + Z_OFFSET]

            # Reset robot to initial position
            if arg_dict["initial"]:
                

                resetsim_pos = []
                
                for j in range(len(joint_indices)):
                    p.resetJointState(robot_id, joint_indices[j],ik_init[j])
                for j in range(len(joint_indices)):
                        resetsim_pos.append(p.getJointState(robot_id, joint_indices[j])[0])
                time.sleep(0.1)
                #CONVERT TO NICO DEGREES
                nicodeg_resetsim_pos = resetsim_pos
                
                
                #simdifference = array(actuated_initpos) - array(nicodeg_resetsim_pos)
                (x, y, z), (a, b, c, d), _, _, _, _ = p.getLinkState(robot_id, end_effector_index)
                if not arg_dict["verbose"]:
                    print('Time init: 0s \n Error: {} \n Goal: {} \n Real: {} \n InitPos: {}'.format(
                                                                        ['{:.2f}'.format(diff) for diff in simdifference],
                                                                        ['{:.2f}'.format(goal) for goal in actuated_initpos],
                                                                        ['{:.2f}'.format(sim) for sim in nicodeg_resetsim_pos],
                                                                        ['{:.2f}'.format(simpos) for simpos in [x, y, z]]
                                                                        ))
                if arg_dict["real_robot"]:
                    robot = reset_actuated(robot, actuated_joints, actuated_initpos)
                    time_res = check_execution(robot, actuated_joints, actuated_initpos, 3, arg_dict["verbose"])
                    reset_pos = get_real_joints(robot, actuated_joints)
                    difference = array(actuated_initpos) - array(reset_pos)
                    if arg_dict["verbose"]:
                        print('RealNICO init: {:.2f}s \n Error: {} \n Goal: {} \n Real: {}'.format(time_res,
                                                                        ['{:.2f}'.format(diff) for diff in difference],
                                                                        ['{:.2f}'.format(goal) for goal in actuated_initpos],
                                                                        ['{:.2f}'.format(real) for real in reset_pos]))
                        input('Compare real and sim position visually')
                    JointIstat.append(difference)
                    TimeIstat.append(time)
                    
                # spin_simulation(20)

            # target_orientation = target_position + [1]
            # Perform IK
            # ik_solution = p.calculateInverseKinematics(robot_id, end_effector_index, target_position,
            #                                           targetOrientation=target_orientation,
            #                                        maxNumIterations=max_iterations,
            #                                        residualThreshold=residual_threshold)

            # if not len(ik_solution):            # If we are reading trajectory from the file, we don't need to calculate
            #target_orientation = [-0.18901219284742923, -0.2983003154209216, 0.8705412315868658, 0.3427087347617619]
            if arg_dict["orientation"]:
                ik_solution = calculate_ik_orientation(robot_id,
                                                            end_effector_index,
                                                            target_position,
                                                            p.getQuaternionFromEuler(arg_dict["orientation"]),
                                                            max_iterations,
                                                            residual_threshold)
            
            else:
                ik_solution = calculate_ik_position(robot_id,
                                                            end_effector_index,
                                                            target_position,
                                                            max_iterations,
                                                            residual_threshold)


            trajectory = []

            save_trajectory_joints, save_trajectory_durations, save_trajectory_end_effector_coords = [], [], []

            if arg_dict["animate"]:
                for j in range(len(joint_indices)):
                    speed = sim_speed_control(p.getJointState(robot_id, joint_indices[j])[0], ik_solution[j],
                                            movement_duration)

                    p.setJointMotorControl2(robot_id, joint_indices[j],
                                            p.POSITION_CONTROL, ik_solution[j],
                                            maxVelocity=speed,
                                            force=FORCE,
                                            positionGain=0.7,
                                            velocityGain=0.3)

                tic = time.perf_counter()

                step = 1
                while not finished:
                    for j in range(len(joint_indices)):
                        state.append(p.getJointState(robot_id, joint_indices[j])[0])
                    
                    #print(state)
                    
                    simdiff = array(ik_solution) - array(state)
                    if arg_dict["verbose"]:
                        #CONVERT TO NICO DEGREES
                        nicodeg_pos = state
                        #print('SimNICO, Step: {}, JointDeg: {}'.format(step, ['{:.2f}'.format(pos) for pos in nicodeg_pos], end='\n'))
                    

                    # Saving trajectory points in lists for writing into file
                    if arg_dict["save_trajectory"]:
                        save_trajectory_joints.append(state)
                        save_trajectory_durations.append(time.perf_counter() - tic)
                        save_trajectory_end_effector_coords.append(p.getLinkState(robot_id, end_effector_index)[0])
                    spin_simulation(1)
                    step += 1
                    last_state = state
                    state = []

                    if linalg.norm(simdiff) <= JOINTACCURACY:
                        finished = True

                    if step > 350:
                        #failed += 1
                        #print('ANIMATION MODE FAILED - NEEDS DEBUGGGING')
                        
                        finished = True
                        

                toc = time.perf_counter()

                # Saving the last point of trajectory
                if arg_dict["save_trajectory"]:
                    for j in range(len(joint_indices)):
                        state.append(p.getJointState(robot_id, joint_indices[j])[0])
                    save_trajectory_joints.append(state)
                    save_trajectory_durations.append(toc - tic)
                    save_trajectory_end_effector_coords.append(p.getLinkState(robot_id, end_effector_index)[0])
                    state = []

                # print('Sim time using step count:', step * SIM_STEP_DELAY)

                # print("sim time using time module:", toc - tic)

                sim_time_error = abs(arg_dict['duration'] - step * SIM_STEP_DELAY)
                # print('Sim time error using step count:', sim_time_error)
                sim_time_errors.append(sim_time_error)

                # WRITING TRAJECTORY POINTS TO TEXT FILE
                if arg_dict["save_trajectory"]:
                    steps = arg_dict["trajectory_steps"]

                    filename = 'trajectories/trajectory_' + str(i) + '.txt'
                    with open(filename, 'w') as f:
                        write_first_line(f)
                        if steps > 1:
                            i_diff = (len(save_trajectory_joints) - 1) / (steps - 1)
                            for j in range(steps):
                                index = int(j * i_diff)
                                if index != int((j - 1) * i_diff):
                                    duration = 1
                                    if j != 0:
                                        duration = save_trajectory_durations[index] - save_trajectory_durations[
                                            int((j - 1) * i_diff)]

                                    write_line(f, save_trajectory_joints[index], duration,
                                            save_trajectory_end_effector_coords[index])
                        else:
                            print("Trajectory steps must be greater than 1")
                            quit()

                    filename = 'linear_trajectories/trajectory_' + str(i) + '.txt'
                    with open(filename, 'w') as f:
                        write_first_line(f)
                        duration = arg_dict['duration'] / (steps - 1)
                        write_line(f, save_trajectory_joints[0], 1, save_trajectory_end_effector_coords[0])

                        end_effector_coords = list(save_trajectory_end_effector_coords[0])
                        end_effector_target_coords = save_trajectory_end_effector_coords[-1]
                        end_effector_step = ((end_effector_target_coords[0] - end_effector_coords[0]) / (steps - 1),
                                            (end_effector_target_coords[1] - end_effector_coords[1]) / (steps - 1),
                                            (end_effector_target_coords[2] - end_effector_coords[2]) / (steps - 1))
                        for j in range(steps - 2):
                            for k in range(3):
                                end_effector_coords[k] += end_effector_step[k]

                            ik_solution = p.calculateInverseKinematics(robot_id,
                                                                    end_effector_index,
                                                                    end_effector_coords,
                                                                    lowerLimits=joints_limits[0],
                                                                    upperLimits=joints_limits[1],
                                                                    jointRanges=joints_ranges,
                                                                    restPoses=joints_rest_poses,
                                                                    maxNumIterations=max_iterations,
                                                                    residualThreshold=residual_threshold)

                            write_line(f, ik_solution, duration, end_effector_coords)

                        write_line(f, save_trajectory_joints[-1], duration, save_trajectory_end_effector_coords[-1])

                    filename = 'test_trajectories/trajectory_' + str(i) + '.txt'
                    with open(filename, 'w') as f:
                        write_first_line(f)
                        for j in range(len(save_trajectory_joints)):
                            duration = 1
                            if j != 0:
                                duration = save_trajectory_durations[j] - save_trajectory_durations[j - 1]

                            write_line(f, save_trajectory_joints[j], duration, save_trajectory_end_effector_coords[j])

                finished = False

            else:
                tic = time.perf_counter()
                
                for j in range(len(joint_indices)):
                    p.resetJointState(robot_id, joint_indices[j], ik_solution[j])
                for j in range(len(joint_indices)):
                        state.append(p.getJointState(robot_id, joint_indices[j])[0])
                last_state = state
                simdiff = rad2deg(array(ik_solution)) - rad2deg(array(state))
                state = []
                toc = time.perf_counter()
                
                #spin_simulation(10)
        
            # Calculate IK solution error
            
            (x, y, z), (a, b, c, d), _, _, _, _ = p.getLinkState(robot_id, end_effector_index)
            
            
            IKdiff = (array(target_position) - array([x, y, z]))
            IKoridiff = (array (p.getQuaternionFromEuler(arg_dict["orientation"])) - array([a, b, c, d])) 
            # print('SimNico target_pos: {}'.format(target_position))
            #print('SimNico IK error: {}'.format(IKoridiff))
            p.createMultiBody(
                baseVisualShapeIndex=p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.01, rgbaColor=[0, 1, 0, .6]),
                baseCollisionShapeIndex=-1, baseMass=0, basePosition=[x, y, z])
            IKposstat.append(IKdiff)
            #poserror.append([target_position,IKdiff])
            IKoristat.append(IKoridiff)
            #orierror.append([target_position,IKoridiff])
            IKjointstat.append(simdiff)
            reacherror.append([target_pos_relative,IKdiff,IKoridiff,simdiff])
            #simdiff = rad2deg(array(ik_solution)) - rad2deg(array(last_state))
            #print('Cumulative  IK error: {}'.format(mean(IKstat, axis=0)), end='\r')
            meanposerror = mean(IKposstat, axis=0)
            meanorierror = mean(IKoristat, axis=0)
            meanjointerror = mean(IKjointstat, axis=0)
            if linalg.norm(IKdiff) > EEACCURACY:
                failpos += 1 
            if linalg.norm(IKoridiff) > EEACCURACY:
                failori += 1
            if linalg.norm(simdiff) > JOINTACCURACY:
                failjoint += 1
            p.addUserDebugText(f"Mean pos error:{meanposerror}",[.0, -0.6, 1.80], textSize=1.5, lifeTime=2, textColorRGB=[1, 0, 0]) 
            p.addUserDebugText(f"Mean ori error:{meanorierror}",[.0, -0.6, 1.70], textSize=1.5, lifeTime=2, textColorRGB=[1, 0, 0]) 
            p.addUserDebugText(f"Mean joint error:{meanjointerror}",[.0, -0.6, 1.60], textSize=1.5, lifeTime=2, textColorRGB=[1, 0, 0])
            p.addUserDebugText(f"Failed pos/ori/joint/total: {failpos} / {failori} / {failjoint} / {i+1}",[.0, -0.8, 1.5], textSize=1.5, lifeTime=2, textColorRGB=[1, 0, 0]) 
            #p.addUserDebugText(f"EE Orient: {p.getEulerFromQuaternion([a,b,c,d])}",[.0, -0.4, 1.55], textSize=1, lifeTime=2, textColorRGB=[1, 0, 0])
            #print([a,b,c,d])
            
            print(f"Failed pos/ori/joint/total: {failpos} / {failori} / {failjoint} / {i+1}")
            time.sleep(0.1)
            #CONVERT TO NICO DEGREES
            
            nicodeg_ik = ik_solution
            if arg_dict["verbose"]:
                #CONVERT TO NICO DEGREES

                print('Time: {:.2f}s \n Error: {} \n Goal: {} \n Real: {} \n PosError: {} \n GoalPos: {} \n RealPos: {} \n OriError: {} \n GoalOri: {} \n RealOri: {} \n'.format((toc - tic),
                                                                        ['{:.2f}'.format(diff) for diff in simdiff],
                                                                        ['{:.2f}'.format(goal) for goal in ik_solution],
                                                                        ['{:.2f}'.format(real) for real in last_state],
                                                                        ['{:.2f}'.format(posdiff) for posdiff in IKdiff],
                                                                        ['{:.2f}'.format(goalpos) for goalpos in target_position],
                                                                        ['{:.2f}'.format(realpos) for realpos in [x, y, z]],
                                                                        ['{:.2f}'.format(posdiff) for posdiff in IKoridiff],
                                                                        ['{:.2f}'.format(goalori) for goalori in arg_dict["orientation"]],
                                                                        ['{:.2f}'.format(realori) for realori in p.getEulerFromQuaternion([a, b, c, d])]))
                
                print (linalg.norm(simdiff))
                print (linalg.norm(IKdiff))
                print (linalg.norm(IKoridiff)) 
                                                                    
                
                

            if arg_dict["grasp"]:
                #open gripper gjoints
                spin_simulation(50)
                for j in range(len(gjoint_indices)):     
                    if arg_dict["animate"]:     
                        p.setJointMotorControl2(robot_id, gjoint_indices[j], p.POSITION_CONTROL, OPENGRIPPER, force=FORCE)
                        spin_simulation(50)
                    else:
                        p.resetJointState(robot_id, gjoint_indices[j], OPENGRIPPER)
                
                target_position2 = [target_position[0], target_position[1], target_position[2] - Z_OFFSET]

                if arg_dict["orientation"]:
                    ik_solution = calculate_ik_orientation(robot_id,
                                                            end_effector_index,
                                                            target_position2,
                                                            p.getQuaternionFromEuler(arg_dict["orientation"]),
                                                            max_iterations,
                                                            residual_threshold)
            
                else:
                    ik_solution = calculate_ik_position(robot_id,
                                                            end_effector_index,
                                                            target_position2,
                                                            max_iterations,
                                                            residual_threshold)    

                step = 1
                if arg_dict["animate"]:
                    for j in range(len(joint_indices)):
                        speed = sim_speed_control(p.getJointState(robot_id, joint_indices[j])[0], ik_solution[j],
                                                movement_duration)

                        p.setJointMotorControl2(robot_id, joint_indices[j],
                                                p.POSITION_CONTROL, ik_solution[j],
                                                maxVelocity=speed,
                                                force=FORCE,
                                                positionGain=0.7,
                                                velocityGain=0.3)

                    tic = time.perf_counter()

                    step = 1
                    while not finished:
                        for j in range(len(joint_indices)):
                            state.append(p.getJointState(robot_id, joint_indices[j])[0])
                        
                        #print(state)
                        
                        simdiff = array(ik_solution) - array(state)
                        if arg_dict["verbose"]:
                            #CONVERT TO NICO DEGREES
                            nicodeg_pos = state
                            #print('SimNICO, Step: {}, JointDeg: {}'.format(step, ['{:.2f}'.format(pos) for pos in nicodeg_pos], end='\n'))
                        
                        spin_simulation(1)
                        step += 1
                        last_state = state
                        state = []

                        if linalg.norm(simdiff) <= JOINTACCURACY:
                            finished = True

                        if step > 350:
                            #failed += 1
                            #print('ANIMATION MODE FAILED - NEEDS DEBUGGGING')
                            
                            finished = True

                    finished = False

                else:
                    tic = time.perf_counter()
                    
                    for j in range(len(joint_indices)):
                        p.resetJointState(robot_id, joint_indices[j], ik_solution[j])
                    for j in range(len(joint_indices)):
                            state.append(p.getJointState(robot_id, joint_indices[j])[0])
                    last_state = state
                    simdiff = rad2deg(array(ik_solution)) - rad2deg(array(state))
                    state = []
                    toc = time.perf_counter()
                    
                    #spin_simulation(10)
                #close gripper gjoints
                for j in range(len(gjoint_indices)):     
                    if arg_dict["animate"]:     
                        p.setJointMotorControl2(robot_id, gjoint_indices[j], p.POSITION_CONTROL, CLOSEDGRIPPER, force=FORCE)
                        spin_simulation(50)
                    else:
                        p.resetJointState(robot_id, gjoint_indices[j], CLOSEDGRIPPER)
                # Calculate IK solution error

                if arg_dict["orientation"]:
                    ik_solution = calculate_ik_orientation(robot_id,
                                                            end_effector_index,
                                                            target_position,
                                                            p.getQuaternionFromEuler(arg_dict["orientation"]),
                                                            max_iterations,
                                                            residual_threshold)
            
                else:
                    ik_solution = calculate_ik_position(robot_id,
                                                            end_effector_index,
                                                            target_position,
                                                            max_iterations,
                                                            residual_threshold)    

                step = 1
                if arg_dict["animate"]:
                    for j in range(len(joint_indices)):
                        speed = sim_speed_control(p.getJointState(robot_id, joint_indices[j])[0], ik_solution[j],
                                                movement_duration)

                        p.setJointMotorControl2(robot_id, joint_indices[j],
                                                p.POSITION_CONTROL, ik_solution[j],
                                                maxVelocity=speed,
                                                force=FORCE,
                                                positionGain=0.7,
                                                velocityGain=0.3)

                    tic = time.perf_counter()

                    step = 1
                    while not finished:
                        for j in range(len(joint_indices)):
                            state.append(p.getJointState(robot_id, joint_indices[j])[0])
                        
                        #print(state)
                        
                        simdiff = array(ik_solution) - array(state)
                        if arg_dict["verbose"]:
                            #CONVERT TO NICO DEGREES
                            nicodeg_pos = state
                            #print('SimNICO, Step: {}, JointDeg: {}'.format(step, ['{:.2f}'.format(pos) for pos in nicodeg_pos], end='\n'))
                        
                        spin_simulation(1)
                        step += 1
                        last_state = state
                        state = []

                        if linalg.norm(simdiff) <= JOINTACCURACY:
                            finished = True

                        if step > 350:
                            #failed += 1
                            #print('ANIMATION MODE FAILED - NEEDS DEBUGGGING')
                            
                            finished = True

                    finished = False

                else:
                    tic = time.perf_counter()
                    
                    for j in range(len(joint_indices)):
                        p.resetJointState(robot_id, joint_indices[j], ik_solution[j])
                    for j in range(len(joint_indices)):
                            state.append(p.getJointState(robot_id, joint_indices[j])[0])
                    last_state = state
                    simdiff = rad2deg(array(ik_solution)) - rad2deg(array(state))
                    state = []
                    toc = time.perf_counter()


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
                #time_response = check_response(robot, actuated_joints, joint_values, tic)
                #print('RealNICO response time: {:.2f}s'.format(time_response))

                #TimeRstat.append(time_response)
                
                # time.sleep(arg_dict['duration'])
                time_ex = check_execution(robot, actuated_joints, targetdeg, 3, arg_dict["verbose"])
                # time_ex= (arg_dict['duration'])
                # time.sleep(2)
                final_pos = get_real_joints(robot, actuated_joints)
                difference = array(targetdeg) - array(final_pos)
                if arg_dict["verbose"]:
                    print('RealNICO goal: {:.2f}s \n Error: {} \n Goal: {} \n Real: {}'.format(time_res,
                                                                        ['{:.2f}'.format(diff) for diff in difference],
                                                                        ['{:.2f}'.format(goal) for goal in targetdeg],
                                                                        ['{:.2f}'.format(real) for real in final_pos]))
                JointGstat.append(difference)
                TimeGstat.append(time_ex)
    except StopIteration:  
        pass

    # filename = 'sim_time_errors/sim_time_error_with_duration_' + str(arg_dict['duration']) + '.txt'
    # with open(filename, 'w') as f:
    #     for sim_time_error in sim_time_errors:
    #         f.write("%s\n" % sim_time_error)

    if arg_dict["save_statistics"]:
        filename = 'statistics/10_steps_position_target.txt'
        # filename = '../jurajgavura/BP/exp_random/random_sw_duration_3/random_position_target.txt'
        with open(filename, 'w') as f:
            for targetP in TargetPstat:
                f.write("%s\n" % ', '.join(map(str, round(targetP, 3))))
        
        filename = 'statistics/10_steps_joint_diffs.txt'
        # filename = '../jurajgavura/BP/exp_random/random_sw_duration_3/random_joint_diffs.txt'
        with open(filename, 'w') as f:
            for joint_diff in JointGstat:
                f.write("%s\n" % ' '.join(map(str, round(joint_diff, 3))))
        
        filename = 'statistics/10_steps_time_response.txt'
        # filename = '../jurajgavura/BP/exp_random/random_sw_duration_3/random_time_response.txt'
        with open(filename, 'w') as f:
            for timeR in TimeRstat:
                f.write("%s\n" % '{:.2f}'.format(timeR))

        filename = 'statistics/10_steps_time_execution.txt'
        # filename = '../jurajgavura/BP/exp_random/random_sw_duration_3/random_time_execution.txt'
        with open(filename, 'w') as f:
            for timeG in TimeGstat:
                f.write("%s\n" % '{:.2f}'.format(timeG))

    # Create a new figure
    # di = pd.DataFrame({'JointIstat': JointIstat})
    # dg = pd.DataFrame({'JointGstat': JointGstat})
    # dt = pd.DataFrame({'TimeIstat': TimeIstat, 'TimeGstat': TimeGstat})
    # dik = pd.DataFrame({'IKstat': IKstat})
    # Create boxplots
    # di.boxplot(column=['JointIstat'])

    # Show the figure
    # di.to_csv('i_joint_stat.csv', index=False)
    # dg.to_csv('g_joint_stat.csv', index=False)
    # dt.to_csv('time_stat.csv', index=False)
    # dik.to_csv('ik_stat.csv', index=False)
    # plt.savefig('boxplot.png')

    #input('Press enter to exit')
    # Create DataFrames from the
    
    filepath = './reachability/' + arg_dict['output_file']

    filename = filepath + '_summary.txt'
    with open(filename, 'w') as f:
        f.write("Meanposerror (x,y,z): ")
        f.write(str(meanposerror)+"\n")
        f.write("Meanorierror (a,b,c,d): ")
        f.write(str(meanorierror)+"\n")
        f.write("Meanjointerror (joints): ")
        f.write(str(meanjointerror)+"\n")
        f.write("Failed P/O/J/A: ")
        f.write(str(failpos) + "/" + str(failori) + "/" + str(failjoint) + "/" + str(i))
    filename = filepath + '_detail.txt'
    with open(filename, 'w') as f:
        f.write("Position, Position error, Orientation error, Joint error\n")
        for targetP in reacherror:
            rounded_targetP = [round(item, 3) for item in targetP]
            f.write("%s\n" % ', '.join(map(str, rounded_targetP)))



    save_image(filepath + '.png')

    p.disconnect()


if __name__ == "__main__":
    main()
