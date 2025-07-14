import numpy as np
from grasper import Grasper
from sim_height_calculation import calculate_z
import random
import time
print("Initializing Grasper...")
try:
    grasper = Grasper(
        urdf_path="./urdf/nico_grasper.urdf",
        motor_config="./nico_humanoid_upper_rh7d_ukba.json",
        connect_robot=True,     # Connect to the real robot hardware
    )
    print("Grasper initialized successfully for real robot.")
except Exception as e:
    print(f"Error initializing Grasper for real robot: {e}")

init_pos_l = [0, 0.3, 0.5]
init_pos_r = [init_pos_l[0], -init_pos_l[1], init_pos_l[2]]
init_ori = [0, -1.57, 0]
grasp_ori = [1.57,0,0.0] # Top grap [0,0,0] or side grasp [1.57,0,0]
grasp_ori2 = [2.5,0,0.0]
grasp_ori3 = [2,0,0.0]
goal= [0.4, -0.02]
    
print("\n--- Executing Sequence with IK Move ---")
# Initial position
grasper.move_both_arms (init_pos_r, init_ori)
while True:
    grasper.move_both_arms([0.4 ,-.2, 0.06], grasp_ori)
    time.sleep(1)
    grasper.move_both_arms([0.3 ,-.01, 0.09], grasp_ori)
    time.sleep(1)
    grasper.move_both_arms([0.3 ,-.02, 0.3], grasp_ori2)
    time.sleep(1)
    grasper.move_both_arms([0.3 ,-.07, 0.3], grasp_ori3)
    time.sleep(1)


    #object_z2 = calculate_z(goal2[0],goal2[1]) + 0.03
    #grasper.place_object([goal1[0],goal1[1],object_z1], grasp_ori, "right")
    #grasper.move_both_arms (init_pos_r, init_ori)
