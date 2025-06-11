import numpy as np
from grasper import Grasper
from sim_height_calculation import calculate_z
import random
import time
print("Initializing Grasper...")
try:
    grasper = Grasper(
        urdf_path="./nico_grasper.urdf",
        motor_config="./nico_humanoid_upper_rh7d_ukba.json",
        connect_robot=True,     # Connect to the real robot hardware
    )
    print("Grasper initialized successfully for real robot.")
except Exception as e:
    print(f"Error initializing Grasper for real robot: {e}")

init_pos_l = [0, 0.3, 0.5]
init_pos_r = [init_pos_l[0], -init_pos_l[1], init_pos_l[2]]
init_ori = [0, -1.57, 0]
grasp_ori = [0,0,0.0] # Top grap [0,0,0] or side grasp [1.57,0,0]
print("\n--- Executing Sequence with IK Move ---")
hand = "left"
# Initial position
grasper.move_both_arms (init_pos_r, init_ori)
#for x in np.arange(0.25, 0.45, 0.05):
#    for y in np.arange(0.3, 0.21, 0.05):
#        grasper.move_arm([x, y, 0.1], grasp_ori, hand)
#        #time.sleep(1)
    #grasper.move_both_arms (init_pos_r, init_ori)
#    time.sleep(1)
hand = "right" 
for x in np.arange(0.25, 0.45, 0.05):
    for y in np.arange(-0.3, 0.21, 0.05):
        grasper.move_arm([x, y, 0.1], grasp_ori, hand,autozpos = True,autoori = True)
        #time.sleep(1)
    grasper.move_both_arms (init_pos_r, init_ori)
    time.sleep(1)


    #object_z2 = calculate_z(goal2[0],goal2[1]) + 0.03
    #grasper.place_object([goal1[0],goal1[1],object_z1], grasp_ori, "right")
    #grasper.move_both_arms (init_pos_r, init_ori)
