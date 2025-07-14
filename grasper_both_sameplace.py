import numpy as np
from grasper import Grasper
from sim_height_calculation import calculate_z

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
grasp_ori = [0,0,0.7] # Top grap [0,0,0] or side grasp [1.57,0,0]
goal1= [0.27, 0.0]
goal2 = [0.4, 0.0]
    
print("\n--- Executing Sequence with IK Move ---")
# Initial position
grasper.init_position(init_pos_l, init_ori, "left")
grasper.init_position(init_pos_r, init_ori, "right")
while True:

    # Left
    object_z1 = calculate_z(goal1[0],goal1[1]) + 0.02
    grasper.pick_object([goal1[0],goal1[1],object_z1], grasp_ori, "left")
    object_z2 = calculate_z(goal2[0],goal2[1]) + 0.02
    grasper.place_object([goal2[0],goal2[1],object_z2], grasp_ori, "left")
    grasper.init_position(init_pos_l, init_ori, "left")
    # Right
    object_z1 = calculate_z(goal1[0],goal1[1]) + 0.03
    grasper.pick_object([goal2[0],goal2[1],object_z2], grasp_ori, "right")
    object_z2 = calculate_z(goal2[0],goal2[1]) + 0.03
    grasper.place_object([goal1[0],goal1[1],object_z1], grasp_ori, "right")
    grasper.init_position(init_pos_r, init_ori, "right")


# Initial position
grasper.init_position(init_pos_l, init_ori, "left")
grasper.init_position(init_pos_r, init_ori, "right")
print("--- Sequence Finished ---\n")