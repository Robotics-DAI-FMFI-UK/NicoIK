import numpy as np
from grasper import Grasper

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
grasp_ori_r = [1.57,0,0] # Top grap [0,0,0] or side grasp [1.57,0,0]
grasp_ori_l = [-1.57,0,0]
hand = "left"
object_z = 0.13
    
print("\n--- Executing Sequence with IK Move ---")
# Initial position
grasper.init_position(init_pos_l, init_ori, "left")
grasper.init_position(init_pos_r, init_ori, "right")
for i in range(30, 35, 10):

    # Pick object
    grasper.pick_object([i/100,0.05,object_z+0.001*i], grasp_ori_l, "left")
    grasper.pick_object([i/100,-0.05,object_z+0.001*i], grasp_ori_r, "right")
    # Place object
    grasper.place_object([i/100,init_pos_l[1],object_z+0.001*i], grasp_ori_l, "left")
    grasper.place_object([i/100,init_pos_r[1],object_z+0.001*i], grasp_ori_r, "right")


# Initial position
grasper.init_position(init_pos_l, init_ori, "left")
grasper.init_position(init_pos_r, init_ori, "right")
print("--- Sequence Finished ---\n")