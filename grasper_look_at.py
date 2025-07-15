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
#grasper.init_position(init_pos_l, init_ori, "left")
#grasper.init_position(init_pos_r, init_ori, "right")
for z in np.arange(0, 1, 0.2):
    pos = [0.5, 0, 0]
    print("Looking at:", pos)
    grasper.look_at(pos)
  

# Initial position
#grasper.init_position(init_pos_l, init_ori, "left")
#grasper.init_position(init_pos_r, init_ori, "right")
print("--- Sequence Finished ---\n")