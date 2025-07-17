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
grasp_ori = [0,0,0] # Top grap [0,0,0] or side grasp [1.57,0,0]
pos = [0.35, -0.3, 0.19]

print("\n--- Executing Sequence with IK Move ---")
# Initial position
grasper.init_position(init_pos_l, init_ori, "left")
grasper.init_position(init_pos_r, init_ori, "right")
grasper.look_at([0.5,0,0.4]) 
grasper.point_gripper("left")
grasper.point_gripper("right")
for y in np.arange(0.3, -0.3, 0.06):
    grasper.move_arm([pos[0],y,pos[2]],grasp_ori,"right", autozpos=True, autoori=True)
    grasper.look_at([pos[0],y,pos[2]])
    #grasper.move_arm([pos[0],-y,pos[2]],grasp_ori,"left", autozpos=True, autoori=True)
    #grasper.look_at([pos[0],-y,pos[2]])
  

# Initial position
grasper.init_position(init_pos_l, init_ori, "left")
grasper.init_position(init_pos_r, init_ori, "right")
print("--- Sequence Finished ---\n")