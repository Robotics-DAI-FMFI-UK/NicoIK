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

init_pos = [0, 0.3, 0.5]
init_ori = [0, -1.57, 0]
grasp_ori = [0,0,0] # Top grap [0,0,0] or side grasp [1.57,0,0]
hand = "left"
object_z = 0.19
    
print("\n--- Executing Sequence with IK Move ---")
# Initial position
grasper.init_position(init_pos, init_ori, hand)
for i in range(30, 50, 5):

    # Pick object
    grasper.pick_object([i/100,0.15,object_z+0.001*i], grasp_ori, hand)
    # Place object
    grasper.place_object([i/100,0.3,object_z+0.001*i], grasp_ori, hand)

# Initial position
grasper.init_position([0, 0.3, 0.5], [0,-1.57,0], hand)

print("--- Sequence Finished ---\n")