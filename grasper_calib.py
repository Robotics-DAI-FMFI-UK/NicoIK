import numpy as np
from grasper import Grasper
from sim_height_calculation import calculate_z

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

init_pos = [0, -0.3, 0.5]
init_ori = [0, -1.57, 0]
grasp_ori = [0,0,0] # Top grap [0,0,0] or side grasp [1.57,0,0]
hand = "right"
    
print("\n--- Executing Sequence with IK Move ---")
# Initial position
grasper.init_position([0, -0.3, 0.5], [0,-1.57,0], hand)

for i in range(5):
    object_x, object_y, object_z = 0.35, 0.20, 0.11

    # Calculate z for picking up the object
    # object_z = 0.125
    print(f"Calculated z for pick at x={object_x}, y={object_y}: {object_z}")
    # Pick object
    grasper.pick_object([object_x,object_y,object_z], grasp_ori, hand)

    # grasper.move_arm([object_x,object_y,object_z], grasp_ori, hand,autozpos = True,autoori = True)

    # Place object
    # grasper.place_object([object_x,object_y,object_z], grasp_ori, hand)

    # Initial position
    grasper.init_position([0, -0.3, 0.5], [0,-1.57,0], hand)


print("--- Sequence Finished ---\n")