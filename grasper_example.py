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
# object_z = 0.07
    
print("\n--- Executing Sequence with IK Move ---")
# Initial position
grasper.init_position([0, -0.3, 0.5], [0,-1.57,0], hand)
for i in range(30, 50, 5):

    # # Pick object
    # grasper.pick_object([i/100,-0.15,object_z+0.001*i], grasp_ori, hand)
    # # Place object
    # grasper.place_object([i/100,-0.3,object_z+0.001*i], grasp_ori, hand)

    # Calculate z for picking up the object
    object_z = calculate_z(i/100, -0.15) + 0.04
    print(f"Calculated z for pick at x={i/100}, y=-0.15: {object_z}")
    # Pick object
    grasper.pick_object([i/100,-0.15,object_z], grasp_ori, hand)

    # Calculate z for placing the object
    object_z = calculate_z(i/100, -0.3) + 0.05
    print(f"Calculated z for placing at x={i/100}, y=-0.3: {object_z}")
    # Place object
    grasper.place_object([i/100,-0.3,object_z], grasp_ori, hand)

# Initial position
grasper.init_position([0, -0.3, 0.5], [0,-1.57,0], hand)

print("--- Sequence Finished ---\n")