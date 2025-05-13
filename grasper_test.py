import numpy as np
from grasper import Grasper

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

init_pos = [0.0, -0.3, 0.4]
init_ori = [0, -1.57, 0]
grasp_pos = [0.3, 0.3, 0.2]
grasp_ori = [0,0,0] # Top grap [0,0,0] or side grasp [1.57,0,0]
hand = "right"
object_z = 0.09
    
print("\n--- Executing Sequence with IK Move ---")
# Initial position
grasper.init_position(init_pos,init_ori, hand)
input("Press Enter to continue...")
for i in range(40, -180, -10):
    print(f"Moving gripper to {i} degrees")
    grasper.move_gripper(hand,i)
    input("Press Enter to continue...")
grasper.close_gripper(hand)
input("Press Enter to continue...")
grasper.open_finger("l_middlefingers_x")
input("Press Enter to continue...")
grasper.open_finger("l_indexfinger_x")
input("Press Enter to continue...")
grasper.move_arm(grasp_pos, grasp_ori, hand)
grasper.open_gripper(hand)
print("--- Sequence Finished ---\n")