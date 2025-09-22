import pybullet as p
import pybullet_data
import argparse
import time
import numpy as np
from numpy import rad2deg, deg2rad, set_printoptions, array, linalg, round, any, mean
import os
import json
import commentjson
import copy

# Stable-Baselines3 core imports
HAVE_SB3 = True
try:
    from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.utils import set_random_seed
    from stable_baselines3 import A2C as A2C_P, SAC as SAC_P, TD3 as TD3_P, PPO as PPO_SB3
except Exception as e:
    HAVE_SB3 = False
    print(f"Warning: stable_baselines3 not available: {e}")

# Default PPO loader uses SB3 if available
PPO_Loader = PPO_SB3 if 'PPO_SB3' in globals() else None

# Optional myGym extensions (do not fail if missing)
MultiPPOSB3 = None
try:
    from myGym.stable_baselines_mygym.ppoSB3 import PPO as PPO_P
    PPO_Loader = PPO_P if PPO_Loader is not None else PPO_P
    print("Info: Using myGym PPO wrapper")
except Exception:
    if PPO_Loader is not None:
        print("Info: myGym PPO wrapper not available, falling back to SB3 PPO")
try:
    from myGym.stable_baselines_mygym.multi_ppo_SB3 import MultiPPOSB3 as MultiPPOSB3
    print("Info: myGym MultiPPO available")
except Exception:
    MultiPPOSB3 = None

from grasper import Grasper

class NeuroGrasper:
    def __init__(self, urdf_path, model_path=None):
        self.urdf_path = urdf_path
        # Resolve default model path and support directory inputs
        default_model = os.path.join('.', 'model', 'best_model.zip')
        if model_path is None:
            self.model_path = default_model
        else:
            self.model_path = model_path
            if os.path.isdir(self.model_path):
                self.model_path = os.path.join(self.model_path, 'best_model.zip')
        self.model = None
        self.use_neural_network = False
        self.neural_trajectory = []
        self.trajectory_index = 0
        self.executing_trajectory = False
        self.live_nn = False  # stream predictions each frame when True
        # Track current goal pose for reporting
        self.current_goal_pos = None
        self.current_goal_quat = None
        
        # Initialize PyBullet
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self._load_model()
        
    def _load_model(self):
        """Load pretrained model if path is provided"""
        print(f"Attempting to load model from: {self.model_path}")
        if not HAVE_SB3:
            print("Error: stable_baselines3 is required to load a model. Please install it.")
            return
        if self.model_path and os.path.exists(self.model_path):
            try:
                # Load model configuration (optional)
                config_path = os.path.join(os.path.dirname(self.model_path), "train.json")
                if os.path.exists(config_path):
                    with open(config_path, "r") as f:
                        self.model_config = commentjson.load(f)
                    print(f"Loaded model configuration from {config_path}")
                else:
                    self.model_config = {}
                
                # Determine algorithm and choose loader
                algo = self.model_config.get("algo", "ppo").lower()
                is_zip = str(self.model_path).lower().endswith('.zip')
                
                if algo == "ppo":
                    if is_zip:
                        # Force SB3 loader for .zip files
                        self.model = PPO_SB3.load(self.model_path, device="cpu")
                    else:
                        if PPO_Loader is None:
                            raise RuntimeError("No PPO loader available")
                        self.model = PPO_Loader.load(self.model_path, device="cpu")
                elif algo == "multippo":
                    if is_zip:
                        print("Warning: MultiPPO with .zip provided; falling back to SB3 PPO loader")
                        self.model = PPO_SB3.load(self.model_path, device="cpu")
                    else:
                        if MultiPPOSB3 is not None:
                            self.model = MultiPPOSB3.load(self.model_path, device="cpu")
                        else:
                            print("Warning: MultiPPO not available; falling back to SB3 PPO loader")
                            self.model = PPO_SB3.load(self.model_path, device="cpu")
                elif algo == "sac":
                    self.model = SAC_P.load(self.model_path, device="cpu")
                elif algo == "td3":
                    self.model = TD3_P.load(self.model_path, device="cpu")
                elif algo == "a2c":
                    self.model = A2C_P.load(self.model_path, device="cpu")
                else:
                    print(f"Unknown algorithm '{algo}', attempting SB3 PPO loader")
                    self.model = PPO_SB3.load(self.model_path, device="cpu")
                    
                print(f"Successfully loaded model from {self.model_path}")
                # Print observation/action space info
                try:
                    obs_space = getattr(self.model, 'observation_space', None)
                    act_space = getattr(self.model, 'action_space', None)
                    obs_shape = getattr(obs_space, 'shape', None)
                    act_shape = getattr(act_space, 'shape', None)
                    obs_dim = int(np.prod(obs_shape)) if obs_shape is not None else None
                    act_dim = int(np.prod(act_shape)) if act_shape is not None else None
                    print(f"Model spaces -> obs_shape={obs_shape}, act_shape={act_shape}, obs_dim={obs_dim}, act_dim={act_dim}")
                except Exception as e:
                    print(f"Could not read model spaces: {e}")
                
                self.use_neural_network = True
                
            except Exception as e:
                print(f"Error loading model: {e}")
                self.model = None
        else:
            print(f"No valid model path found. Expected at: {self.model_path}")

    def get_look_at_quaternion(self, eye_pos, target_pos):
        """Calculate quaternion to orient object at eye_pos to look at target_pos"""
        eye_pos = np.array(eye_pos)
        target_pos = np.array(target_pos)
        
        direction = target_pos - eye_pos
        
        if np.linalg.norm(direction) < 1e-6:
            return [0, 0, 0, 1]
            
        yaw = np.arctan2(direction[1], direction[0])
        horizontal_dist = np.sqrt(direction[0]**2 + direction[1]**2)
        pitch = np.arctan2(-direction[2], horizontal_dist)
        roll = 0
        
        return p.getQuaternionFromEuler([roll, pitch, yaw])

    def rad2nicodeg(self, nicojoints, rads):
        """Convert radians to Nico-specific degrees"""
        if isinstance(nicojoints, str):
            nicojoints = [nicojoints]
        if isinstance(rads, (int, float)):
            rads = [rads]

        nicodegrees_dict = {}
        for nicojoint, rad in zip(nicojoints, rads):
            if isinstance(rad, (int, float)):
                if nicojoint == 'r_wrist_z':
                    nicodegree = rad2deg(rad) * 2
                elif nicojoint == 'r_wrist_x':
                    nicodegree = rad2deg(rad) * 4
                else:
                    nicodegree = rad2deg(rad)
                nicodegrees_dict[nicojoint] = nicodegree
            else:
                print(f"Warning: Non-numeric radian value '{rad}' for joint '{nicojoint}'. Skipping conversion.")

        return nicodegrees_dict

    def calculate_ik(self, robot_id, end_effector_index, target_pos, orientation):
        """Calculate inverse kinematics solution for the robot arm"""
        ik_solution = p.calculateInverseKinematics(
            robot_id,
            end_effector_index,
            target_pos,
            targetOrientation=orientation,
            maxNumIterations=100,
            residualThreshold=0.001
        )
        return ik_solution

    def apply_joint_solution(self, robot_id, joint_solution, joint_idxs):
        """Apply joint solution to the robot joints"""
        joint_values = []
        for index, joint_idx in enumerate(joint_idxs):
            if index < len(joint_solution):
                joint_pos = joint_solution[index]
                p.setJointMotorControl2(
                    bodyIndex=robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=joint_pos
                )
                joint_values.append(f"{p.getJointInfo(robot_id, joint_idx)[1].decode('utf-8')}: {joint_pos:.4f} rad ({joint_pos*57.2958:.2f}°)")
        return joint_values

    def get_robot_observation(self, robot_id, joint_idxs, target_pos):
        """Get current robot state as observation for neural network"""
        # Get current joint positions
        joint_positions = []
        for joint_idx in joint_idxs:
            joint_state = p.getJointState(robot_id, joint_idx)
            joint_positions.append(joint_state[0])  # Joint position
        
        # Get end effector position
        ee_state = p.getLinkState(robot_id, self.end_effector_index)
        ee_pos = list(ee_state[0])
        
        # Combine observation: [joint_positions, ee_position, target_position]
        observation = joint_positions + ee_pos + list(target_pos)
        return np.array(observation, dtype=np.float32)

    def get_expected_obs_dim(self):
        """Return expected observation dimension from the loaded model if available."""
        try:
            shape = self.model.observation_space.shape
            if isinstance(shape, tuple) and len(shape) > 0:
                return int(np.prod(shape))
        except Exception:
            pass
        return None

    def get_current_joint_positions(self, robot_id, joint_idxs):
        """Return current joint positions as np.array."""
        vals = []
        for j in joint_idxs:
            vals.append(p.getJointState(robot_id, j)[0])
        return np.asarray(vals, dtype=np.float32)

    def get_current_ee_pos(self, robot_id):
        """Return current end-effector position as np.array (3,)."""
        return np.asarray(p.getLinkState(robot_id, self.end_effector_index)[0], dtype=np.float32)

    def print_observation_info(self, robot_id, joint_idxs, target_pos, target_orientation):
        """Print observation structure and sizing details for EE/Goal pose based observation."""
        ee_state = p.getLinkState(robot_id, self.end_effector_index)
        ee_pos = np.asarray(ee_state[0], dtype=np.float32)
        # ee_quat
        ee_quat = np.asarray(ee_state[1], dtype=np.float32)
        tgt_pos = np.asarray(target_pos, dtype=np.float32)
        tgt_quat = np.asarray(target_orientation, dtype=np.float32)
        ee_extra = np.asarray([ee_pos[0], ee_pos[2], ee_pos[1]], dtype=np.float32)
        ee_pos_len = ee_pos.size
        ee_quat_len = ee_quat.size
        tgt_pos_len = tgt_pos.size
        tgt_quat_len = tgt_quat.size
        extra_len = ee_extra.size
        base_len = ee_pos_len + ee_quat_len + tgt_pos_len + tgt_quat_len + extra_len
        exp = self.get_expected_obs_dim()
        pad = 0
        trunc = 0
        if exp is not None:
            if base_len < exp:
                pad = exp - base_len
            elif base_len > exp:
                trunc = base_len - exp
        print(
            f"Observation(parts): ee_pos={ee_pos_len}, ee_quat={ee_quat_len}, tgt_pos={tgt_pos_len}, tgt_quat={tgt_quat_len}, extra(ee_x,ee_z,ee_y)={extra_len}, total={base_len}, expected={exp}, pad={pad}, trunc={trunc}"
        )

    def build_model_observation(self, robot_id, joint_idxs, target_pos, target_orientation):
        """Build observation: [ee_pos(3), ee_quat(4), goal_pos(3), goal_quat(4), ee_pos_extra(x,z,y)]"""
        ee_state = p.getLinkState(robot_id, self.end_effector_index)
        ee_pos = list(ee_state[0])
        ee_quat = list(ee_state[1])
        goal_pos = list(target_pos)
        goal_quat = list(target_orientation)
        ee_extra = [ee_pos[0], ee_pos[2], ee_pos[1]]
        base = ee_pos + ee_quat + goal_pos + goal_quat + ee_extra
        obs = np.asarray(base, dtype=np.float32)
        exp = self.get_expected_obs_dim()
        if exp is None:
            return obs
        if obs.size < exp:
            obs = np.pad(obs, (0, exp - obs.size), mode='constant')
        elif obs.size > exp:
            obs = obs[:exp]
        return obs

    def calculate_neural_trajectory(self, robot_id, joint_idxs, target_pos, target_orientation, steps=200):
        """Use neural network to calculate trajectory to target. If shape mismatch occurs, return []."""
        if self.model is None:
            print("No model loaded! Falling back to IK.")
            return []
        # Print observation structure and start point
        self.print_observation_info(robot_id, joint_idxs, target_pos, target_orientation)
        start_joints = self.get_current_joint_positions(robot_id, joint_idxs)
        start_ee = self.get_current_ee_pos(robot_id)
        print(f"Trajectory start (joints): {np.round(start_joints, 4).tolist()}  ee: {np.round(start_ee, 4).tolist()}")
        trajectory = []
        try:
            for step in range(steps):
                current_obs = self.build_model_observation(robot_id, joint_idxs, target_pos, target_orientation)
                action, _ = self.model.predict(current_obs, deterministic=True)
                # Map action to joint positions
                if hasattr(self.model, 'action_space') and hasattr(self.model.action_space, 'shape') and self.model.action_space.shape is not None:
                    act_dim = int(np.prod(self.model.action_space.shape))
                else:
                    act_dim = len(action)
                use_dim = min(len(joint_idxs), act_dim, len(action))
                if use_dim <= 0:
                    print("Warning: action dimension invalid; aborting trajectory calc")
                    break
                joint_positions = np.zeros(len(joint_idxs), dtype=np.float32)
                joint_positions[:use_dim] = action[:use_dim]
                trajectory.append(joint_positions.copy())
                # Early stop if close to target
                ee_state = p.getLinkState(robot_id, self.end_effector_index)
                ee_pos = np.array(ee_state[0])
                target_pos_array = np.array(target_pos)
                if np.linalg.norm(ee_pos - target_pos_array) < 0.05:
                    print(f"Target condition met at step {step+1}")
                    break
        except Exception as e:
            print(f"Neural trajectory precompute failed: {e}")
            return []
        if len(trajectory) > 0:
            end_joints = trajectory[-1]
            print(f"Trajectory end (joints): {np.round(end_joints, 4).tolist()}")
        else:
            print("Trajectory end (joints): []")
        print(f"Generated neural trajectory with {len(trajectory)} steps")
        return trajectory

    def find_gripper_joints(self, robot_id):
        """Find all joints related to the right fingers"""
        num_joints = p.getNumJoints(robot_id)
        gripper_idxs = []
        for joint_idx in range(num_joints):
            joint_info = p.getJointInfo(robot_id, joint_idx)
            joint_name = joint_info[1].decode("utf-8")
            if 'right_' in joint_name and 'finger' in joint_name:
                gripper_idxs.append(joint_idx)
        return gripper_idxs

    def get_controllable_arm_joints(self, robot_id, num_joints):
        """Get joint information for arm control (non-fixed joints), excluding gripper joints"""
        joint_idxs = []
        joint_names = []
        for joint_idx in range(num_joints):
            joint_info = p.getJointInfo(robot_id, joint_idx)
            joint_name = joint_info[1].decode("utf-8")
            joint_type = joint_info[2]
            
            if joint_type != p.JOINT_FIXED:
                if 'right_' not in joint_name or 'finger' not in joint_name: 
                    joint_idxs.append(joint_idx)
                    joint_names.append(joint_name)
        
        return joint_idxs, joint_names

    def run_gui(self, scene_path=None, texture_path=None, init_pos=[0.0, -0.3, 0.5], goal_pos=[0.3, -0.3, 0.07]):
        """Run the main GUI simulation loop"""
        # Load scene
        if scene_path:
            scene_uid = p.loadURDF(scene_path, useFixedBase=True)
            if texture_path and os.path.exists(texture_path):
                texture_id = p.loadTexture(texture_path)
                p.changeVisualShape(scene_uid, -1, rgbaColor=[1, 1, 1, 1], textureUniqueId=texture_id)
            p.resetBasePositionAndOrientation(scene_uid, [0.0, 0.0, -0.73], p.getQuaternionFromEuler([0, 0, -np.pi / 2]))

        # Load robot
        try:
            robot_id = p.loadURDF(self.urdf_path, useFixedBase=True)
        except:
            print(f"Error: Failed to load URDF file '{self.urdf_path}'")
            return

        # Find end effector links
        num_joints = p.getNumJoints(robot_id)
        end_effector_index_r = -1
        end_effector_index_l = -1
        end_effector_index_h = -1
        
        for joint_idx in range(num_joints):
            joint_info = p.getJointInfo(robot_id, joint_idx)
            link_name = joint_info[12].decode("utf-8")
            if link_name == 'endeffectol':
                end_effector_index_l = joint_idx
            elif link_name == 'endeffector':
                end_effector_index_r = joint_idx
            elif link_name == 'head':
                end_effector_index_h = joint_idx
        
        self.end_effector_index = end_effector_index_r
        mode = "right"

        # Get joint information
        joint_idxs, joint_names = self.get_controllable_arm_joints(robot_id, num_joints)
        gripper_idxs = self.find_gripper_joints(robot_id)

        # Create visual boxes
        box_size = 0.03
        box_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[box_size/2]*3, rgbaColor=[1, 0.0, 0.0, 0.8]),
            basePosition=goal_pos
        )

        box2_size = 0.03
        box2_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[box2_size/2]*3, rgbaColor=[0, 0, 1, 0.7]),
            basePosition=init_pos
        )

        # Create sliders
        x2_slider = p.addUserDebugParameter("Init X", -0.2, 1.0, init_pos[0])
        y2_slider = p.addUserDebugParameter("Init Y", -0.6, 0.6, init_pos[1])
        z2_slider = p.addUserDebugParameter("Init Z", 0.3, 1.5, init_pos[2])
        init_roll_slider = p.addUserDebugParameter("Init Roll", -np.pi, np.pi, 0)
        init_pitch_slider = p.addUserDebugParameter("Init Pitch", -np.pi, np.pi, -np.pi/2)
        init_yaw_slider = p.addUserDebugParameter("Init Yaw", -np.pi, np.pi, 0) 
        
        x_slider = p.addUserDebugParameter("Goal X", 0.0, 0.6, goal_pos[0])
        y_slider = p.addUserDebugParameter("Goal Y", -0.5, 0.5, goal_pos[1])
        z_slider = p.addUserDebugParameter("Goal Z", 0.0, 0.7, goal_pos[2])
        roll_slider = p.addUserDebugParameter("Goal_Roll", -np.pi, np.pi, 0)
        pitch_slider = p.addUserDebugParameter("Goal_Pitch", -np.pi, np.pi, 0)
        yaw_slider = p.addUserDebugParameter("Goal_Yaw", -np.pi, np.pi, 0)

        # Set up simulation
        p.setRealTimeSimulation(1)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.95,
            cameraYaw=90,
            cameraPitch=-15,
            cameraTargetPosition=[0, 0, 0.3]
        )

        # Initialize variables
        orientation_line_id = None
        use_second_target = True
        
        print("Controls:")
        print("  X - Toggle between Neural Network and IK mode / Switch target")
        print("  N - Toggle Neural Network mode")
        print("  C - Close gripper")
        print("  D - Open gripper")
        print("  B - Right arm mode")
        print("  H - Head mode")
        print("  V - Left arm mode")

        try:
            while True:
                keys = p.getKeyboardEvents()
                
                # Handle key presses
                if ord('x') in keys and keys[ord('x')] & p.KEY_WAS_TRIGGERED:
                    if self.use_neural_network:
                        # Calculate neural trajectory; if fails, switch to live streaming
                        target_pos = [
                            p.readUserDebugParameter(x_slider),
                            p.readUserDebugParameter(y_slider), 
                            p.readUserDebugParameter(z_slider)
                        ]
                        roll = p.readUserDebugParameter(roll_slider)
                        pitch = p.readUserDebugParameter(pitch_slider)
                        yaw = p.readUserDebugParameter(yaw_slider)
                        target_orientation = p.getQuaternionFromEuler([roll, pitch, yaw])
                        # Store current goal for reporting
                        self.current_goal_pos = target_pos
                        self.current_goal_quat = target_orientation
                        print("Calculating neural trajectory...")
                        traj = self.calculate_neural_trajectory(
                            robot_id, joint_idxs, target_pos, target_orientation
                        )
                        if len(traj) == 0:
                            print("Switching to live NN control (streamed predictions)")
                            # Also print observation info for live mode
                            self.print_observation_info(robot_id, joint_idxs, target_pos, target_orientation)
                            self.live_nn = True
                        else:
                            self.neural_trajectory = traj
                            self.trajectory_index = 0
                            self.live_nn = False
                            print(f"Starting trajectory execution with {len(self.neural_trajectory)} steps")
                        self.executing_trajectory = True
                    else:
                        use_second_target = not use_second_target
                        print(f"Switched target source: {'Initial state' if use_second_target else 'Goal State'}")

                if ord('n') in keys and keys[ord('n')] & p.KEY_WAS_TRIGGERED:
                    self.use_neural_network = not self.use_neural_network
                    print(f"Neural Network mode: {'ON' if self.use_neural_network else 'OFF'}")

                if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
                    print("Closing gripper")
                    gripper_solution = [-2.0] * len(gripper_idxs)
                    self.apply_joint_solution(robot_id, gripper_solution, gripper_idxs)

                if ord('d') in keys and keys[ord('d')] & p.KEY_WAS_TRIGGERED:
                    print("Opening gripper")
                    gripper_solution = [0.0] * len(gripper_idxs)
                    self.apply_joint_solution(robot_id, gripper_solution, gripper_idxs)

                if ord('b') in keys and keys[ord('b')] & p.KEY_WAS_TRIGGERED:
                    self.end_effector_index = end_effector_index_r
                    print("Switched to right end effector")
                    mode = "right"

                if ord('h') in keys and keys[ord('h')] & p.KEY_WAS_TRIGGERED:
                    self.end_effector_index = end_effector_index_h
                    print("Switched to head end effector")
                    mode = "head"

                if ord('v') in keys and keys[ord('v')] & p.KEY_WAS_TRIGGERED:
                    self.end_effector_index = end_effector_index_l
                    print("Switched to left end effector")
                    mode = "left"

                # Execute neural trajectory or use IK
                if self.executing_trajectory and self.use_neural_network:
                    if self.live_nn:
                        # Stream predictions each frame
                        target_pos_live = [
                            p.readUserDebugParameter(x_slider),
                            p.readUserDebugParameter(y_slider), 
                            p.readUserDebugParameter(z_slider)
                        ]
                        roll_l = p.readUserDebugParameter(roll_slider)
                        pitch_l = p.readUserDebugParameter(pitch_slider)
                        yaw_l = p.readUserDebugParameter(yaw_slider)
                        target_ori_live = p.getQuaternionFromEuler([roll_l, pitch_l, yaw_l])
                        obs = self.build_model_observation(robot_id, joint_idxs, target_pos_live, target_ori_live)
                        try:
                            action, _ = self.model.predict(obs, deterministic=True)
                            use_dim = min(len(joint_idxs), len(action))
                            joint_positions = np.zeros(len(joint_idxs), dtype=np.float32)
                            joint_positions[:use_dim] = action[:use_dim]
                            self.apply_joint_solution(robot_id, joint_positions, joint_idxs)
                        except Exception as e:
                            print(f"Live NN predict failed: {e}")
                            self.executing_trajectory = False
                            self.live_nn = False
                    else:
                        if self.trajectory_index < len(self.neural_trajectory):
                            joint_positions = self.neural_trajectory[self.trajectory_index]
                            self.apply_joint_solution(robot_id, joint_positions, joint_idxs)
                            self.trajectory_index += 1
                        else:
                            # Report end positions before resetting
                            try:
                                ee_pos_end = np.asarray(p.getLinkState(robot_id, self.end_effector_index)[0], dtype=np.float32)
                                goal_pos_arr = np.asarray(self.current_goal_pos if self.current_goal_pos is not None else [0,0,0], dtype=np.float32)
                                dist = float(np.linalg.norm(ee_pos_end - goal_pos_arr))
                                print(f"Neural trajectory execution completed. Goal pos: {np.round(goal_pos_arr,4).tolist()}  EE pos: {np.round(ee_pos_end,4).tolist()}  dist: {dist:.4f}")
                            except Exception:
                                print("Neural trajectory execution completed")
                            self.executing_trajectory = False
                            self.neural_trajectory = []
                            self.trajectory_index = 0
                elif not self.executing_trajectory:
                    # Use IK control when not executing neural trajectory
                    if use_second_target:
                        target_pos = [
                            p.readUserDebugParameter(x2_slider),
                            p.readUserDebugParameter(y2_slider),
                            p.readUserDebugParameter(z2_slider)
                        ]
                        roll = p.readUserDebugParameter(init_roll_slider)
                        pitch = p.readUserDebugParameter(init_pitch_slider)
                        yaw = p.readUserDebugParameter(init_yaw_slider)
                        target_orientation_quat = p.getQuaternionFromEuler([roll, pitch, yaw])
                        p.resetBasePositionAndOrientation(box2_id, target_pos, [0,0,0,1])
                        box1_pos_ref = [p.readUserDebugParameter(x_slider), p.readUserDebugParameter(y_slider), p.readUserDebugParameter(z_slider)]
                        p.resetBasePositionAndOrientation(box_id, box1_pos_ref, [0,0,0,1])
                    else:
                        target_pos = [
                            p.readUserDebugParameter(x_slider),
                            p.readUserDebugParameter(y_slider), 
                            p.readUserDebugParameter(z_slider)
                        ]
                        roll = p.readUserDebugParameter(roll_slider)
                        pitch = p.readUserDebugParameter(pitch_slider)
                        yaw = p.readUserDebugParameter(yaw_slider)
                        target_orientation_quat = p.getQuaternionFromEuler([roll, pitch, yaw])
                        p.resetBasePositionAndOrientation(box_id, target_pos, [0,0,0,1])
                        box2_pos_ref = [p.readUserDebugParameter(x2_slider), p.readUserDebugParameter(y2_slider), p.readUserDebugParameter(z2_slider)]
                        p.resetBasePositionAndOrientation(box2_id, box2_pos_ref, [0,0,0,1])

                    # Apply IK
                    if mode == "head":
                        head_link_state = p.getLinkState(robot_id, end_effector_index_h)
                        head_pos = head_link_state[0]
                        target_orientation_quat = self.get_look_at_quaternion(head_pos, target_pos)
                        ik_solution = p.calculateInverseKinematics(robot_id, self.end_effector_index, target_pos, target_orientation_quat)
                    else:
                        ik_solution = p.calculateInverseKinematics(robot_id, self.end_effector_index, target_pos, target_orientation_quat)
                    
                    self.apply_joint_solution(robot_id, ik_solution, joint_idxs)

                # Visualization
                link_state = p.getLinkState(robot_id, self.end_effector_index)
                ee_pos = link_state[0]
                
                # Remove previous orientation line
                if orientation_line_id is not None:
                    p.removeUserDebugItem(orientation_line_id)

                # Draw orientation line
                if not self.executing_trajectory:
                    line_length = 0.05
                    if use_second_target:
                        roll = p.readUserDebugParameter(init_roll_slider)
                        pitch = p.readUserDebugParameter(init_pitch_slider)
                        yaw = p.readUserDebugParameter(init_yaw_slider)
                    else:
                        roll = p.readUserDebugParameter(roll_slider)
                        pitch = p.readUserDebugParameter(pitch_slider)
                        yaw = p.readUserDebugParameter(yaw_slider)
                    
                    target_orientation_quat = p.getQuaternionFromEuler([roll, pitch, yaw])
                    rot_matrix_desired = p.getMatrixFromQuaternion(target_orientation_quat)
                    z_axis_direction_desired = [rot_matrix_desired[2], rot_matrix_desired[5], rot_matrix_desired[8]]
                    line_end_desired = [ee_pos[0] - z_axis_direction_desired[0] * line_length,
                                        ee_pos[1] - z_axis_direction_desired[1] * line_length,
                                        ee_pos[2] - z_axis_direction_desired[2] * line_length]
                    
                    color = [1, 0, 0] if not self.use_neural_network else [0, 1, 0]
                    orientation_line_id = p.addUserDebugLine(ee_pos, line_end_desired, color, 5)

                time.sleep(0.01)
                
        except KeyboardInterrupt:
            p.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Nico Robot Neural Grasping Control")
    parser.add_argument("--urdf", type=str, default="./urdf/nico_grasper.urdf", help="Path to the robot URDF file.")
    # Default to ./model/best_model.zip
    parser.add_argument("--model", type=str, default="./model/best_model.zip", help="Path to pretrained model .zip (default ./model/best_model.zip)")
    parser.add_argument("--scene", type=str, default="./urdf/table_nico.urdf", help="Path to workspace scene object.")
    parser.add_argument("--texture", type=str, default="./urdf/textures/table.jpg", help="Path to the texture of scene object.")
    parser.add_argument("-i", "--init_pos", nargs=3, type=float, default=[0.0, -0.3, 0.5], help="Initial position for the robot end effector.")
    parser.add_argument("-g", "--goal_pos", nargs=3, type=float, default=[0.3, -0.3, 0.07], help="Goal position for the robot end effector.")
    
    args = parser.parse_args()

    print("Initializing NeuroGrasper...")
    neurograsper = NeuroGrasper(args.urdf, args.model)
    
    print("Starting GUI simulation...")
    neurograsper.run_gui(
        scene_path=args.scene,
        texture_path=args.texture,
        init_pos=args.init_pos,
        goal_pos=args.goal_pos
    )

if __name__ == "__main__":
    main()