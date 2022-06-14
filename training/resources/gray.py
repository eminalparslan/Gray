from this import d
from time import time
import numpy as np
import pybullet as p
from math import pi
from collections import deque
import os

from training.env_config import *


class Gray:
    def __init__(self):
        urdf_filepath = os.path.join(os.path.dirname(__file__), 'gray.urdf')
        base_pos = [0, 0, 0.2]
        base_ori = p.getQuaternionFromEuler([pi/2, 0, 0]) # gray starts on its side, so we need to rotate it 90 degrees
        flags = p.URDF_USE_SELF_COLLISION
        self.gray_id = p.loadURDF(urdf_filepath, base_pos, base_ori, useFixedBase=False, flags=flags)

        self.curr_pos = (0, 0, 0)
        # position memory (last second)
        self.memory_size = int(1 / TIMESTEP)
        self.joint_pos_memory = deque(maxlen=self.memory_size)
        self.pos_memory = deque(maxlen=self.memory_size)

        for joint in range(p.getNumJoints(self.gray_id)):
            p.changeDynamics(self.gray_id, joint, linearDamping=0, angularDamping=0)
        #for link in LEG_LINK_INDICES:
        #    p.changeDynamics(self.gray_id, link, lateralFriction=1.0)
        #    p.changeDynamics(self.gray_id, link, rollingFriction=1.0)
        #    p.changeDynamics(self.gray_id, link, spinningFriction=1.0)

    def apply_action(self, actions):
        deltas = np.array([STEP_SIZES[action] for action in actions])
        current_pos = np.array([state[0] for state in p.getJointStates(self.gray_id, JOINT_INDICES)])
        # limit the range of movement
        new_pos = np.clip(current_pos + deltas, -0.2*pi, 0.2*pi)
        self.joint_pos_memory.append(new_pos)
        p.setJointMotorControlArray(self.gray_id, JOINT_INDICES, p.POSITION_CONTROL, new_pos)

    def get_observation(self, timestep_counter):
        lin_vel, ang_vel = p.getBaseVelocity(self.gray_id)
        
        joint_states = np.array([(jPos, jVel, jTor) for jPos, jVel, _, jTor in p.getJointStates(self.gray_id, JOINT_INDICES)]).T
        joint_pos = joint_states[0]
        self.joint_vel = joint_states[1]
        self.joint_tor = joint_states[2]
        
        self.prev_pos = self.curr_pos
        self.curr_pos, curr_ori = p.getBasePositionAndOrientation(self.gray_id)
        self.pos_memory.append(self.curr_pos)
        height = [self.curr_pos[2]]

        done = False
        if timestep_counter > self.memory_size:
            done = self.pos_memory[0][1] - self.pos_memory[-1][1] < DONE_EPSILON

        obs = np.concatenate((lin_vel, ang_vel, joint_pos, self.joint_vel, curr_ori, height))
        return obs, done

    def get_state_reward(self, timestep_counter):
        # punish for falling/tilting
        #eulerOri = p.getEulerFromQuaternion(self.ang)
        #reward -= abs(eulerOri[0] - pi/2) * 1.0  # ROLL: non-slanted: pi/2
        #reward -= abs(eulerOri[1]) * 1.0 # PITCH up: -pi/2 down: pi/2

        # TODO Test these rewards

        # reward for moving forward
        x_dis_reward = (self.curr_pos[0] - self.prev_pos[0]) * X_DIS_WEIGHT

        # punish for energy consumption
        energy_penalty = np.dot(np.abs(self.joint_vel), np.abs(self.joint_tor)) * TIMESTEP * ENERGY_WEIGHT

        # punish for not moving joints enough
        joint_std_penalty = 0
        if timestep_counter > self.memory_size:
            # more variation in joint position => smaller penalty
            joint_std_penalty = JOINT_STD_WEIGHT / np.mean(np.std(self.joint_pos_memory, axis=0))
        
        # bonus for not changing direction in each joint (ensures smooth movement)
        dir_reward = 0
        if timestep_counter > 2:
            last_pos = self.joint_pos_memory[-1]
            second_to_last_pos = self.joint_pos_memory[-2]
            num_no_dir_change = np.count_nonzero((np.sign(last_pos) + np.sign(second_to_last_pos)) != 0)
            # find proportion of joints that have not changed direction
            dir_reward = (num_no_dir_change / len(JOINT_INDICES)) * DIR_WEIGHT

        total_reward = x_dis_reward - energy_penalty - joint_std_penalty + dir_reward

        # log rewards
        if REWARD_LOGGING and timestep_counter % REWARD_LOG_EVERY == 0:
            self.logging(timestep_counter,
                x_dis_reward=x_dis_reward,
                energy_penalty=energy_penalty,
                joint_std_penalty=joint_std_penalty,
                dir_reward=dir_reward,
                total_reward=total_reward)

        return total_reward
    
    def logging(self, timestep_counter, **kwargs):
        print(f"[INFO] Rewards (Timestep: {timestep_counter}):")
        for key, val in kwargs.items():
            print("%24s: %6f" % (key, val))
