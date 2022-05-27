import numpy as np
import pybullet as p
from math import pi
from collections import deque
import os

from training.constants import *


class Gray:
    def __init__(self):
        urdf_filepath = os.path.join(os.path.dirname(__file__), 'gray.urdf')
        urdf_flags = p.URDF_USE_SELF_COLLISION
        # start slightly above ground
        base_pos = [0, 0, 0.2]
        # gray starts on his side, so we need to rotate it 90 degrees
        base_ori = p.getQuaternionFromEuler([pi/2, 0, 0])

        self.gray_id = p.loadURDF(urdf_filepath, base_pos, base_ori, useFixedBase=False, flags=urdf_flags)

        # possible step sizes for each joint
        self.step_sizes = [-0.3, -0.1, -0.03, 0, 0.03, 0.1, 0.3]
        # joint indices (doesn't include hip joints)
        self.joints = [1, 2, 4, 5, 7, 8, 10, 11]
        # current position
        self.pos = (0, 0, 0)
        # position memory
        self.memory_size = int(1 / TIMESTEP)
        self.memory = deque(maxlen=self.memory_size)

        # FIXME don't know what this does
        for joint in range(p.getNumJoints(self.gray_id)):
            p.changeDynamics(self.gray_id, joint, linearDamping=0, angularDamping=0)

    def apply_action(self, actions):
        deltas = np.array([self.step_sizes[a] for a in actions])
        current_pos = np.array([val[0] for val in p.getJointStates(self.gray_id, self.joints)])
        new_pos = np.clip(current_pos + deltas, -0.2*pi, 0.2*pi)

        self.memory.append(new_pos)

        p.setJointMotorControlArray(self.gray_id, self.joints, p.POSITION_CONTROL, new_pos)

    def get_observation(self):
        self.lin_vel, self.ang_vel = p.getBaseVelocity(self.gray_id)
        
        # TODO make more readable
        joint_states = np.array([(jPos, jVel, jTor) for jPos, jVel, _, jTor in p.getJointStates(self.gray_id, self.joints)]).T
        self.joint_pos = joint_states[0]
        self.joint_vel = joint_states[1]
        self.joint_tor = joint_states[2]
        
        self.prev_pos = self.pos
        self.pos, self.ang = p.getBasePositionAndOrientation(self.gray_id)
        self.height = [self.pos[2]]

        # observation: 
        # xyz_vel, xyz_ang_vel, jointPos*8, jointVel*8, orientation quaternion, height
        return np.concatenate((self.lin_vel, self.ang_vel, self.joint_pos, self.joint_vel, self.ang, self.height))

    def get_state_reward(self, counter):
        # punish for falling/tilting
        #eulerOri = p.getEulerFromQuaternion(self.ang)
        #reward -= abs(eulerOri[0] - pi/2) * 1.0  # ROLL: non-slanted: pi/2
        #reward -= abs(eulerOri[1]) * 1.0 # PITCH up: -pi/2 down: pi/2

        # punish for not moving joints enough
        joint_pos_std = 0
        if counter > self.memory_size:
            joint_std_weight = 0.015
            joint_pos_std = joint_std_weight / np.mean(np.std(self.memory, axis=0))

        # reward for moving forward
        x_dis_weight = 10.0
        x_dis = (self.pos[0] - self.prev_pos[0]) * x_dis_weight

        # punish little for energy consumption (joint torques)
        energy_weight = 0.01
        energy = np.dot(np.abs(self.joint_vel), np.abs(self.joint_tor)) * TIMESTEP * energy_weight

        reward = x_dis - energy - joint_pos_std

        # log every 500 timesteps
        if counter % 500 == 0:
            self.logging(x_dis=x_dis, energy=-energy, delta_std=-joint_pos_std) 

        return reward
    
    def logging(self, **kwargs):
        print("[INFO] Rewards:")
        for key, val in kwargs.items():
            print("%10s: %5f" % (key, val))
