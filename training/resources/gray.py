import pybullet as p
import numpy as np
import os
from math import pi

class Gray:
    def __init__(self, client):
        self.client_id = client
        f_name = os.path.join(os.path.dirname(__file__), 'gray.urdf')
        self.gray_id = p.loadURDF(fileName=f_name, basePosition=[0, 0, 0.2], physicsClientId=self.client_id, baseOrientation=p.getQuaternionFromEuler([pi/2, 0, 0]), flags=p.URDF_USE_SELF_COLLISION)
        self.step_sizes = [-0.3, -0.1, -0.03, 0, 0.03, 0.1, 0.3]
        self.joints = [1, 2, 4, 5, 7, 8, 10, 11]
        self.prev_deltas = np.zeros(7)
        
    def apply_action(self, action):
        # Clip position somehow. (Can Pybullet do this?)
        deltas = np.array([self.step_sizes[a] for a in action], dtype=np.float32)
        current_pos = np.array([val[0] for val in p.getJointStates(self.gray_id, jointIndices=self.joints)], dtype=np.float32)
        new_pos = current_pos + deltas
        np.clip(new_pos, -0.2*pi, 0.2*pi, out=new_pos)
        p.setJointMotorControlArray(self.gray_id, jointIndices=self.joints, controlMode=p.POSITION_CONTROL, targetPositions=new_pos)
        jitterPunishment = sum([1 if np.sign(a) != np.sign(b) else 0 for a, b in zip(deltas, self.prev_deltas)])
        self.prev_deltas = deltas
        return jitterPunishment
    
    def get_observation(self):
        # vel and angular vel
        linVal, angVal = p.getBaseVelocity(self.gray_id, self.client_id)
        # joint pos and vel
        jointPos = []
        jointVel = []
        for jPos, jVel, _, _ in p.getJointStates(self.gray_id, jointIndices=self.joints):
            jointPos.append(jPos)
            jointVel.append(jVel)
        # xyz_vel, wxyz_vel, jointPos*8, jointVel*8
        obs = np.array(list(linVal) + list(angVal) + jointPos + jointVel, dtype=np.float32)
        return obs

    def get_pos(self):
        pos, ang = p.getBasePositionAndOrientation(self.gray_id, self.client_id)
        ori = p.getEulerFromQuaternion(ang)
        # xyz_pos, rpy
        return np.array(pos + ori, dtype=np.float32)

