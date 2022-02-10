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
        
    def apply_action(self, action):
        # Clip position somehow. (Can Pybullet do this?)
        deltas = np.array([self.step_sizes[a] for a in action], dtype=np.float32)
        current_pos = np.array([val[0] for val in p.getJointStates(self.gray_id, jointIndices=self.joints)], dtype=np.float32)
        new_pos = current_pos + deltas
        p.setJointMotorControlArray(self.gray_id, jointIndices=self.joints, controlMode=p.POSITION_CONTROL, targetPositions=new_pos)
    
    def get_observation(self):
        # pos, ang = p.getBasePositionAndOrientation(self.gray_id, self.client_id)
        # pos = pos[:2]
        # ang = p.getEulerFromQuaternion(self.gray_id, self.client_id)
        vel = p.getBaseVelocity(self.gray_id, self.client_id)[0][:2]
        # x_vel, y_vel, 12 * pos
        current_pos = [val[0] for val in p.getJointStates(self.gray_id, jointIndices=self.joints)]
        obs = (vel + tuple(current_pos))
        return np.array(obs, dtype=np.float32)
