import gym
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
from math import pi

from training.resources.plane import Plane
from training.resources.gray import Gray

class GrayEnv(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self):
        # super(GrayEnv, self).__init__()
        self.action_space = gym.spaces.multi_discrete.MultiDiscrete(8*[7])
        self.observation_space = gym.spaces.box.Box(
            low=np.array([-10, -10] + [-pi] * 8, dtype=np.float32),
            high=np.array([10, 10] + [pi] * 8, dtype=np.float32))
        self.np_random, _ = gym.utils.seeding.np_random()

        self.client = p.connect(p.GUI)
        p.setTimeStep(1/30, self.client)

        self.counter = 0

        self.rendered_img = None
        self.reset()

    def step(self, action):
        self.gray.apply_action(action)
        p.stepSimulation()
        self.counter += 1
        obs = self.gray.get_observation()

        # Compute rewards
        reward = action[2] * 10.0 - action[3] * 5.0

        if self.counter == 2000:
            self.done = True

        return obs, reward, self.done, dict()

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -10)

        Plane(self.client)
        self.gray = Gray(self.client)

        # Useless for now
        self.done = False
        self.counter = 0

        obs = self.gray.get_observation()
        return obs
 
    def render(self):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((100, 100, 4)))
        
        proj_matrix = p.computeProjectionMatrixFOV(fov=80, aspect=1, nearVal=0.01, farVal=100)
        pos, ori = [list(l) for l in p.getBasePositionAndOrientation(self.gray.gray_id, self.gray.client_id)]
        pos[2] = 0.2

        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, np.array([1, 0, 0]))
        up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        view_matrix = p.computeViewMatrix(pos, pos + camera_vec, up_vec)

        frame = p.getCameraImage(100, 100, view_matrix, proj_matrix)[2]
        frame = np.reshape(frame, (100, 100, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.00001)

    def close(self):
        p.disconnect(self.client)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
