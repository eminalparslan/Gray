import gym
import pybullet as p
import numpy as np
from collections import deque
from math import pi
import time

from training.resources.plane import Plane
from training.resources.gray import Gray
from training.env_config import *


# create Open AI gym environment
class GrayEnv(gym.Env):
    def __init__(self):
        super(GrayEnv, self).__init__()

        # create the action space for the neural network
        # each of the 8 joints have 7 possible discrete actions
        self.action_space = gym.spaces.multi_discrete.MultiDiscrete(NUM_JOINTS*[NUM_DELTAS])

        # continuous observation space for the neural network
        # xyz_vel (3), xyz_ang_vel (3), jointPos (8), ~jointVel (8)~, orientation quaternion (4), height (1)
        self.observation_space = gym.spaces.box.Box(
            low=np.array([-5] * 6 + [-0.2*pi] * 8 + [-10] * 12 + [0]),
            high=np.array([5] * 6 + [0.2*pi] * 8 + [10] * 12 + [3])
        )
        
        print(f"SEED: {self.seed()}")

        # connect to the physics server
        self.client = p.connect(p.GUI)
        p.setTimeStep(TIMESTEP, self.client)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, ENABLE_GUI)

        self.reset()

    def step(self, action):
        # for "real time" simulation, uncomment following line
        time.sleep(TIMESTEP)

        self.gray.apply_action(action)
        p.stepSimulation()

        self.timestep_counter += 1

        obs, no_limit_done = self.gray.get_observation(self.timestep_counter)

        if TIMESTEPS_PER_EP is None:
            self.done = no_limit_done
        else:
            if self.timestep_counter == TIMESTEPS_PER_EP:
                self.done = True

        reward = self.gray.get_state_reward(self.timestep_counter)
        return obs, reward, self.done, dict()

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -9.8)

        Plane()
        self.gray = Gray()

        self.done = False
        self.timestep_counter = 0

        obs, _ = self.gray.get_observation(self.timestep_counter)
        return obs
 
    def close(self):
        p.disconnect(self.client)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
