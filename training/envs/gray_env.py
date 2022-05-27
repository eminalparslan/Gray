import gym
import numpy as np
import pybullet as p
from math import pi
import time

from training.resources.plane import Plane
from training.resources.gray import Gray
from training.constants import *


# create Open AI gym environment
class GrayEnv(gym.Env):
    # FIXME no idea what this does
    metadata = {"render.modes": ["human"]}

    def __init__(self):
        # FIXME don't know if I need this
        # super(GrayEnv, self).__init__()

        # create the action space for the neural network
        # each of the 8 joints have 7 possible discrete actions
        self.action_space = gym.spaces.multi_discrete.MultiDiscrete(NUM_JOINTS*[NUM_DELTAS])

        # if I want to use the continuous action space
        #self.action_space = gym.spaces.box.Box(
        #    low=np.array([-0.2 * pi] * 8, dtype=np.float32),
        #    high=np.array([0.2 * pi] * 8, dtype=np.float32)
        #)

        # observation space for the neural network
        # xyz_vel (3), xyz_ang_vel (3), jointPos (8), jointVel (8), orientation quaternion (4), height (1)
        self.observation_space = gym.spaces.box.Box(
            low=np.array([-5] * 6 + [-0.2*pi] * 8 + [-10] * 12 + [0]),
            high=np.array([5] * 6 + [0.2*pi] * 8 + [10] * 12 + [3])
        )
        
        # seed if I want the same random numbers generated every run
        self.seed()

        # connect to the physics server
        self.client = p.connect(p.GUI)
        # set up physics simulation
        p.setTimeStep(TIMESTEP, self.client)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, ENABLE_GUI)

        self.reset()

    def step(self, action):
        time.sleep(TIMESTEP)

        self.gray.apply_action(action)
        
        p.stepSimulation()

        self.timestep_counter += 1
        # TODO done iff no movement in x direction for a certain amount of time
        if self.timestep_counter == TIMESTEPS_PER_EP:
            self.done = True

        # xyz_vel, xyz_ang_vel, jointPos*8, jointVel*8, orientation quaternion, height
        obs = self.gray.get_observation()
        reward = self.gray.get_state_reward(self.timestep_counter)
        return obs, reward, self.done, dict()

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -9.8)

        Plane()
        self.gray = Gray()

        self.done = False
        self.timestep_counter = 0

        return self.gray.get_observation() 
 
    def close(self):
        p.disconnect(self.client)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
