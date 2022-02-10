from gym.envs.registration import register

register(id="Gray-v0", entry_point="training.envs:GrayEnv")