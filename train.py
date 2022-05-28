import gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import training
import os
import time

# PARALELLIZATION

MODEL_NAME = "PPO_NO_DIR_CHANGE_BONUS"
TIMESTEPS = 500_000
EPISODES = 20
TENSORBOARD_LOG_DIR = None # "logs"

models_dir = f"models/{MODEL_NAME}_{int(time.time())}"
if not TENSORBOARD_LOG_DIR is None:
    os.makedirs(TENSORBOARD_LOG_DIR, exist_ok=True)
os.makedirs(models_dir, exist_ok=True)

env = DummyVecEnv([lambda: gym.make("Gray-v0")])
env = VecNormalize(env, norm_obs=True, norm_reward=True)

model = PPO("MlpPolicy", env, verbose=0, tensorboard_log=TENSORBOARD_LOG_DIR)

for i in range(1, EPISODES + 1):
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=f"{MODEL_NAME}_{int(time.time())}")
    model.save(f"{models_dir}/{TIMESTEPS*i}")

env.close()
