import gym
from stable_baselines3 import PPO
import training
import os
import time

models_dir = f"models/PPO{int(time.time())}"
logdir = "logs"

os.makedirs(models_dir, exist_ok=True)
os.makedirs(logdir, exist_ok=True)

env = gym.make("Gray-v0")

model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=logdir)
TIMESTEPS = 1_000_000
for i in range(1, 10):
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=f"PPO{int(time.time())}")
    model.save(f"{models_dir}/{TIMESTEPS*i}")

env.close()
