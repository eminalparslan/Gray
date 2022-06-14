import gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.cmd_util import make_vec_env
import training
import os
import time

TIMESTEPS = 200_000
EPISODES = 20

models_dir = "models"
model_name = "TESTING_PYBULLET_NON_REALTIME"
model = f"{model_name}_{int(time.time())}"
model_path = os.path.join(models_dir, model)
os.makedirs(model_path, exist_ok=True)

log_dir = None #"logs"
if log_dir:
    os.makedirs(log_dir, exist_ok=True)

#env = DummyVecEnv([lambda: gym.make("Gray-v0")])

env = make_vec_env("Gray-v0", n_envs=1)
env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)

stats_dir = os.path.join(model_path, "vec_normalize.pkl")
env.save(stats_dir)

model = PPO("MlpPolicy", env, verbose=0, tensorboard_log=log_dir)

for i in range(EPISODES):
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=f"{model_name}_{int(time.time())}")
    model.save(f"{model_path}/{TIMESTEPS*(i+1)}")

env.close()
