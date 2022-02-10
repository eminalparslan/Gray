from tabnanny import verbose
import gym
from stable_baselines3 import PPO
import training
import os

models_dir = "models/PPO"
logdir = "logs"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(logdir):
    os.makedirs(logdir)

env = gym.make("Gray-v0")

model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=1_000_000)

episodes = 10

for ep in range(episodes):
    obs = env.reset()
    for i in range(1_000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        # env.render()

env.close()
