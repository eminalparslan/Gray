import gym
from stable_baselines3 import PPO
import training

env = gym.make("Gray-v0")
env.reset()

models_dir = "models/PPO"
model_path = f"{models_dir}1644530595/1500000.zip"

model = PPO.load(model_path, env=env)

EPISODES = 10

for ep in range(EPISODES):
    obs = env.reset()
    done = False
    while not done:
        action, _ = model.predict(obs)
        obs, reward, done, info = env.step(action)
