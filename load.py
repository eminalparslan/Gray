import gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import training

env = DummyVecEnv([lambda: gym.make("Gray-v0")])
env = VecNormalize(env)
env.training = False
env.norm_reward = False
env.reset()

models_dir = "models/"
model_path = f"{models_dir}PPO_60HZ_VEC_NORMALIZE_1644900974/5000000.zip"

model = PPO.load(model_path, env=env)

EPISODES = 10

for ep in range(EPISODES):
    obs = env.reset()
    done = False
    while not done:
        action, _ = model.predict(obs)
        obs, reward, done, info = env.step(action)
