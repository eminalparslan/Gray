from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.cmd_util import make_vec_env
import training
import os

EPISODES = 30

models_dir = "models"
model = "PPO_NO_DIR_CHANGE_BONUS_1653830591"
timestep = "4000000.zip"
model_path = os.path.join(models_dir, model, timestep)

stats_path = os.path.join(models_dir, model, "vec_normalize.pkl")

#env = DummyVecEnv([lambda: gym.make("Gray-v0")])
env = make_vec_env("Gray-v0", n_envs=1)
env = VecNormalize.load(stats_path, env)

model = PPO.load(model_path, env=env)

obs = env.reset()

for ep in range(EPISODES):
    done = False
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        # VecEnv automatically resets the environment when done
