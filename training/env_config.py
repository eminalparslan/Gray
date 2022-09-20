# possible step sizes for each joint
STEP_SIZES = [-0.3, -0.1, -0.03, 0, 0.03, 0.1, 0.3]
# joint indices (not including hip joints)
JOINT_INDICES = [1, 2, 4, 5, 7, 8, 10, 11]
LEG_LINK_INDICES = [2, 5, 8, 11]
NUM_JOINTS = 8
NUM_DELTAS = 7
# timestep for simulation in seconds
TIMESTEP = 1/100
# if 'None', then episode will end after robot stops moving forward
TIMESTEPS_PER_EP = 2000
# if robot hasn't traveled more than this distance in the past second, episode is done
# only works if TIMESTEPS_PER_EP is None
DONE_EPSILON = 0
# pybullet sim settings
ENABLE_GUI = 0
# reward logging
REWARD_LOGGING = False
REWARD_LOG_EVERY = 400
# reward weights
X_DIS_WEIGHT = 10.0
ENERGY_WEIGHT = 0.01
#JOINT_STD_WEIGHT = 0.015
JOINT_STD_WEIGHT = 0.025
DIR_WEIGHT = 0.1
