import pybullet as p
import numpy as np
import pybullet_data
import os
import time
from math import pi

p.connect(p.GUI)

p.setGravity(0, 0, -9.8)

TIMESTEP = 1/100
p.setTimeStep(TIMESTEP)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(f"{os.path.dirname(os.path.realpath(__file__))}/resources/gray.urdf", basePosition=[0, 0, 0.2], baseOrientation=p.getQuaternionFromEuler([pi/2, 0, 0]), flags=p.URDF_USE_SELF_COLLISION, useFixedBase=False)


while p.isConnected(): 
    # TODO

    p.stepSimulation()
    time.sleep(TIMESTEP)
