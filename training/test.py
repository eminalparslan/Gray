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

links = [2, 5, 8, 11]

counter = 0
while p.isConnected(): 
    if counter % 100 == 0:
        for i in links:
            p.setDebugObjectColor(robotId, i, [1, 0, 0])
    print()
    p.stepSimulation()
    counter += 1
    time.sleep(TIMESTEP)
