import pybullet as p
import pybullet_data
import os
import time
from math import pi

# Create a client of pybullet physics server
client = p.connect(p.GUI)

p.setGravity(0, 0, -10, physicsClientId=client)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

pos = p.addUserDebugParameter("Position", 0, 2*pi, 0)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(f"{os.path.dirname(os.path.realpath(__file__))}/gray.urdf", basePosition=[0, 0, 0.2], baseOrientation=p.getQuaternionFromEuler([pi/2, 0, 0]), flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)

position, orientation = p.getBasePositionAndOrientation(robotId)

#time_step = 0.01
for i in range(p.getNumJoints(robotId)):
    print(p.getJointInfo(robotId, i))

# while p.isConnected(): 
#     #p.setJointMotorControl2(robotId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=10, force=20)
#     target_pos = p.readUserDebugParameter(pos)
#     p.setJointMotorControl2(robotId, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=target_pos)

    
#     p.stepSimulation()
    #time.sleep(time_step)
