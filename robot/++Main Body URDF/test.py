import pybullet as p
import pybullet_data
import os
import time
from math import pi

# Create a client of pybullet physics server
client = p.connect(p.GUI)

p.setGravity(0, 0, -10, physicsClientId=client)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

pos1 = p.addUserDebugParameter("Position", -pi, pi, 0)
pos2 = p.addUserDebugParameter("Position", -pi, pi, 0)
pos3 = p.addUserDebugParameter("Position", -pi, pi, 0)
pos4 = p.addUserDebugParameter("Position", -pi, pi, 0)
pos5 = p.addUserDebugParameter("Position", -pi, pi, 0)
pos6 = p.addUserDebugParameter("Position", -pi, pi, 0)
pos7 = p.addUserDebugParameter("Position", -pi, pi, 0)
pos8 = p.addUserDebugParameter("Position", -pi, pi, 0)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(f"{os.path.dirname(os.path.realpath(__file__))}/gray.urdf", basePosition=[0, 0, 0.2], baseOrientation=p.getQuaternionFromEuler([pi/2, pi, 0]))
#  | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)
# 
position, orientation = p.getBasePositionAndOrientation(robotId)

#time_step = 0.01
# for i in range(p.getNumJoints(robotId)):
#     print(p.getJointInfo(robotId, i))
i = 0
while p.isConnected(): 
    #p.setJointMotorControl2(robotId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=10, force=20)
    target_pos1 = p.readUserDebugParameter(pos1)
    target_pos2 = p.readUserDebugParameter(pos2)
    target_pos3 = p.readUserDebugParameter(pos3)
    target_pos4 = p.readUserDebugParameter(pos4)
    target_pos5 = p.readUserDebugParameter(pos5)
    target_pos6 = p.readUserDebugParameter(pos6)
    target_pos7 = p.readUserDebugParameter(pos7)
    target_pos8 = p.readUserDebugParameter(pos8)
    p.setJointMotorControl2(robotId, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=target_pos1)
    p.setJointMotorControl2(robotId, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=target_pos2)
    p.setJointMotorControl2(robotId, jointIndex=4, controlMode=p.POSITION_CONTROL, targetPosition=target_pos3)
    p.setJointMotorControl2(robotId, jointIndex=5, controlMode=p.POSITION_CONTROL, targetPosition=target_pos4)
    p.setJointMotorControl2(robotId, jointIndex=7, controlMode=p.POSITION_CONTROL, targetPosition=target_pos5)
    p.setJointMotorControl2(robotId, jointIndex=8, controlMode=p.POSITION_CONTROL, targetPosition=target_pos6)
    p.setJointMotorControl2(robotId, jointIndex=10, controlMode=p.POSITION_CONTROL, targetPosition=target_pos7)
    p.setJointMotorControl2(robotId, jointIndex=11, controlMode=p.POSITION_CONTROL, targetPosition=target_pos8)
    # pos, ang = p.getBasePositionAndOrientation(robotId, client)
    # pitch = p.getEulerFromQuaternion(ang)[1]
    # if i % 240 == 0:
    #     print(f"RIPBOZO: {pitch}")
    #     print(pos[0])
    joints = [1]
    jointPos = list(zip(*[(val[0], val[1]) for val in p.getJointStates(robotId, jointIndices=joints)]))
    if i % 500 == 0:
        print(jointPos)
    p.stepSimulation()
    i += 1
    #time.sleep(time_step)
