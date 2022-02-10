import pybullet as p
import pybullet_data
import os

class Plane:
    def __init__(self, client):
        # f_name = os.path.join(os.path.dirname(__file__), 'plane.urdf')
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf", basePosition=[0, 0, 0], physicsClientId=client)
