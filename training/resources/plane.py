import pybullet as p
import pybullet_data

class Plane:
    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf", basePosition=[0, 0, 0])
