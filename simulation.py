"""
    Author: Lute Lillo Portero

"""
from world import WORLD
from robot import ROBOT
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import time


class SIMULATION:
    
    def __init__(self):
        
        # Start the simulation engine
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

        # Add gravity
        p.setGravity(0,0,-9.8)
        
        self.world = WORLD()
        self.robot = ROBOT()
        
    def Run(self):
        for i in range(0,1000):
            p.stepSimulation()
            
            self.robot.Sense(i)
            self.robot.Act(i)
            
            time.sleep(0.001)
    
    def __del__(self):

        p.disconnect()
        # Make calls if need to save motors and sensors values