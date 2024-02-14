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
    
    def __init__(self, directOrGUI):
        
        # Start the simulation engine
        if directOrGUI == "DIRECT":
            # Run the simulation blindly to not get it on screen
            self.physicsClient = p.connect(p.DIRECT) 
        else:
            # Heads-up mode
            self.physicsClient = p.connect(p.GUI, options="--width=1920 --height=1080")
        
        
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
            self.robot.Think()
            self.robot.Act(i)
            self.Get_Fitness()
            
            time.sleep(0.001)
        
    def Get_Fitness(self):
        self.robot.Get_Fitness()
    
    def __del__(self):

        p.disconnect()
        # Make calls if need to save motors and sensors values