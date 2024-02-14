"""
    Author: Lute Lillo Portero

"""
from world import WORLD
from robot import ROBOT
import pybullet as p
import pybullet_data
import time


class SIMULATION:
    
    def __init__(self, directOrGUI, solutionID):
        
        self.directOrGUI = directOrGUI
        self.solutionID = solutionID
        
        
        # Start the simulation engine
        if self.directOrGUI == "DIRECT":
            # Run the simulation blindly to not get it on screen
            self.physicsClient = p.connect(p.DIRECT) 
        else:
            # Heads-up mode
            self.physicsClient = p.connect(p.GUI)
        
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

        # Add gravity
        p.setGravity(0,0,-9.8)
        
        self.world = WORLD()
        self.robot = ROBOT(self.solutionID)
        
    def Run(self):
        for i in range(0,1000):
            p.stepSimulation()
            
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act()
            self.Get_Fitness()
            
            if self.directOrGUI == "GUI":
                time.sleep(0.01)
        
    def Get_Fitness(self):
        self.robot.Get_Fitness(self.solutionID)
    
    def __del__(self):

        p.disconnect()
        # Make calls if need to save motors and sensors values