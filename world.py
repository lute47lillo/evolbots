"""
    Author: Lute Lillo Portero
    
    Generates the world (and objects) that lives in the simulation created by the physics engine.
"""
import pybullet as p

class WORLD:

    def __init__(self):
        
        # Generate the world within the simulation
        self.planeId = p.loadURDF("plane.urdf") #Add a floor
        # self.robotID = p.loadURDF("body.urdf") #Add a robot

        self.world = p.loadSDF("world.sdf")