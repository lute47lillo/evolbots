"""
    Author: Lute Lillo Portero
    
    Generates the robot that interacts in the simulation created by the physics engine.
"""
import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR

class ROBOT:

    def __init__(self):
        
        # Load the entities generated with pyrosim
        self.robot3Piece_ID = p.loadURDF("3p_body.urdf")
        
        # Start simulation upon the inherited robot
        pyrosim.Prepare_To_Simulate(self.robot3Piece_ID)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        
    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)
            
    def Sense(self, timestep):
        for n, linkName in enumerate(self.sensors.keys()):
            self.sensors[linkName].Get_Value(timestep)
            
    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self, timestep):
        for motor in self.motors.values():
            motor.Set_Value(self.robot3Piece_ID, timestep)
        
        
        