"""
    Author: Lute Lillo Portero
    
    Generates the robot that interacts in the simulation created by the physics engine.
"""
import pybullet as p
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR
import os

class ROBOT:

    def __init__(self, solID):
        
        # Load the entities generated with pyrosim
        self.robot3Piece_ID = p.loadURDF("3p_body.urdf")

        # Create a neural network (self.nn), 
        # and add any neurons and synapses to it from brain.nndf.
        self.nn = NEURAL_NETWORK(f"brain{solID}.nndf")
        
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

    def Act(self):
        for neuronName in self.nn.Get_Neuron_Names(): # iterate over all the neurons in the neural network
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.motors[jointName].Set_Value(self.robot3Piece_ID, desiredAngle)
            
    def Think(self):
        self.nn.Update() # Propagate sensor values to hidden and motor neurons
        # self.nn.Print()
        
    def Get_Fitness(self, id):
        """
            Get fitness value from an arbitrary function.
            Write that fitness to a file to be used in evolution.
            
            Parameters:
            - id: Unique ID associated with a parent/child robot.
        """
        # Temporary file writing to ensure W/R problems
        f = open(f"tmp{id}.txt", "w")
        f.write(str(p.getLinkState(self.robot3Piece_ID, 0)[0][0]))
        f.close()
        os.rename(f"tmp{id}.txt", f"fitness{id}.txt")
        
        
        