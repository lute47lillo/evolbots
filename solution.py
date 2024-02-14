import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time

class SOLUTION:
    
    def __init__(self, nextAvailableID) -> None:
        self.weights = np.random.rand(3, 2) * 2 -1
        self.myID = nextAvailableID
        
    def evaluate(self, directOrGUI):
        """
            Generate the robot's world, its body, its neural network,
            and send the weights in this solution when it sends the synaptic weights.
        """
        pass
        
        
    def set_ID(self, id):
        """
            Set unique ID for each parent and child offspring.
            
            Parameters:
            - id: Next available ID when robot generation.
        """
        self.myID = id
        
    def start_simulation(self, directOrGUI):
        # self.create_world()
        # self.create_3piece_robot()
        self.generate_brain()
        os.system("python3 simulate.py " + directOrGUI + " " + str(self.myID) + " &" )
    
    def wait_for_simulation_to_end(self):
        """
            Reads fitness value from file and waits for simulation to be done 
        """
        while not os.path.exists(f"fitness{self.myID}.txt"):
            time.sleep(0.01)
        
        f = open(f"fitness{self.myID}.txt", "r")
        self.fitness = float(f.read())
        f.close()
        
        os.system(f'rm fitness{self.myID}.txt')
        
    def mutate(self):
        """
            Choose randomly a particular weight of a synaptic connection and mutate it.
            
            TODO: select the ith and jth connection by general variables. Currently hard-coded
            for Assignment 9.
        """
        self.weights[random.randint(0, 2)][random.randint(0, 1)] = random.random()*2 - 1
    
    def create_world(self):
        pyrosim.Start_SDF("world.sdf")

        # Store the box
        length = 1
        width = 1
        height = 1
        x,y,z = -5,5,0.5
        
        
        pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length, width, height])

        pyrosim.End()
    
    def create_3piece_robot(self):
        length = 1
        width = 1
        height = 1
        x,y,z = 1.5,0,1.5
        
        # Create Robot Body
        pyrosim.Start_URDF("3p_body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[length, width, height])
            
        
        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1,0,1])
        
        pyrosim.Send_Cube(name="BackLeg", pos=[x-2,y,z-2] , size=[length, width, height])
        
        
        pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2,0,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[x-1,y,z-2] , size=[length, width, height])
        
        pyrosim.End()
        
    def generate_brain(self):
        
        # Create neural network
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")
        
        # Send value from links sensors to sensor neuros.
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
        
        # Send to motor neurons for actions. Using the joints
        pyrosim.Send_Motor_Neuron(name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")
        
        for curr_row in range(0, 3):
            for curr_column in range(0,2):
                pyrosim.Send_Synapse(sourceNeuronName = curr_row,
                                     targetNeuronName = curr_column+3,
                                     weight = self.weights[curr_row][curr_column] )
        
        pyrosim.End()
    