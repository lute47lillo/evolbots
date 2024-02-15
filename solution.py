import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import constants as c
import time

class SOLUTION:
    
    def __init__(self, nextAvailableID) -> None:
        self.weights = np.random.rand(c.num_sensor_neurons, c.num_motor_neurons) * 2 -1
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
        self.create_3piece_robot()
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
        self.weights[random.randint(0, c.num_sensor_neurons-1)][random.randint(0, c.num_motor_neurons-1)] = random.random()*2 - 1
    
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
        pyrosim.Send_Cube(name="Torso", pos=[0,0,1] , size=[length, width, height])
            
        # Back Leg connected to Torso
        pyrosim.Send_Joint(name = "Torso_BackLeg", parent= "Torso", child = "BackLeg",
                           type = "revolute", position = [0,-0.5,1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0, -0.5, 0] , size=[0.2,1,0.2])
        
        # Front Leg connected to Torso
        pyrosim.Send_Joint(name = "Torso_FrontLeg", parent= "Torso", child = "FrontLeg",
                           type = "revolute", position=[0,0.5,1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0], size=[0.2,1,0.2])
        
        # Left Leg connected to Torso
        pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso", child="LeftLeg",
                           type="revolute", position=[-0.5,0,1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5, 0, 0], size=[1,0.2,0.2])
        
        # Right Leg connected to Torso
        pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso", child="RightLeg",
                           type="revolute", position=[0.5,0,1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5, 0, 0], size=[1,0.2,0.2])


        # Add Lower Legs to all Legs connected to Torso
        pyrosim.Send_Joint(name="FrontLeg_FrontLowerLeg", parent="FrontLeg", child="FrontLowerLeg",
                           type="revolute", position=[0,1,0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0, 0, -0.5], size=[0.2,0.2,1])

        pyrosim.Send_Joint(name="BackLeg_BackLowerLeg", parent="BackLeg", child="BackLowerLeg",
                           type="revolute", position=[0,-1,0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackLowerLeg", pos=[0, 0, -0.5], size=[0.2,0.2,1])

        pyrosim.Send_Joint(name="LeftLeg_LeftLowerLeg", parent="LeftLeg", child="LeftLowerLeg",
                           type="revolute", position=[-1,0,0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0, 0, -0.5], size=[0.2,0.2,1])

        pyrosim.Send_Joint(name="RightLeg_RightLowerLeg", parent="RightLeg", child="RightLowerLeg",
                           type="revolute", position=[1,0,0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0, 0, -0.5], size=[0.2,0.2,1])
        
        pyrosim.End()
        
    def generate_brain(self):
        
        # Create neural network
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")
        
        # Send value from links sensors to sensor neuros.
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        # pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        # pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
        
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "FrontLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "BackLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = "LeftLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 4, linkName = "RightLowerLeg")
        
        # Send to motor neurons for actions. Using the joints
        # pyrosim.Send_Motor_Neuron(name = 3 , jointName = "Torso_BackLeg")
        # pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")
        
        pyrosim.Send_Motor_Neuron(name = 5, jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 6, jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name = 7, jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name = 8, jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name = 9, jointName = "FrontLeg_FrontLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 10, jointName = "BackLeg_BackLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 11, jointName = "LeftLeg_LeftLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 12, jointName = "RightLeg_RightLowerLeg")
        
        for curr_row in range(0, c.num_sensor_neurons):
            for curr_column in range(0,c.num_motor_neurons):
                pyrosim.Send_Synapse(sourceNeuronName = curr_row,
                                     targetNeuronName = curr_column + c.num_sensor_neurons,
                                     weight = self.weights[curr_row][curr_column] )
        
        pyrosim.End()
    