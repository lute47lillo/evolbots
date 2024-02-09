import pyrosim.pyrosim as pyrosim

def create_world():
    pyrosim.Start_SDF("world.sdf")

    # Store the box
    length = 1
    width = 1
    height = 1
    x,y,z = -5,5,0.5
    
    
    pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length, width, height])

    pyrosim.End()
    
def create_robot():
    # Store the box
    length = 1
    width = 1
    height = 1
    x,y,z = 0,0,0.5
    
    # Create Robot Body
    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="Link0", pos=[x,y,z] , size=[length, width, height])
    
    # Only the first link in a robot --- the "root" link --- has an absolute position.
    # Every other link has a position relative to its "upstream" joint.
    
    
    # Create the joint for the links. The position should be right where both joints come together.
    # By convention the name should be Parent_Child joint names.
    # BE aware of relative and absolute joint positions.
    # First Joint is always absolute position wrt 1st Link. The rest are relative to this first set.
    pyrosim.Send_Joint( name = "Link0_Link1" , parent= "Link0" , child = "Link1" , type = "revolute", position = [0,0,1])
    
    leg_pos = [0,0,0.5]
    pyrosim.Send_Cube(name="Link1", pos=[leg_pos[0], leg_pos[1], leg_pos[2]], size=[length, width, height])
    
    
    
    pyrosim.Send_Joint( name = "Link1_Link2" , parent= "Link1" , child = "Link2" , type = "revolute", position = [0,0,1])
    
    leg_pos2 = [0,0,0.5]
    pyrosim.Send_Cube(name="Link2", pos=[leg_pos2[0], leg_pos2[1], leg_pos2[2]], size=[length, width, height])
    
    
    
    pyrosim.Send_Joint( name = "Link2_Link3" , parent= "Link2" , child = "Link3" , type = "revolute", position = [0,0.5,0.5])
    
    leg_pos2 = [0,0.5,0]
    pyrosim.Send_Cube(name="Link3", pos=[leg_pos2[0], leg_pos2[1], leg_pos2[2]], size=[length, width, height])
    
    
    
    pyrosim.Send_Joint( name = "Link3_Link4" , parent= "Link3" , child = "Link4" , type = "revolute", position = [0,1,0])
    
    leg_pos2 = [0,0.5,0]
    pyrosim.Send_Cube(name="Link4", pos=[leg_pos2[0], leg_pos2[1], leg_pos2[2]], size=[length, width, height])
    
    
    pyrosim.Send_Joint( name = "Link4_Link5" , parent= "Link4" , child = "Link5" , type = "revolute", position = [0,0,-1])
    
    leg_pos2 = [0,0.5,0]
    pyrosim.Send_Cube(name="Link5", pos=[leg_pos2[0], leg_pos2[1], leg_pos2[2]], size=[length, width, height])
    
    
    pyrosim.Send_Joint( name = "Link5_Link6" , parent= "Link5" , child = "Link6" , type = "revolute", position = [0,0,-1])
    
    leg_pos2 = [0,0.5,0]
    pyrosim.Send_Cube(name="Link6", pos=[leg_pos2[0], leg_pos2[1], leg_pos2[2]], size=[length, width, height])
    
    
    pyrosim.End()
    
    
def create_3piece_robot():
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
    
def generate_brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")
    
    # Send value from links sensors to sensor neuros.
    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
    pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
    
    # Send to motor neurons for actions. Using the joints
    pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
    pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
    
    # Generate synapse to close the brain loop
    # pyrosim.Send_Synapse(sourceNeuronName = 0, targetNeuronName = 3 , weight = -1.0 )
    # pyrosim.Send_Synapse(sourceNeuronName = 1, targetNeuronName = 3, weight = -1.0)
    # pyrosim.Send_Synapse(sourceNeuronName = 2, targetNeuronName = 3, weight = 0.0)
    # pyrosim.Send_Synapse(sourceNeuronName = 1, targetNeuronName = 4, weight = 0.0)
    # pyrosim.Send_Synapse(sourceNeuronName = 2, targetNeuronName = 4, weight = 1.0)
    
    pyrosim.Send_Synapse(sourceNeuronName = 0, targetNeuronName = 3, weight = -1.0)
    pyrosim.Send_Synapse(sourceNeuronName = 1, targetNeuronName = 3, weight = 0.0)
    pyrosim.Send_Synapse(sourceNeuronName = 2, targetNeuronName = 3, weight = 0.0)
    pyrosim.Send_Synapse(sourceNeuronName = 0, targetNeuronName = 4, weight = 0.0)
    pyrosim.Send_Synapse(sourceNeuronName = 1, targetNeuronName = 4, weight = 0.0)
    pyrosim.Send_Synapse(sourceNeuronName = 2, targetNeuronName = 4, weight = 1.0)
    pyrosim.End()

create_world()
# create_robot()
create_3piece_robot()
generate_brain()
    