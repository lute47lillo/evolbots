"""
Author: Eleuterio Juan Lillo Portero
"""

import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy as np 
import random


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

# Add gravity
p.setGravity(0,0,-9.8)

# Load the entities generated with pyrosim
planeId = p.loadURDF("plane.urdf") #Add a floor
# robotID = p.loadURDF("body.urdf") #Add a robot

p.loadSDF("world.sdf")
robot3Piece_ID = p.loadURDF("3p_body.urdf")
pyrosim.Prepare_To_Simulate(robot3Piece_ID)

# Update range

# Define the desired range
new_min = -np.pi/4
new_max = np.pi/4

# Original range of numpy.sin function is [-1, 1]
old_min = -1
old_max = 1

# Sinusoidally values. Later used for movements in the motor schema.
targetAngles = np.linspace(0, 2*np.pi, 1000)

# Calculate the scaled values using linear transformation
scaled_values = (np.sin(targetAngles) - old_min) / (old_max - old_min) * (new_max - new_min) + new_min
sin = np.sin(scaled_values)

# Create offset movement Back Leg
amplitude = np.pi/2
frequency = 8
phaseOffset = 0
sin_offset = []

# Create offset movement front Leg
amplitude_front = (np.pi/4)/2
frequency_front = 8
phaseOffset_front = np.pi/4
sin_offset_front = []

# Just to plot
# for i in range(1000):
#     x = frequency * targetAngles[i] + phaseOffset
#     sin_offset.append(amplitude * np.sin(x))
# np.save("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/target_angles.npy", sin_offset)
# exit()


backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)
for i in range(0,1000):
    p.stepSimulation()
    
    # Back Leg motor
    x = frequency * targetAngles[i] + phaseOffset
    sin_offset.append(amplitude * np.sin(x))
    
    # Front Leg motor
    x_front = frequency_front * targetAngles[i] + phaseOffset_front
    sin_offset_front.append(amplitude_front * np.sin(x_front))

    
    # Back Leg sensor
    backLegTouch = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg") # Touch sensors only work in non-root links !!!!!
    backLegSensorValues[i] = backLegTouch
    
    # Front Leg sensor
    frontLegTouch = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    frontLegSensorValues[i] = frontLegTouch
    
    # Create a motor 
    """
        bodyIndex: what robot the motor is going to be attached to.
        jointName: what joint from the robot, the motor is going to be attached to.
        controlMode: determines how the motor will attempt to control the motion of the joint.
            - Position control, the motor receives as input a target position.
            - Velocity control is usually used for continuously rotating objects.
        targetPosition: Desired position. In radians units.
        maxForce: cap the total torque ever used by a motor. Newton/meter units.
    """
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robot3Piece_ID,
        jointName = "Torso_BackLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = sin_offset[i],
        maxForce = 35
    )
    
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robot3Piece_ID,
        jointName = "Torso_FrontLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = sin_offset_front[i],
        maxForce = 35
    )
    
    
    time.sleep(0.001)

np.save("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/sensors.npy", backLegSensorValues)
np.save("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/front_sensors.npy", frontLegSensorValues)

p.disconnect()