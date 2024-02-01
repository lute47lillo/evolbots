"""
Author: Eleuterio Juan Lillo Portero
"""

import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy as np 
import random
import constants as c


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

# Sinusoidally values. Later used for movements in the motor schema.
targetAngles = np.linspace(0, 2*np.pi, 1000)

# Calculate the scaled values using linear transformation
scaled_values = (np.sin(targetAngles) - c.old_min) / (c.old_max - c.old_min) * (c.new_max - c.new_min) + c.new_min
sin = np.sin(scaled_values)

# Just to plot
# for i in range(1000):
#     x = frequency * targetAngles[i] + phaseOffset
#     sin_offset.append(amplitude * np.sin(x))
# np.save("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/target_angles.npy", sin_offset)
# exit()

for i in range(0,1000):
    p.stepSimulation()
    
    # Back Leg motor
    x = c.frequency * targetAngles[i] + c.phaseOffset
    c.sin_offset.append(c.amplitude * np.sin(x))
    
    # Front Leg motor
    x_front = c.frequency_front * targetAngles[i] + c.phaseOffset_front
    c.sin_offset_front.append(c.amplitude_front * np.sin(x_front))

    
    # Back Leg sensor
    backLegTouch = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg") # Touch sensors only work in non-root links !!!!!
    c.backLegSensorValues[i] = backLegTouch
    
    # Front Leg sensor
    frontLegTouch = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    c.frontLegSensorValues[i] = frontLegTouch
    
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
        targetPosition = c.sin_offset[i],
        maxForce = 35
    )
    
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robot3Piece_ID,
        jointName = "Torso_FrontLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = c.sin_offset_front[i],
        maxForce = 35
    )
    
    
    time.sleep(0.001)

np.save("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/sensors.npy", c.backLegSensorValues)
np.save("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/front_sensors.npy", c.frontLegSensorValues)

p.disconnect()