"""
    Author: Lute Lillo Portero
    
    Create the motors to be used with the robot and some useful util functions

    bodyIndex: what robot the motor is going to be attached to.
    jointName: what joint from the robot, the motor is going to be attached to.
    controlMode: determines how the motor will attempt to control the motion of the joint.
        - Position control, the motor receives as input a target position.
        - Velocity control is usually used for continuously rotating objects.
    targetPosition: Desired position. In radians units.
    maxForce: cap the total torque ever used by a motor. Newton/meter units.

"""
import pybullet as p
import numpy as np
import pyrosim.pyrosim as pyrosim
import constants as c

class MOTOR:

    def __init__(self, jointName):

        # Set variables
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.phase_offset = c.phaseOffset
        self.jointName = jointName
        
        self.Prepare_To_Act()

    
    def Prepare_To_Act(self):
        # if self.jointName == "Torso_BackLeg":
        #     self.frequency = self.frequency * 0.5 
        
        self.targetAngles = np.arange(0, 2*np.pi, 2*np.pi/1000)
        self.x = self.frequency * self.targetAngles + self.phase_offset
        self.motor_values = self.amplitude * np.sin(self.x)
    
        
    def Set_Value(self, robot, t):
        self.robot = robot
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = self.robot,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = self.motor_values[t],
            maxForce = 35
        )
        
    def Save_Values(self):
        np.save("data/" + self.jointName + "MotorValues", self.motor_values)
        