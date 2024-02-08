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

    # desired angle is the angle passed to the Motor Neuron to act
    def Set_Value(self, robot, desiredAngle):
        self.robot = robot
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = self.robot,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = desiredAngle,
            maxForce = 35
        )