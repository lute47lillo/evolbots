import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy as np 


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


backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)
for i in range(0,1000):
    p.stepSimulation()
    
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
        targetPosition: Desired position.
        maxForce: cap the total torque ever used by a motor. Newton/meter units
    """
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robot3Piece_ID,
        jointName = "Torso_BackLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = 0.0,
        maxForce = 500
    )
    
    time.sleep(0.01)

np.save("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/sensors.npy", backLegSensorValues)
np.save("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/front_sensors.npy", frontLegSensorValues)

p.disconnect()