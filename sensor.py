"""
    Author: Lute Lillo Portero
    
    Create the sensors to be used with the robot and some useful util functions
"""
import numpy as np
import pyrosim.pyrosim as pyrosim

class SENSOR:

    def __init__(self, linkName):
        
        # Get specific link name for a given sensor
        self.linkName = linkName
        
        # Sensors
        self.values = np.zeros(1000)
        
    def Get_Value(self, timestep):
        self.touch_sensor = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName) # Touch sensors only work in non-root links !!!!!
        self.values[timestep] = self.touch_sensor
        
    def Save_Values(self):
        np.save("data/" + self.linkName + "SensorValues", self.values)