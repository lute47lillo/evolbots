import numpy as np
from matplotlib import pyplot as plt

def plot_sensors():
    backLegSensorValues = np.load("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/sensors.npy")

    frontLegSensorValues = np.load("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/front_sensors.npy")

    print(backLegSensorValues)
    print(frontLegSensorValues)


    plt.plot(backLegSensorValues, linewidth=1, label="back")
    plt.plot(frontLegSensorValues, label="front")
    plt.legend()
    plt.show()
    
def plot_target_angles():
    target_angles = np.load("/Users/lutelillo/Desktop/UVM SPRING'24/CSYS5990A_Advanced Evolutionary Robotics/evolbots/data/target_angles.npy")
    plt.plot(target_angles, linewidth=1, label="target_angles")
    plt.legend()
    plt.show()
    
plot_target_angles()