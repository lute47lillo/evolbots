import pybullet as p
import pybullet_data
import time 

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

# Add gravity
p.setGravity(0,0,-9.8)

# Load the entities generated with pyrosim
planeId = p.loadURDF("plane.urdf") #Add a floor
p.loadSDF("boxes.sdf")

for i in range(0,1000):
    p.stepSimulation()
    time.sleep(0.01)
p.disconnect()