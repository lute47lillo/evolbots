"""
Author: Eleuterio Juan Lillo Portero

File that contains all variables used across the rest of the files.
"""
import numpy as np

# Plot ranges for oscillations

# Define the desired range
new_min = -np.pi/4
new_max = np.pi/4

# Original range of numpy.sin function is [-1, 1]
old_min = -1
old_max = 1

# Sensors
backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)

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