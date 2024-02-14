import os
from hillclimber import HILLCLIMBER
import time
from parallel_hillclimber import PARALLEL_HILL_CLIMBER

phc = PARALLEL_HILL_CLIMBER()    
phc.evolve()
phc.Show_Best()

        
# hc = HILLCLIMBER()
# hc.evolve()
# # Show best individual
# hc.Show_Best()