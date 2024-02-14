from solution import SOLUTION
import constants as c
import copy
import os
import time


class PARALLEL_HILL_CLIMBER:
    
    def __init__(self) -> None:
  
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        # time.sleep(0.1)
        
        
        self.parents = {}
        self.nextAvailableID = 0 
        for i in range(c.population_size):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1 
        

    def evolve(self):
        # 1st Generate and evaluate parent individual robot.
        self.Evaluate(self.parents)        
        
        for curr_generation in range(c.n_generations):
            self.Evolve_For_One_Generation()
    
    def Evaluate(self, solutions):
        for sol in solutions.values():
            sol.start_simulation("DIRECT")
            
        for sol in solutions.values():
            sol.wait_for_simulation_to_end()
            
    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()
        
    def Spawn(self):
        """
            Generate child offspring from a copy of the parent.
            Assign next available unique ID for each child
        """
        self.children = {}
        for key in self.parents.keys():
            self.children[key] = copy.deepcopy(self.parents[key])
            self.children[key].set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        """
            Mutate all child offsrprings based on an arbitrary mutation rule.
        """
        for i in self.children: #range(len(self.children)):
            self.children[i].mutate()

    def Select(self):
        """
            Arbitrary Selection function. When parent fitness is higher than child.
            Make the new parent to be the child.
            
            * NOTE:  In this case lower fitness is better *
        """
        for key in self.parents.keys(): 
            if self.parents[key].fitness > self.children[key].fitness:
                self.parents[key] = self.children[key]
            
    def Show_Best(self):
        """
            Select the best parent after evolution, mutation and selection.
            Then, simulate it.
        """
        best = self.parents[0]
        for parent in self.parents.values():
            if parent.fitness < best.fitness:
                best = parent

        best.start_simulation("GUI")
        # print(best.fitness)
            
    def Print(self):
        for key in self.parents.keys():
            print(f'\n\nParent: {self.parents[key].fitness}, Child: {self.children[key].fitness}\n\n')