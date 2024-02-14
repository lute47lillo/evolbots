from solution import SOLUTION
import constants as c
import copy


class HILLCLIMBER:
    
    def __init__(self) -> None:
        self.parent = SOLUTION()
        

    def evolve(self):
        # 1st Generate and evaluate parent individual robot.
        self.parent.evaluate("GUI")
        
        for curr_generation in range(c.n_generations):
            self.Evolve_For_One_Generation()
            
            
    def Evolve_For_One_Generation(self):
        self.Spawn()

        self.Mutate()

        self.child.evaluate("DIRECT")
        self.Print()

        self.Select()
        
    def Spawn(self):
        """
            Generate child offspring from a copy of the parent.
        """
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.mutate()
    
    def Select(self):
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child
            
    def Show_Best(self):
        self.parent.evaluate("GUI")
            
    def Print(self):
        print(f'\n\nParent: {self.parent.fitness}, Child: {self.child.fitness}\n\n')