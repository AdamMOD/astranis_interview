import numpy as np

class Actuator:
    def __init__(self, params: dict):
        self.params = params
    
    def update(self, u:float, t: float):
        pass

class Thruster(Actuator):
    def __init__(self, params: dict):
        super().__init__(
                params
            )
        self.F = params["thrust"]
        self.Tm = params["min_time"]

        self.last_on_time = -self.Tm
        self.thrust = 0
    
    def update(self, u: bool, t: float):
        if(t - self.last_on_time > self.Tm):
            if(u == 1 and self.thrust == 0):
                self.last_on_time = t
            self.thrust = u * self.F
 
        