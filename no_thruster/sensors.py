import numpy as np

class Sensor:
    def __init__(self, params: dict):
        self.params = params
    
    def update(self, x:float):
        pass

    def sense(self, x: float):
        return x

class bad_sensor(Sensor):
    def __init__(self, params: dict):
        super().__init__(
                params
            )
        self.Ts = params["sense_period"]
        self.Td = params["delay_time"]
        self.v = params["noise_variance"]
        self.x_hist = np.array([[0, 0]])
        self.t_last_measurement = -1 * self.Ts
        self.last_measurement = 0

    def _update(self, x:float, t: float):
        self.x_hist = np.append(self.x_hist, [[t, x[0]]], axis= 0)
        if(self.x_hist[-1,0] - self.x_hist[0,0] > self.Td):
            self.x_hist = np.delete(self.x_hist, 0, 0)
        
    def sense(self, x: float, t: float):
        self._update(x, t)
        if(t - self.t_last_measurement >= self.Ts):
            self.last_measurement = np.random.choice(self.x_hist[:,1]) + np.random.normal(scale=self.v)
            self.t_last_measurement = t
        return self.last_measurement