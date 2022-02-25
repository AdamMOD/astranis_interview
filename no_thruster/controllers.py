
import numpy as np

class Controller:
    def __init__(self, params: dict):
        self.params = params

    def control(self, x: float, xsp: float):
        return 0

class PID(Controller):
    def __init__(self, params: dict):
        super().__init__(
                params
            )
        self.Ki = params["Ki"]
        self.Kp = params["Kp"]
        self.Kd = params["Kd"]
        self.alpha = params["alpha"]

        self.x = 0
        self.xsp = 0
        self.time = 0
        self.integral = 0
        self.derivative = 0
        self.prev_derivative = 0
        self.prev_proportional= 0
        self.prev_time = 0

        self.proportional = self.calc_proportional()
        #self.derivative = self.calc_derivative()
        self.integral = self.calc_integral()
    
    def _update(self, x: float, xsp: float, t: float):
        self.x = x
        self.xsp = xsp
        self.time = t
        if(self.time == 0):
            self.prev_proportional = self.calc_proportional()
        self.proportional = self.calc_proportional()
        self.integral = self.calc_integral()
        if(self.time > 0):
            self.derivative = self.calc_derivative()
        self.prev_proportional = self.proportional
        self.prev_derivative = self.derivative
        self.prev_time = self.time

    def control(self, x: float, xsp:float, t: float) -> float:
        self._update(x, xsp, t)
        return self.proportional * self.Kp + self.derivative * self.Kd + self.integral * self.Ki
    
    def calc_proportional(self) -> float:
        return self.xsp - self.x

    def calc_derivative(self)-> float:
        d = (self.proportional - self.prev_proportional) / (self.time - self.prev_time)
        return d * self.alpha + self.prev_derivative * (1 - self.alpha)

    def calc_integral(self)-> float:
        return self.integral + (self.time - self.prev_time) * self.proportional

#class Sensor: