
from re import L
import numpy as np
import itertools

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
        self.derivative = self.calc_derivative()
        self.integral = self.calc_integral()
    
    def _update(self, x: float, xsp: float, t: float):
        self.x = x
        self.xsp = xsp
        self.time = t
        self.proportional = self.calc_proportional()
        self.integral = self.calc_integral()
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
        d = (self.proportional - self.prev_proportional)
        return d * self.alpha + self.prev_derivative * (1 - self.alpha)

    def calc_integral(self)-> float:
        return self.integral + (self.time - self.prev_time) * self.proportional

class MPC(Controller):
    def __init__(self, params: dict):
        super().__init__(
                params
            )
        self.A = params["A"]
        self.B = params["B"]
        self.dt = params["dt"]
        self.min_fire_time = params["min_fire_time"]
        self.N = params["N"]
        self.list_of_Apows = []
        self._generate_Apows()
        self.list_of_ABsums = [self.list_of_Apows[0] @ self.B]
        self._generate_ABsums()


        self.control_last_time = 0
        self.good_actions = [0, self.N]

        self.possible_actions = []
        self._generate_possible_actions()

        self.actions = [0,0]
        self.control = 0
        self.last_time_control = -1
        self.mincost = 0

    def J(self, x0, x_ref, N0, N1):
        first_term = 0
        for n in range(1, N1+1):
            xf = self.list_of_Apows[n] @ x0
            first_term += -2 * x_ref * xf[0] + xf[0]**2
        
        second_term = 0
        xf_first = self.list_of_Apows[N1] @ x0
        for n in range(1, N0+1):
            xf = self.list_of_Apows[n] @ xf_first + self.list_of_ABsums[n]
            second_term += -2 * x_ref * xf[0] + xf[0]**2
        return 1 * ( first_term + second_term + x_ref**2 * (N1 + N0))

    def _generate_ABsums(self):
        for n in range(1, self.N+1):
            self.list_of_ABsums.append(self.list_of_ABsums[-1] + self.list_of_Apows[n] @ self.B)

    def _generate_Apows(self):
        for n in range(0, self.N+1):
            self.list_of_Apows.append(np.linalg.matrix_power(self.A, n))

    def _generate_possible_actions(self):
        #[[N0,N1], [N0,N1], ...]
        #for now, restrict it to a linear space for easy solution finding
        a = np.arange(0,self.N+1)
        b = np.arange(self.N,-1, -1)
        a = np.reshape(a, (self.N+1, 1))
        b = np.reshape(b, (self.N+1, 1))

        possible_actions = np.concatenate((a, b),axis=1)

        if(self.control_last_time == 0):
            possible_actions = possible_actions[possible_actions[:,0] >= 5]

        self.possible_actions = possible_actions


    def find_optimal_control_times(self, x, xref):
        self.costs = []
        self.costs = [self.J(x, xref, N0, N1) for [N0,N1] in self.possible_actions]
        self.costs = np.array(self.costs)
        self.costs = self.costs.flatten()
        good_index = np.argmin(self.costs)
        self.good_actions = self.possible_actions[good_index]
        self.mincost = self.costs[good_index]
        #print(self.costs)
        #print(self.good_actions)


    def do_control(self, x, xref, t):
        if(t - self.last_time_control >= self.N * self.dt):
            self.control_last_time = self.control
            self._generate_possible_actions()
            self.find_optimal_control_times(x, xref)
            self.last_time_control = t
        self.control =  1
        if(t - self.last_time_control < self.good_actions[1] * self.dt):
            self.control = 0