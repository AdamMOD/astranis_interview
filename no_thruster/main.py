import numpy as np
import scipy as sp
import scipy.integrate as integrate
import controllers
import sensors
import matplotlib.pyplot as plt
import seaborn as sns

sns.set_style("whitegrid")
plt.rcParams['font.size'] = '16'

#State is [x, xdot].t
x = np.array([0,0])
xsp = .2
xsp_hist = [xsp]
y = x[0]
x_hist = []
x_hist.append(x)
u_hist = [0]
y_hist = []
y_hist.append(x[0])

dt = .002
b, k, m = 2.0, 3.0, 1.0
A = np.array([[0, 1], [-k/m, -b/m]])
B = np.array([[0], [1/m]])

ctrl_params = {"Kp":  7, "Kd": 7, "Ki": 1, "alpha": .01}
ctrl_params = {"Kp":  10, "Kd": 0, "Ki": .5, "alpha": .01}
ctrl_params = {"Kp":  10, "Kd": 3, "Ki": 7, "alpha": .005}
#ctrl_params = {"Kp":  1, "Kd": 1, "Ki": 1, "alpha": 1}
pid = controllers.PID(ctrl_params)

sns_params = {"sense_period": .1, "delay_time": .01, "noise_variance": .01}
sensor = sensors.bad_sensor(sns_params)

time = np.arange(0,15,dt)

def dxdt(x, t, u):
    x = x.reshape(2, 1)
    dxdt = A @ x + B * u
    dxdt =  dxdt.reshape(2)
    return dxdt 

for t in time:
    #xsp = np.sin(2 * np.pi * .25 * t) * .1
    y = sensor.sense(x, t)
    u = pid.control(y, xsp, t)
    x = integrate.odeint(dxdt, x, [t, t+dt], args = (u,))[-1,:]
    x_hist.append(x)
    u_hist.append(u)
    y_hist.append(y)
    xsp_hist.append(xsp)

x_hist = np.array(x_hist)
fig, axs = plt.subplots(3,1,sharex=True)

axs[0].plot(time[1:], x_hist[1:-1,0])
axs[0].plot(time[1:], xsp_hist[1:-1])
axs[0].legend(["x (m)", "xsp (m)"])
axs[0].set_ylabel("State")
#axs[0].set_xlabel("Time (s)")


axs[1].plot(time[1:],u_hist[1:-1])
axs[1].legend(["u"])
axs[1].set_ylabel("Force (N)")
axs[1].set_xlabel("Time (s)")

axs[2].plot(time[1:], x_hist[1:-1,0])
axs[2].plot(time[1:],y_hist[1:-1])
axs[2].legend(["x","y"])
axs[2].set_ylabel("x (m)")
axs[2].set_xlabel("Time (s)")

plt.show()