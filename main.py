import numpy as np
import scipy as sp
import scipy.integrate as integrate
import controllers
import sensors
import actuators
import matplotlib.pyplot as plt
import seaborn as sns

sns.set_style("whitegrid")

#State is [x, xdot].t
x = np.array([0,0])
xsp = 0
xsp_hist = [xsp]
y = x[0]
x_hist = []
x_hist.append(x)
u_hist = [0]
y_hist = []
y_hist.append(x[0])
F_hist = [0]
J_hist = [0]

dt = .002
b, k, m = 2.0, 3.0, 1.0
A = np.array([[0, 1], [-k/m, -b/m]])
B = np.array([[0], [1/m]])

ctrl_params = {"Kp": 1, "Kd": 1, "Ki": 1, "alpha": .1}
pid = controllers.PID(ctrl_params)


dt_mpc = .01
Adisc = np.array([[0.999850998740046,	0.00990016832418058],
        [-0.0297005049725417,	0.980050662091685]])

Bdisc = np.array([[4.96670866514088e-05],
        [0.00990016832418058]])
N = 30
min_fire_time = .05
mpc_crtl_params = {"A": Adisc, "B": Bdisc, "N": N, "min_fire_time": min_fire_time, "dt": dt_mpc}
mpc = controllers.MPC(mpc_crtl_params)

sns_params = {"sense_period": .1, "delay_time": .01, "noise_variance": .01}
sensor = sensors.Bad_sensor(sns_params)

thruster_params = {"thrust": 1, "min_time": min_fire_time}
thruster = actuators.Thruster(thruster_params)

tmax = 8
time = np.arange(0,tmax,dt)

def dxdt(x, t, u):
    x = x.reshape(2, 1)
    dxdt = A @ x + B * u
    dxdt =  dxdt.reshape(2)
    return dxdt 

for t in time:
    xsp = np.sin(2 * np.pi * .25 * t) * .1
    xsp = .2
    y = sensor.sense(x, t)
    #u = pid.control(y, xsp, t)

    mpc.do_control(x.reshape(2, 1), xsp, t)
    u = mpc.control
    #u = (pid.control(y, xsp, t)) > 0
    thruster.update(u, t)
    F = thruster.thrust
    #x = integrate.odeint(dxdt, x, [t, t+dt], args = (u,))[-1,:]
    x = integrate.odeint(dxdt, x, [t, t+dt], args = (F,))[-1,:]
    x_hist.append(x)
    u_hist.append(u)
    y_hist.append(y)
    xsp_hist.append(xsp)
    F_hist.append(F)
    J_hist.append(mpc.mincost)

x_hist = np.array(x_hist)
fig, axs = plt.subplots(3,1,sharex=True)

#At = [A * t for t in time] 
#expsol = np.array([a @ np.array([[1],[0]]) for a in np.exp(At)])
#[print(a) for a in np.exp(At)]
#print(expsol)

axs[0].plot(time, x_hist[:-1,0])
axs[0].plot(time, xsp_hist[:-1])
axs[0].legend(["x (m)", "xsp (m)"])
axs[0].set_ylabel("State")


axs[1].plot(time,u_hist[:-1])
axs[1].set_ylabel("Command")
#axs[1].set_xlabel("Time (s)")
#axs[1].vlines(np.arange(0,tmax,N*dt_mpc), np.min(x_hist[:-1,0]), np.max(x_hist[:-1,0]), colors = 'r',linestyles="--")


axs[1].plot(time, F_hist[:-1], ls="--", color="r")
#axs[1].set_xlabel("Time (s)")
axs[1].legend(["cmd", "thrust (n) "])


axs[2].plot(time, J_hist[:-1])
axs[2].set_ylabel("Cost")
axs[2].set_xlabel("Time (s)")
axs[2].legend(["J"])

"""
axs[2].plot(time, x_hist[:-1,0])
axs[2].plot(time,y_hist[:-1])
axs[2].legend(["x","y"])
axs[2].set_ylabel("x (m)")
axs[2].set_xlabel("Time (s)")
"""

plt.show()