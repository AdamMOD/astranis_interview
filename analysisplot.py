import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.pyplot as plt
from scipy import signal
import seaborn as sns
from matplotlib.widgets import Slider

sns.set_style("whitegrid")


b, k, m = 2.0, 3.0, 1.0
kp,ki,kd = 1, 1, 1

P = lambda s: 1/(m* s**2 + b * s + k)
C = lambda s: (kd * s ** 2 + kp * s + ki) / s

P = signal.TransferFunction([1], [m, b, k])
C = signal.TransferFunction([kd, kp, ki], [1, 0])

PC = signal.TransferFunction([kd, kp, ki], [m, b, k, 0])
PC11 = signal.TransferFunction([m, b, k, 0], [m, b+kd, k + kp, ki])
w, mag, phase = signal.bode(PC11)
freq = w / (2 * np.pi)

fig, axs = plt.subplots(2,1,sharex=True)

axs[0].semilogx(freq, mag)
axs[0].set_ylabel("Gain")
axs[0].set_ylim(-3,3)
axs[1].semilogx(freq, phase) 
axs[1].set_ylabel("Phase")
axs[1].set_ylim(-360,180)
axs[1].set_xlabel("Frequency")

wrange = 2 * np.pi * np.linspace(.1,1,100)

# The parametrized function to be plotted
def f(t, kp,ki,kd):
    #PC11 = signal.TransferFunction([m, b, k, 0], [m, b+kd, k + kp, ki])
    PC = signal.TransferFunction([kd, kp, ki], [m, b, k, 0])
    w, mag, phase = signal.bode(PC, w = wrange)
    freq = w / (2 * np.pi)
    return mag

def f2(t, kp,ki,kd):
    #PC11 = signal.TransferFunction([m, b, k, 0], [m, b+kd, k + kp, ki])
    PC = signal.TransferFunction([kd, kp, ki], [m, b, k, 0])
    w, mag, phase = signal.bode(PC, w = wrange)
    freq = w / (2 * np.pi)
    return phase

t = np.linspace(0, 1, 1000)

# Define initial parameters
init_kp = .775
init_ki = 1.12
init_kd = 2.45

# Create the figure and the line that we will manipulate
fig, axs = plt.subplots(2,1,sharex=True)
line1, = axs[0].semilogx(freq, f(t, init_kp, init_ki,init_kd), lw=2)
line2, = axs[1].semilogx(freq, f2(t, init_kp, init_ki,init_kd), lw=2)
axs[0].vlines(.25,-50,50,colors = 'r',linestyles="--")
axs[0].set_ylabel("Gain")
axs[0].set_ylim(-50,50)
axs[1].set_ylabel("Phase")
axs[1].set_xlabel("Frequency")
axs[1].set_ylim(-360,180)
axs[1].vlines(.25,-360,180,colors = 'r',linestyles="--")

# adjust the main plot to make room for the sliders
plt.subplots_adjust(left=0.1, bottom=0.35)

# Make a horizontal slider to control the frequency.
axfreq = plt.axes([0.25, 0.24, 0.65, 0.03])
kp_slider = Slider(
    ax=axfreq,
    label='Kp',
    valmin=0.1,
    valmax=10,
    valinit=init_kp,
)

# Make a horizontal slider to control the frequency.
axfreq = plt.axes([0.25, 0.175, 0.65, 0.03])
ki_slider = Slider(
    ax=axfreq,
    label='Ki',
    valmin=0.1,
    valmax=10,
    valinit=init_ki,
)

# Make a vertically oriented slider to control the amplitude
axamp = plt.axes([0.25, 0.1, 0.65, 0.03])
kd_slider = Slider(
    ax=axamp,
    label="Kd",
    valmin=0,
    valmax=10,
    valinit=init_kd,
)

# The function to be called anytime a slider's value changes
def update(val):
    line1.set_ydata(f(t, kp_slider.val, ki_slider.val, kd_slider.val))
    line2.set_ydata(f2(t, kp_slider.val, ki_slider.val, kd_slider.val))
    fig.canvas.draw_idle()


# register the update function with each slider
kp_slider.on_changed(update)
ki_slider.on_changed(update)
kd_slider.on_changed(update)



plt.show()