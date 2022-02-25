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
w, mag, phase = signal.bode(PC)

f = w / (2 * np.pi)

fig, axs = plt.subplots(2,1,sharex=True)

axs[0].semilogx(f, mag)
axs[0].set_ylabel("Gain")
axs[1].semilogx(f, phase) 
axs[1].set_ylabel("Phase")
axs[1].set_xlabel("Frequency")