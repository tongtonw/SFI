import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
from LiveLFilter import LiveLFilter

# read data, set data properties
data = pd.read_csv('data.csv')
deltaT = data['time'][1] - data['time'][0]
variable = 'x'  # set which variable from the data file we want to filter
Fs = 1 / deltaT  # sampling rate [Hz]
Ts = 1.0 / Fs  # sampling interval [s]
t = data['time']  # time vector
y = data[variable]

# plot the raw data
plt.figure()
plt.title('Original data')
plt.plot(t, y)
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.legend([variable])
plt.grid()

# get frequencies in signal. Used for finding the frequency you want to filter out. If you know it is related to
# the wave period ~8s or similar input that into the filter creation (scipy.)
n = len(y)  # length of the signal
k = np.arange(n)
T = n / Fs
frq = k / T  # two sides frequency range
frq = frq[:len(frq) // 2]  # one side frequency range
Y = np.fft.fft(y) / n  # dft and normalization
Y = Y[:n // 2]

# plot the frequencies
freq_plot_lim = 0.05
fig0, ax = plt.subplots(2, 1)
fig0.suptitle('frequencies in signal')
ax[0].plot(frq, abs(Y))  # plotting the spectrum
ax[0].set_xlabel('Freq (Hz)')
ax[0].set_ylabel('|Y(freq)|')
ax[0].set_xlim([0, freq_plot_lim])
ax[0].set_ylim([0, 5])
ax[1].plot(1 / (frq), abs(Y))  # plotting the spectrum
ax[1].set_xlabel('Period (s)')
ax[1].set_ylabel('|Y(freq)|')
ax[1].set_xlim([0, 1 / freq_plot_lim])
ax[1].set_ylim([0, 1])

# create notch filter
######## try to detect frequency we want to filter ##########
# f0 = np.abs(Y[np.argmax(np.abs(Y[1:]),axis=0)]) # locate frequency with highest energy -> we want to filter it out
######## manually set frequency we want to filter ##########
f0 = 0.11  # frequency we want to filter out [Hz]
Q = 1  # Quality factor

# Design notch filter
b, a = signal.iirnotch(f0, Q, Fs)

# Frequency response
freq, h = signal.freqz(b, a, fs=Fs)

# Plot
fig, ax = plt.subplots(2, 1, figsize=(8, 6))
ax[0].plot(freq, 20 * np.log10(abs(h)), color='blue')
ax[0].set_title("Frequency Response")
ax[0].set_ylabel("Amplitude (dB)", color='blue')
ax[0].set_xlim([0, 1])
ax[0].set_ylim([-20, 2])
ax[0].grid(True)
ax[1].plot(freq, np.unwrap(np.angle(h)) * 180 / np.pi, color='green')
ax[1].set_ylabel("Angle (degrees)", color='green')
ax[1].set_xlabel("Frequency (Hz)")
ax[1].set_xlim([0, 1])
ax[1].set_yticks([-90, -60, -30, 0, 30, 60, 90])
ax[1].set_ylim([-90, 90])
ax[1].grid(True)

# apply filter to live data
livefilter = LiveLFilter(b, a)
x_filt = []
x_raw = []
for i in range(0, n):
    print(str(i))
    t = data['time'][i]
    x = data[variable][i]
    x_filt.append(livefilter(x))
    x_raw.append(x)

# plot filtered vs raw data
plt.figure()
plt.suptitle('Raw vs filtered signal')
plt.plot(data['time'], x_raw)
plt.plot(data['time'], x_filt)
plt.legend(['raw ' + variable, 'filtered ' + variable])
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.grid()

plt.show()
