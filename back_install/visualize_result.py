import pandas as pd
import matplotlib.pyplot as plt
import os

path = './results'
file = os.listdir(path)
print(file)
"""dj111, dj112, distance btw owt and hull
    djsc11, djsc22 spar and hull"""
csv_2_read = ['attachH.csv', 'owt.csv', 'spar.csv']
owt = pd.read_csv(path + '/' + 'owt.csv')
spar = pd.read_csv(path + '/' + 'spar.csv')
owt_free = pd.read_csv(path + '/' + 'owt_woc.csv')
spar_free = pd.read_csv(path + '/' + 'spar_woc.csv')
owt_fc = pd.read_csv(path + '/' + 'owt_fc.csv')
spar_fc = pd.read_csv(path + '/' + 'spar_fc.csv')
owt_amp = pd.read_csv(path + '/' + 'owt_amp.csv')
spar_amp = pd.read_csv(path + '/' + 'spar_amp.csv')


hull_free = pd.read_csv(path + '/' + 'attachH_H2_free.csv')
hull_H8 = pd.read_csv(path + '/' + 'attachH_H8_free.csv')
owt_H8 = pd.read_csv(path + '/' + 'owt_H8_free.csv')
spar_H8 = pd.read_csv(path + '/' + 'spar_H8_free.csv')

hull_nolink = pd.read_csv(path + '/' + 'attachH_nolink.csv')
spar_nolink = pd.read_csv(path + '/' + 'spar_nolink.csv')


fig, axs = plt.subplots(3, 1)
axs[0].plot(hull_nolink['time'], hull_nolink['x'], label='w.o.t link')
axs[0].plot(hull_H8['time'], hull_H8['x'], label='with link')
axs[0].set_title('x')
axs[0].legend()
axs[1].plot(hull_nolink['time'], hull_nolink['y'], label='w.o.t link')
axs[1].plot(hull_H8['time'], hull_H8['y'], label='with link')
axs[1].set_title('y')
axs[1].legend()
axs[2].plot(hull_nolink['time'], hull_nolink['Rz'], label='w.o.t link')
axs[2].plot(hull_H8['time'], hull_H8['Rz'], label='with link')
axs[2].set_title('heading')
axs[2].legend()
# plt.legend()
plt.tight_layout()

dj111_H8_free = pd.read_csv(path + '/' + 'dj111_H8_free.csv')
dj111_amp = pd.read_csv(path + '/' + 'dj111_fc.csv')
fig2, axs = plt.subplots(3, 1)
axs[0].plot(dj111_H8_free['time'], dj111_H8_free['x'], label='w.o.t control')
axs[0].plot(dj111_amp['time'], dj111_amp['x'], label='with control')
axs[0].set_xlim()
axs[0].set_title('Fx')
axs[0].legend()
axs[1].plot(dj111_H8_free['time'], dj111_H8_free['y'], label='w.o.t control')
axs[1].plot(dj111_amp['time'], dj111_amp['y'], label='with control')
axs[1].set_title('Fy')
axs[1].legend()
# axs[2].plot(dj111_free['time'], dj111_free['z'], label='T=2')
axs[2].plot(dj111_H8_free['time'], dj111_H8_free['z'], label='w.o.t control')
axs[2].plot(dj111_amp['time'], dj111_amp['z'], label='with control')
axs[2].set_title('Fz')
axs[2].legend()
plt.tight_layout()
plt.show()

fig3, axs = plt.subplots(3, 1)
# axs[0].plot(dj111_free['time'], dj111_free['x'], label='T=2')
axs[0].plot(dj111_H8_free['time'], dj111_H8_free['x'], label='T=8')
axs[0].legend()
axs[0].set_title('Fx')
# axs[1].plot(dj111_free['time'], dj111_free['y'], label='T=2')
axs[1].plot(dj111_H8_free['time'], dj111_H8_free['y'], label='T=8')
axs[1].legend()
axs[1].set_title('Fy')
# axs[2].plot(dj111_free['time'], dj111_free['z'], label='T=2')
axs[2].plot(dj111_H8_free['time'], dj111_H8_free['z'], label='T=8')
axs[2].legend()
axs[2].set_title('Fz')
plt.tight_layout()
plt.show()

dz = owt['z'] - spar['z']
dz_free = owt_free['z'] - spar_free['z']
dz_fc = owt_fc['z'] - spar_fc['z']
dz_amp = owt_amp['z'] - spar_amp['z']
# plt.figure()
# plt.plot(owt['Ry'])
# plt.show()
plt.figure()
# plt.plot(owt['time'], owt['z'], label='owt')
plt.plot(owt_free['time'], owt_free['z'], label='owt_free')
plt.plot(owt_fc['time'], owt_fc['z'], label='owt_fc')
plt.plot(owt_amp['time'], owt_amp['z'], label='owt_amp')
plt.ylabel('z[m]')
plt.legend()

plt.figure()
# plt.plot(spar['time'], spar['z'], label='spar')
plt.plot(spar_free['time'], spar_free['z'], label='spar_free')
plt.plot(owt_free['time'], owt_free['z'], label='owt_free')
plt.plot(owt_amp['time'], owt_amp['z'], label='owt_ctr')
plt.axvline(x=100, color='r')
plt.ylabel('z[m]')
plt.legend()

plt.figure()
# plt.plot(owt['time'], dz, label='ctr')
plt.plot(owt_free['time'], dz_free, label='free')
plt.plot(owt_fc['time'], dz_fc, label='fc')
plt.plot(owt_amp['time'], dz_amp, label='amp')
plt.ylabel('distance')
plt.legend()

plt.show()
