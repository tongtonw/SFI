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
owt_free = pd.read_csv(path+'/' + 'owt_woc.csv')
spar_free = pd.read_csv(path+'/' + 'spar_woc.csv')
owt_fc = pd.read_csv(path+'/'+'owt_fc.csv')
spar_fc = pd.read_csv(path+'/'+'spar_fc.csv')
# print(owt.head())
dz = owt['z'] - spar['z']
dz_free = owt_free['z'] - spar_free['z']
dz_fc = owt_fc['z']-spar_fc['z']
# plt.figure()
# plt.plot(owt['Ry'])
# plt.show()
plt.figure()
# plt.plot(owt['time'], owt['z'], label='owt')
plt.plot(owt_free['time'], owt_free['z'], label='owt_free')
plt.plot(owt_fc['time'], owt_fc['z'], label='owt_fc')
plt.ylabel('z[m]')
plt.legend()

# plt.figure()
# plt.plot(spar['time'], spar['z'], label='spar')
# plt.plot(spar_free['time'], spar_free['z'], label='spar_free')
# plt.ylabel('z[m]')
# plt.legend()

plt.figure()
# plt.plot(owt['time'], dz, label='ctr')
plt.plot(owt_free['time'], dz_free, label='free')
plt.plot(owt_fc['time'],dz_fc,label='fc')
plt.ylabel('distance')
plt.legend()
# for f in csv_2_read:
#     data = pd.read_csv(path + '/' + f)
#     Var = f[:-4]
#     cols = list(data.columns)
#     n_cols = len(cols)
#     n_row = int((n_cols-1)/3)
#     fig, axs = plt.subplots(n_row, 3, figsize=(12, 3*n_row))
#
#     for iRow in range(n_row):
#         if n_row == 1:
#             for iCol in range(3):
#                 axs[iCol].plot(data.iloc[:, 0], data.iloc[:, iCol + iRow * 3 + 1])
#                 axs[iCol].set_ylabel(cols[iCol + iRow * 3 + 1])
#         else:
#             for iCol in range(3):
#                 axs[iRow, iCol].plot(data.iloc[:, 0], data.iloc[:, iCol+iRow*3+1])
#                 axs[iRow, iCol].set_ylabel(cols[iCol+iRow*3+1])
#     fig.suptitle(Var)
#     fig.tight_layout()
plt.show()





