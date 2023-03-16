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
print(owt.head())
dz = owt['z'] - spar['z']
plt.figure()
plt.plot(owt['z'], label='owt')
plt.plot(spar['z'], label='spar')
plt.ylabel('z[m]')
plt.legend()
plt.figure()
plt.plot(dz)
plt.ylabel('distance')
plt.show()
for f in csv_2_read:
    data = pd.read_csv(path + '/' + f)
    Var = f[:-4]
    cols = list(data.columns)
    n_cols = len(cols)
    n_row = int((n_cols-1)/3)
    fig, axs = plt.subplots(n_row, 3, figsize=(12, 3*n_row))

    for iRow in range(n_row):
        if n_row == 1:
            for iCol in range(3):
                axs[iCol].plot(data.iloc[:, 0], data.iloc[:, iCol + iRow * 3 + 1])
                axs[iCol].set_ylabel(cols[iCol + iRow * 3 + 1])
        else:
            for iCol in range(3):
                axs[iRow, iCol].plot(data.iloc[:, 0], data.iloc[:, iCol+iRow*3+1])
                axs[iRow, iCol].set_ylabel(cols[iCol+iRow*3+1])
    fig.suptitle(Var)
    fig.tight_layout()
plt.show()





