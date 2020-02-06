import csv
import copy
import numpy as np

def desired_trajectory():
    _Y = []
    _t = []
    # _Y[:,0] = np.array([8*i for i in t])
    # _Y[:,1] = np.array([0 for i in range(len(t))])
    # _Y[:,2] = np.array([8 for i in range(len(t))])
    # _Y[:,3] = np.array([0 for i in range(len(t))])
    # _Y[:,4] = np.array([0 for i in range(len(t))])
    # _Y[:,5] = np.array([0 for i in range(len(t))])

    with open('Gait desired Final.csv') as csvfile:
        desired_gait = csv.reader(csvfile, delimiter=',')
        for row_num,row in enumerate(desired_gait):
            if row_num!=0:
                _t.append(float(row[0]))
                _Y.append([float(row[i+1]) for i in range(6)])

    return np.array(_t),np.array(_Y)

def gen_undesired_trajectory():
    t, desired_states = desired_trajectory()
    Y = copy.copy(desired_states)
    Y[:,1] = 0.15*(np.exp(-((t-0.2)/0.1)**2) - np.exp(-(0.2/0.1)**2))/5+desired_states[:,1]
    Y[:-1,3] = np.divide(Y[1:,1]-Y[0:-1,1],t[1:]-t[:-1])
    Y[:-1,5] = np.divide(Y[1:,3]-Y[0:-1,3],t[1:]-t[:-1])

    return Y