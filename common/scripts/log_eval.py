#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm


def ptsFrenetToCartesian(Xc,Yc,psic,d): # todo import from somewhere else
    # inputs and outputs are np arrays
    X = Xc - d*np.sin(psic);
    Y = Yc + d*np.cos(psic);
    return X,Y  


# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 10, 10

# load file
log_filepath = '/home/larsvens/ros/saasqp_ws/src/common/data/log_latest.npy'
log = np.load(log_filepath).item()

# unpack
tvec = np.array(log['tvec'])
N = len(tvec)

pathglobal = log['pathglobal']

states = log['states']
X_cl = np.zeros((N))
Y_cl = np.zeros((N))
vx_cl = np.zeros((N))

for i in range(N):
    state_i = states[i]
    X_cl[i] = state_i['X']
    Y_cl[i] = state_i['Y']
    vx_cl[i] = state_i['vx']


# remove the bit in the beginning before getting started
startidx = 50
X_cl = X_cl[startidx:-1]
Y_cl = Y_cl[startidx:-1]
vx_cl = vx_cl[startidx:-1]


# plot
fig, ax = plt.subplots()
ax.axis("equal")
ax.set_facecolor('lightgray')

# global path                 
llX, llY = ptsFrenetToCartesian(np.array(pathglobal['X']), \
                                np.array(pathglobal['Y']), \
                                np.array(pathglobal['psi_c']), \
                                np.array(pathglobal['dub']))
rlX, rlY = ptsFrenetToCartesian(np.array(pathglobal['X']), \
                                np.array(pathglobal['Y']), \
                                np.array(pathglobal['psi_c']), \
                                np.array(pathglobal['dlb']))
ax.set_xlim(min([min(llX),min(rlX)]), max([max(llX),max(rlX)]))
ax.set_ylim(min([min(llY),min(rlY)]), max([max(llY),max(rlY)]))
ax.plot(llX,llY,'k') 
ax.plot(rlX,rlY,'k') 


# closed loop trajectory
points = np.array([X_cl, Y_cl]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
norm = plt.Normalize(vx_cl.min(), vx_cl.max())
lc = LineCollection(segments, cmap='plasma', norm=norm)
lc.set_array(vx_cl)
lc.set_linewidth(4)
line = ax.add_collection(lc)
#ax.set_title('Line collection')
fig.colorbar(line)
plt.show()






