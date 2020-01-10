#!/usr/bin/env python

import numpy as np
#import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from coordinate_transforms import ptsFrenetToCartesian

from matplotlib.collections import LineCollection
#from matplotlib.colors import ListedColormap, BoundaryNorm



def getcolorlineXYvx(x,y,vx):
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = plt.Normalize(vx.min(), vx.max())
    lc = LineCollection(segments, cmap='plasma', norm=norm)
    lc.set_array(vx)
    lc.set_linewidth(3)
    return lc



# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 20, 10

# load file and unpack
filename = "/home/larsvens/ros/tamp__ws/src/saarti/common/logs/explog_latest.npy" # todo get from param
log = np.load(filename,allow_pickle=True).item()
pathglobal = log["pathglobal"]
trajstar = log["trajstar"]
trajcl = log["trajcl"]

trajcl["t"] = trajcl["t"]-trajcl["t"][0]

## plot


f0 = plt.figure(constrained_layout=True)
gs = gridspec.GridSpec(ncols=2, nrows=3, figure=f0)
#gs = f0.add_gridspec(3, 2)
f0_ax0 = f0.add_subplot(gs[:, 0])
f0_ax0.set_title("top view")
f0_ax1 = f0.add_subplot(gs[0, 1])
f0_ax1.set_title("psi")
f0_ax2 = f0.add_subplot(gs[1, 1])
f0_ax2.set_title("vx")
f0_ax3 = f0.add_subplot(gs[2, 1])
f0_ax3.set_title("ax")

#
# OVERHEAD VIEW
#

# global path    
Xll,Yll = ptsFrenetToCartesian(np.array(pathglobal['s']), \
                                 np.array(pathglobal['dub']), \
                                 np.array(pathglobal['X']), \
                                 np.array(pathglobal['Y']), \
                                 np.array(pathglobal['psi_c']), \
                                 np.array(pathglobal['s']))
Xrl,Yrl = ptsFrenetToCartesian(np.array(pathglobal['s']), \
                                 np.array(pathglobal['dlb']), \
                                 np.array(pathglobal['X']), \
                                 np.array(pathglobal['Y']), \
                                 np.array(pathglobal['psi_c']), \
                                 np.array(pathglobal['s']))

f0_ax0.plot(Xll,Yll,'k') 
f0_ax0.plot(Xrl,Yrl,'k') 

# planned traj
#f0_ax0.plot(trajstar["X"],trajstar["Y"],'b',linewidth=3.0)
line_trajstar = f0_ax0.add_collection(getcolorlineXYvx(trajstar["X"],trajstar["Y"],trajstar["vx"]))

# closed loop traj
line_trajcl = f0_ax0.add_collection(getcolorlineXYvx(trajcl["X"],trajcl["Y"],trajcl["vx"]))
f0.colorbar(line_trajcl,ax=f0_ax0)

# settings plotwindow
f0_ax0.axis("equal")
f0_ax0.set_facecolor('lightgray')
f0_ax0.set_xlim(min([min(trajcl["X"]),min(trajstar["X"])]), max([max(trajcl["X"]),max(trajstar["X"])]))
f0_ax0.set_ylim(min([min(trajcl["Y"]),min(trajstar["Y"])]), max([max(trajcl["Y"]),max(trajstar["Y"])]))
f0_ax0.set_xlabel("X")
f0_ax0.set_ylabel("Y")

#
# PSI
#
f0_ax1.plot(trajstar["t"],trajstar["psi"],'m--')
f0_ax1.plot(trajcl["t"],trajcl["psi"],'k')
f0_ax1.legend(["planned","actual"])

#
# vx
#
f0_ax2.plot(trajstar["t"],trajstar["vx"],'m--')
f0_ax2.plot(trajcl["t"],trajcl["vx"],'k')
f0_ax2.legend(["planned","actual"])

#
# ax
#
dt = 0.1
ax_star = np.diff(trajstar["vx"])/dt
ax_cl = np.diff(trajcl["vx"])/dt
f0_ax3.plot(trajstar["t"][0:-1],ax_star,'m--')
f0_ax3.plot(trajcl["t"][0:-1],ax_cl,'k')
f0_ax3.legend(["planned","actual"])


plt.show()




