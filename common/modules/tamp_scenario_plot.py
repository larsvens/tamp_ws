#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 8, 4

# load nonadaptive file and unpack
filepath = "/home/larsvens/ros/tamp__ws/src/saarti/common/logs/"
filename = "explog_latest_reducedmuturn_nonadaptive.npy"
log = np.load(filepath+filename,allow_pickle=True).item()
pathglobal_nonadapt = log["pathglobal"]
trajstar_nonadapt = log["trajstar"]
trajcl_nonadapt = log["trajcl"]
# prep data
mu_nonadapt = np.interp(trajstar_nonadapt["s"],pathglobal_nonadapt["s"],pathglobal_nonadapt["mu"])
Ffmax_nonadapt = mu_nonadapt[0:-1]*trajstar_nonadapt["Fzf"]
Frmax_nonadapt = mu_nonadapt[0:-1]*trajstar_nonadapt["Fzr"]
Ff_nonadapt = np.sqrt(trajstar_nonadapt["Fxf"]**2+trajstar_nonadapt["Fyf"]**2)
Fr_nonadapt = np.sqrt(trajstar_nonadapt["Fxr"]**2+trajstar_nonadapt["Fyr"]**2)

# load adaptive file and unpack
filepath = "/home/larsvens/ros/tamp__ws/src/saarti/common/logs/"
filename = "explog_latest_reducedmuturn_adaptive.npy"
log = np.load(filepath+filename,allow_pickle=True).item()
pathglobal_adapt = log["pathglobal"]
trajstar_adapt = log["trajstar"]
trajcl_adapt = log["trajcl"]
# prep data
mu_adapt = np.interp(trajstar_adapt["s"],pathglobal_adapt["s"],pathglobal_adapt["mu"])
Ffmax_adapt = mu_adapt[0:-1]*trajstar_adapt["Fzf"]
Frmax_adapt = mu_adapt[0:-1]*trajstar_adapt["Fzr"]
Ff_adapt = np.sqrt(trajstar_adapt["Fxf"]**2+trajstar_adapt["Fyf"]**2)
Fr_adapt = np.sqrt(trajstar_adapt["Fxr"]**2+trajstar_adapt["Fyr"]**2)

# time axes
t = trajcl_nonadapt["t"]
t_pred = trajstar_nonadapt["t"]

f, axes = plt.subplots(2, 2, sharex='col')
# d
axes[0,0].plot(t,trajcl_nonadapt["d"],'b')
axes[0,0].plot(t,trajcl_adapt["d"],'r')
axes[0,0].set_ylabel("d (m)")
axes[0,0].legend(["not adapting","adapting"])
# vx
axes[1,0].plot(t,trajcl_nonadapt["vx"],'b')
axes[1,0].plot(t,trajcl_adapt["vx"],'r')
axes[1,0].set_xlabel("t_real (s)")
axes[1,0].set_ylabel("vx (m/s)")
# Ff
axes[0,1].plot(t_pred[0:-1],0.001*Ff_nonadapt,'b.')
axes[0,1].plot(t_pred[0:-1],0.001*Ffmax_nonadapt,'b--')
axes[0,1].plot(t_pred[0:-1],0.001*Ff_adapt,'r.')
axes[0,1].plot(t_pred[0:-1],0.001*Ffmax_adapt,'r--')
axes[0,1].set_ylabel("Ff (kN)")
#axes[0,1].legend(["planned","boundary"])
# Fr
axes[1,1].plot(t_pred[0:-1],0.001*Fr_nonadapt,'b.')
axes[1,1].plot(t_pred[0:-1],0.001*Frmax_nonadapt,'b--')
axes[1,1].plot(t_pred[0:-1],0.001*Fr_adapt,'r.')
axes[1,1].plot(t_pred[0:-1],0.001*Frmax_adapt,'r--')
axes[1,1].set_ylabel("Fr (kN)")
axes[1,1].set_xlabel("t_predicted @ t_real=32 (s)")

plt.show()

# save as pdf
filepath = "/home/larsvens/ros/tamp__ws/src/saarti/common/logs/"
filename = "reduced_mu_turn_plots.pdf"
plt.savefig(filepath + filename) 


