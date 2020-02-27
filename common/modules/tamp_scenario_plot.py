#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
#plt.rc(usetex = True)
plt.rcParams.update({'font.size': 12})


# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 8, 5

# 1 reduced mu turn
# 2 obs avoid wet
# 3 obs avoid dry
scenario = 3

# 0 plot stored run
# 1 plot latest run
plot_latest = 1

plot_sa_analysis = 1


# load nonadaptive file and unpack
if(plot_latest):
    filepath = "/home/larsvens/ros/tamp__ws/src/saarti/common/logs/data_latest/"
else:
    filepath = "/home/larsvens/ros/tamp__ws/src/saarti/common/logs/data_for_plots/"    

if(scenario == 1):
    filename = "explog_reducedmuturn_nonadaptive.npy"
if(scenario == 2):  
    filename = "explog_popup_wet_nonadaptive.npy"
if(scenario == 3):
    filename = "explog_popup_dry_nonadaptive.npy"

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
if(scenario == 1):
    filename = "explog_reducedmuturn_adaptive.npy"
if(scenario == 2):  
    filename = "explog_popup_wet_adaptive.npy"
if(scenario == 3):
    filename = "explog_popup_dry_adaptive.npy"
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
axes[0,0].set_ylabel("$d$ (m)")
axes[0,0].legend(["not adapting","adapting"])
# vx
if(scenario == 1):
    axes[1,0].plot(t,trajcl_nonadapt["vx"],'b')
    axes[1,0].plot(t,trajcl_adapt["vx"],'r')
    axes[1,0].set_ylabel("$v_x$ (m/s)")
else:
    axes[1,0].plot(t,trajcl_nonadapt["deltapsi"],'b')
    axes[1,0].plot(t,trajcl_adapt["deltapsi"],'r')
    axes[1,0].set_ylabel("$\Delta \psi$ (rad)")
axes[1,0].set_xlabel("$t_{real}$ (s)")

# Ff
axes[0,1].plot(t_pred[0:-1],0.001*Ff_nonadapt,'b.')
axes[0,1].plot(t_pred[0:-1],0.001*Ffmax_nonadapt,'b--')
axes[0,1].plot(t_pred[0:-1],0.001*Ff_adapt,'r.')
axes[0,1].plot(t_pred[0:-1],0.001*Ffmax_adapt,'r--')
axes[0,1].set_ylabel("$F_f$ (kN)")
#axes[0,1].legend(["planned","boundary"])
# Fr
axes[1,1].plot(t_pred[0:-1],0.001*Fr_nonadapt,'b.')
axes[1,1].plot(t_pred[0:-1],0.001*Frmax_nonadapt,'b--')
axes[1,1].plot(t_pred[0:-1],0.001*Fr_adapt,'r.')
axes[1,1].plot(t_pred[0:-1],0.001*Frmax_adapt,'r--')
axes[1,1].set_ylabel("$F_r$ (kN)")
axes[1,1].set_xlabel("$t_{predicted}$ (s)")

plt.show()

# save as pdf
filepath = "/home/larsvens/ros/tamp__ws/src/saarti/common/logs/plots/"
if(scenario == 1):
    filename_fig = "reduced_mu_turn_plots.pdf"
if(scenario == 2):
    filename_fig = "reduced_mu_obs_avoid_plots.pdf"
if(scenario == 3):  
    filename_fig = "increased_mu_obs_avoid_plots.pdf"
plt.savefig(filepath + filename_fig) 


# plot sampling aug analysis (only available if adaptive)
if(plot_sa_analysis):
    saartistatus = log["saartistatus"]
    f2, ax0 = plt.subplots()
    ax0.plot(trajcl_adapt["s"],saartistatus["rollout_selected"],'*')
    plt.show()




    
    