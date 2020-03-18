#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 30}

rc('font', **font)
rc('text', usetex=True)
rc('text.latex', preamble=r'\usepackage{amsmath}')

plt.close('all')
plt.rcParams.update({'font.size': 12})


# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 100 # default 100
plt.rcParams['figure.figsize'] = 8, 2.6

# set common params for all plots
linewidth = 2
markersize = 6
labelsize = 24
legendsize = 20
ticksize = 20


# 1 reduced mu turn
# 2 obs avoid wet
# 3 obs avoid dry
# 3 localmin
scenario = 4

# 0 plot stored run
# 1 plot latest run
plot_latest = 0

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
if(scenario == 4):
    filename = "explog_localmin_nonadaptive.npy"

log = np.load(filepath+filename,allow_pickle=True).item()
pathglobal_nonadapt = log["pathglobal"]
trajstar_nonadapt = log["trajstar"]
trajcl_nonadapt = log["trajcl"]
saartistatus_nonadapt = log["saartistatus"]

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
if(scenario == 4):
    filename = "explog_localmin_adaptive.npy"    
    
log = np.load(filepath+filename,allow_pickle=True).item()
pathglobal_adapt = log["pathglobal"]
trajstar_adapt = log["trajstar"]
trajcl_adapt = log["trajcl"]
saartistatus_adapt = log["saartistatus"]
# prep data
mu_adapt = np.interp(trajstar_adapt["s"],pathglobal_adapt["s"],pathglobal_adapt["mu"])
Ffmax_adapt = mu_adapt[0:-1]*trajstar_adapt["Fzf"]
Frmax_adapt = mu_adapt[0:-1]*trajstar_adapt["Fzr"]
Ff_adapt = np.sqrt(trajstar_adapt["Fxf"]**2+trajstar_adapt["Fyf"]**2)
Fr_adapt = np.sqrt(trajstar_adapt["Fxr"]**2+trajstar_adapt["Fyr"]**2)

# time axes
t = trajcl_nonadapt["t"]
t_pred = trajstar_nonadapt["t"]

if(scenario in [1,2,3]):

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

else:
#    f, ax = plt.subplots()
#    coll_idx = 18
#    ax.plot(trajcl_nonadapt["s"][0:coll_idx+1],trajcl_nonadapt["d"][0:coll_idx+1],'b')
#    #ax.plot(t[coll_idx],trajcl_nonadapt["d"][coll_idx],'bo')
#    ax.plot(trajcl_adapt["s"],trajcl_adapt["d"],'r')
#    ax.set_ylabel("$d$ (m)")
#    ax.set_xlabel("$s$ (m)")
#    ax.legend(["RTI","SARTI"])
#    plt.show()

    f2, ax1 = plt.subplots()
    coll_idx = 18
    ax1.plot(trajcl_nonadapt["s"][0:coll_idx+1],saartistatus_nonadapt["n_coll_free"][0:coll_idx+1],'b-',linewidth=linewidth)
    ax1.plot(trajcl_adapt["s"][0:coll_idx+1],saartistatus_adapt["n_coll_free"][0:coll_idx+1]/151.0,'r-',linewidth=linewidth) 
    ax1.set_ylabel("$R_{cf}$",fontsize=labelsize)
    ax1.set_xlabel("$s$ (m)",fontsize=labelsize)
    ax2 = ax1.twinx()
    ax2.plot(trajcl_adapt["s"][0:coll_idx+1],saartistatus_adapt["rollout_selected"][0:coll_idx+1],'ms',linewidth=linewidth ,markersize=markersize)
    #ax2.set_ylabel("rollout selected")
    ax1.legend(["RTI","SARTI"], loc='center right',prop={'size': legendsize})
    ax1.tick_params(axis='both', which='major', labelsize=ticksize)

    
    f2.tight_layout()
    
    ax2.set_yticks([0.0,1.0])
    labels = [item.get_text() for item in ax2.get_yticklabels()]
    labels[0] = "$\widetilde{\mathcal{T}}_t$"
    labels[1] = "$\hat{\mathcal{T}}'_t$"
    ax2.set_yticklabels(labels,fontsize=labelsize)
    
    plt.show()
    
# save as pdf
filepath = "/home/larsvens/ros/tamp__ws/src/saarti/common/logs/plots/"
if(scenario == 1):
    filename_fig = "reduced_mu_turn_plots.pdf"
if(scenario == 2):
    filename_fig = "reduced_mu_obs_avoid_plots.pdf"
if(scenario == 3):  
    filename_fig = "increased_mu_obs_avoid_plots.pdf"
if(scenario == 4):
    filename_fig = "local_min_plot.pdf"
plt.savefig(filepath + filename_fig) 


# plot sampling aug analysis
#if(scenario == 4):
#    #saartistatus = log["saartistatus"]
#    f1, ax0 = plt.subplots()
#    ax0.plot(trajcl_adapt["s"][0:coll_idx+1],saartistatus_adapt["rollout_selected"][0:coll_idx+1],'r-')
#    plt.show()





    
    