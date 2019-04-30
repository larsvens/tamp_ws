#!/usr/bin/env python
# run "%matplotlib auto" in console to get plot window

import scipy.io as sio
import csv 
import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as interpolate

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 20, 10 # full screen

# load original data
filename = '/home/larsvens/ros/saasqp_ws/src/perception/data/pathglobal_th.mat'
mat = sio.loadmat(filename,struct_as_record=False, squeeze_me=True)
s = np.array(mat['path_global'].s)
psi_c = np.array(mat['path_global'].psi_c)        

# plot original data
fig, (a0,a1,a2) = plt.subplots(3, 1)
a0.plot(s,psi_c,'k.',label ='original data')

# separate in pieces (for 2pi flips)
idx_low = 0
idx_high = 0
psi_c_piecewise = []
s_piecewise = []
for i in range(psi_c.size-1):
    if(np.abs(psi_c[i+1] - psi_c[i]) > 0.1 ):
        # if single pt, remove
        if(np.abs(psi_c[i+2] - psi_c[i]) < np.pi ):
            print("removing single flip")
            psi_c[i+1] = psi_c[i]
        # otherwise make a piece
        else:
            print("making pieces")
            idx_high = i+1
            psi_c_piece = psi_c[idx_low:idx_high]
            psi_c_piecewise.append(psi_c_piece)
            s_piece = s[idx_low:idx_high]
            s_piecewise.append(s_piece)
            idx_low = i+1
                          
# add final piece
psi_c_piece = psi_c[idx_low:psi_c.size]
psi_c_piecewise.append(psi_c_piece)
s_piece = s[idx_low:psi_c.size]
s_piecewise.append(s_piece)

# plot pieces
#for j in range(len(psi_c_piecewise)):
#    a0.plot(s_piecewise[j],psi_c_piecewise[j],'.')
#plt.show

# shift pieces to make continous psi_c
for j in range(len(psi_c_piecewise)-1):
    while(psi_c_piecewise[j][-1] - psi_c_piecewise[j+1][0] > np.pi):
        psi_c_piecewise[j+1] = psi_c_piecewise[j+1] + 2*np.pi
        
    while(psi_c_piecewise[j][-1] - psi_c_piecewise[j+1][0] < -np.pi):
        psi_c_piecewise[j+1] = psi_c_piecewise[j+1] - 2*np.pi

# concatenate pieces
psi_c_cont = psi_c_piecewise[0]
for j in range(len(psi_c_piecewise)-1):
    psi_c_cont = np.concatenate((psi_c_cont,psi_c_piecewise[j+1]))        
#a0.plot(s,psi_c_cont,'k')
  

# downsample for smoother curve
step = 100
s_ds = s[0::step]
psi_c_ds = psi_c_cont[0::step]

# interpolate 
t, c, k = interpolate.splrep(s_ds, psi_c_ds, s=0, k=4)
psi_c_spl = interpolate.BSpline(t, c, k, extrapolate=True)

# compute derrivatives (compare with naive numerical)    
kappa_c_spl = psi_c_spl.derivative(nu=1)
kappaprime_c_spl = psi_c_spl.derivative(nu=2)

# put psi_c back on interval [-pi,pi]
psi_c_out = psi_c_spl(s)
for i in range(psi_c_out.size):
    while(psi_c_out[i] > np.pi):
        psi_c_out[i] = psi_c_out[i] -2*np.pi
    while(psi_c_out[i] < -np.pi):
        psi_c_out[i] = psi_c_out[i] +2*np.pi

kappa_c_out = kappa_c_spl(s)
kappaprime_c_out = kappaprime_c_spl(s)

# final plots for checking (magenta)
a0.plot(s, psi_c_out,'m.')
a1.plot(s, kappa_c_out,'m.')
a2.plot(s, kappaprime_c_out,'m.')

# put plot window on top
fig.canvas.manager.window.activateWindow()
fig.canvas.manager.window.raise_()

# save new global path file as .npy
pathglobal = {
        "X":                mat['path_global'].X,
        "Y":                mat['path_global'].Y,
        "s":                mat['path_global'].s,
        "psi_c":            psi_c_out,
        "kappa_c":          kappa_c_out,
        "kappaprime_c":     kappaprime_c_out,
        "theta_c":          mat['path_global'].theta_c,
        "phi_c":            mat['path_global'].phi_c,
        "dub":              mat['path_global'].dub,
        "dlb":              mat['path_global'].dlb
}

np.save('pathglobal.npy',  pathglobal)    

