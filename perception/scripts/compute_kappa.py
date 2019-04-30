#!/usr/bin/env python
# run "%matplotlib auto" in console to get plot window

import scipy.io as sio
import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as interpolate

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 20, 10 # full screen

filename = '/home/larsvens/ros/saasqp_ws/src/perception/data/pathglobal_th.mat'
mat = sio.loadmat(filename,struct_as_record=False, squeeze_me=True)
        
pathglobal =	{
  "X":          mat['path_global'].X,
  "Y":          mat['path_global'].Y,
  "s":          mat['path_global'].s,
  "psi_c":      mat['path_global'].psi_c,
  "theta_c":    mat['path_global'].theta_c,
  "phi_c":      mat['path_global'].phi_c,
  "dub":        mat['path_global'].dub,
  "dlb":        mat['path_global'].dlb
}

# plot original data
s = np.array(pathglobal['s'])
psi_c = np.array(pathglobal['psi_c'])
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
for j in range(len(psi_c_piecewise)):
    a0.plot(s_piecewise[j],psi_c_piecewise[j],'.')
plt.show

# downsample for smoother curve
step = 10
s_ds = s[0::step]
for j in range(len(psi_c_piecewise)):
    s_ds = s_piecewise[j][0::step]
    psi_c_ds = psi_c_piecewise[j][0::step]
    t, c, k = interpolate.splrep(s_ds, psi_c_ds, s=0, k=4)
    psi_c_spl = interpolate.BSpline(t, c, k, extrapolate=False)
    a0.plot(s_piecewise[j], psi_c_spl(s_piecewise[j]),'m')
    # compute derrivatives (compare with naive numerical)    
    kappa_c_spl = psi_c_spl.derivative(nu=1)
    kappaprime_c_spl = psi_c_spl.derivative(nu=2)
    a1.plot(s_piecewise[j], kappa_c_spl(s_piecewise[j]),'m')
    a2.plot(s_piecewise[j], kappaprime_c_spl(s_piecewise[j]),'m')







# save to file .npy? csv? 


