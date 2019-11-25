#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Todo put in notebook with picture for documentation
overlay on asta track import

@author: larsvens
"""

import numpy as np
import matplotlib.pyplot as plt

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 15, 5

# params
la = 300 # back of L shape
lc = 75  # base of L shape
Rb  = 5  # radius of curve at experiment turn
R = 5   # radius at other turns 
lanewidth = 3.5


# section a 
Na = la # default 1 node per meter
Xa = np.linspace(0,-la,Na)
Ya = np.zeros(Na)

# section b
Nb = np.floor(2*Rb*np.pi/4.0)
t = np.linspace(-np.pi/2,-np.pi,Nb)
Xb = Rb*np.cos(t) + Xa[-1]
Yb = Rb*np.sin(t) + Rb + Ya[-1]

# section c
Nc = lc
Xc = np.zeros(Nc) + Xb[-1]
Yc = np.linspace(Yb[-1],Yb[-1]+lc,Nc)

# section d
Nd = np.floor(R*np.pi) 
t = np.linspace(np.pi,0,Nd)
Xd = R*np.cos(t) + Xc[-1] + R
Yd = R*np.sin(t) + Yc[-1]

# section e
le = lc + Rb -3*R
Ne = le
Xe = np.zeros(Ne) + Xd[-1]
Ye = np.linspace(Yd[-1], Yd[-1]-le,Ne)

# section f
Nf = np.floor(2*R*np.pi/4.0)
t = np.linspace(-np.pi,-np.pi/2.0,Nf)
Xf = R*np.cos(t) + Xe[-1] + R
Yf = R*np.sin(t) + Ye[-1]

# section g
lg = la+Rb-3*R
Ng = lg
Xg = np.linspace(Xf[-1],Xf[-1]+lg,Ng)
Yg = np.zeros(Ng)+Yf[-1]

# section h
Nh = np.floor(R*np.pi) 
t = np.linspace(np.pi/2,-np.pi/2,Nh)
Xh = R*np.cos(t) + Xg[-1]
Yh = R*np.sin(t) + Yg[-1]-R

# concatenate vectors
X_cl = np.concatenate((Xa, Xb, Xc, Xd, Xe, Xf, Xg, Xh), axis=0)
Y_cl = np.concatenate((Ya, Yb, Yc, Yd, Ye, Yf, Yg, Yh), axis=0)

# remove duplicate points
cl_tmp = np.column_stack((X_cl,Y_cl))
cl = np.unique(cl_tmp, axis=0)

# compute s and psic
#dX = np.diff(fcl_X)
#dY = np.diff(fcl_Y)
#psic = np.arctan2(dY,dX)
#psic_final = np.arctan2(fcl_Y[0]-fcl_Y[-1],fcl_X[0]-fcl_X[-1])  
#psic = np.append(psic,psic_final) # assuming closed track

# get left and right boundaries
dlb = -lanewidth/2.0
dub = 3.0*lanewidth/2.0





plt.plot(X_cl,Y_cl,'*')
plt.plot(cl[:,0],cl[:,1],'.')
plt.gca().set_aspect('equal', adjustable='box')
plt.show()



