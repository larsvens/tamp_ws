#!/usr/bin/env python

import numpy as np 
import matplotlib.pyplot as plt

from coordinate_transforms import ptsFrenetToCartesian
from coordinate_transforms import ptsCartesianToFrenet


# preprocessong of path
pathglobal_filepath = '/home/larsvens/ros/tamp_ws/src/perception/data/pathglobal.npy'
pathglobal_npy = np.load(pathglobal_filepath)
pathglobal = pathglobal_npy.item()

start = 500
stop = 1500

Xpath = pathglobal['X'][start:stop]
Ypath = pathglobal['Y'][start:stop]
psipath = pathglobal['psi_c'][start:stop]
spath = pathglobal['s'][start:stop]

# define pts in s and d
s_in = 270
d_in = -15


# transform frenet --> cart 
X,Y = ptsFrenetToCartesian(s_in,d_in,Xpath,Ypath,psipath,spath)

# transform cart --> frenet
s,d = ptsCartesianToFrenet(X,Y,Xpath,Ypath,psipath,spath)


# check results
print
print 's in          ', s_in
print 's out         ', s
print 's error       ', s_in-s

print 
print 'd in          ', d_in
print 'd out         ', d
print 'd error       ', d_in-d


f, ax = plt.subplots(1, 1)
ax.plot(Xpath,Ypath,'k')
#ax.plot(Xc,Yc,'og')
ax.plot(X,Y,'*r')
ax.axis('equal')
plt.show() 


