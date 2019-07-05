#!/usr/bin/env python

import numpy as np 
import matplotlib.pyplot as plt

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
N = Xpath.size

# define pt in s and d
s_in = 270
d_in = -15

Xc = np.interp(s_in,spath,Xpath)
Yc = np.interp(s_in,spath,Ypath)
psic = np.interp(s_in,spath,psipath) 

X = Xc - d_in*np.sin(psic);
Y = Yc + d_in*np.cos(psic);

#tmp
#X = 0
#Y = -220


##### function start ######

# find closest pt on centerline
mindist = 1000000
for i in range(N):
    dist = np.sqrt((X-Xpath[i])**2 + (Y-Ypath[i])**2 )
    if (dist<mindist):
        mindist = dist
        idx = i
        
Xc = Xpath[idx] 
Yc = Ypath[idx] 
psic = psipath[idx] 
sc = spath[idx] 

# compute angles etc
deltaX = X - Xc
deltaY = Y - Yc
alpha1 = -np.arctan2(deltaX,deltaY)
alpha2 = psic - alpha1
M = np.sqrt(deltaX**2 + deltaY**2)
deltas = M*np.sin(alpha2)
d = M*np.cos(alpha2)
s = sc + deltas


###### return #### 
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
ax.plot(Xc,Yc,'og')
ax.plot(X,Y,'*r')
ax.axis('equal')
plt.show() 


