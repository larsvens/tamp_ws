#!/usr/bin/env python

# Description:
# This is a standalone script for computing the optimal gain matrix K 
# for the trajectory rollout controller

# Usage: 
# Adjust tuning matrices, params, and/or linearization state 
# run script, copy the K matrix and input into the saarti config script

import numpy as np
import control 
from scipy.linalg import expm

# define tuning matrices
Q = np.diag(np.array([1,100000000,1,1,10000000,1]))
R = np.diag(np.array([1,1,1]))

# misc system vars
N = 40
dt = 0.1

# static params
lf = 1.205
lr = 2.189
Iz = 8158
m = 8350.0 

# controls at init state
Fyf = 0.0
Fxf = 0.0
Fxr = 0.0

# additional vars for init state
Fyr = 0.0
kappac = 0.00
Cr = 100000.0

# init state
s = 0.0
d = 0.0
deltapsi = 0.0
psidot = 0.0
vx = 15.0
vy = 0.0

# linearization about init state (continous time)
Act = np.array([ 
[ 0,    (kappac*(vx*np.cos(deltapsi) - vy*np.sin(deltapsi)))/(d*kappac - 1)**2,           (vy*np.cos(deltapsi) + vx*np.sin(deltapsi))/(d*kappac - 1),                     0,               -np.cos(deltapsi)/(d*kappac - 1),           np.sin(deltapsi)/(d*kappac - 1)], \
[ 0,                                                                  0,                            vx*np.cos(deltapsi) - vy*np.sin(deltapsi),                     0,                               np.sin(deltapsi),                          np.cos(deltapsi)], \
[ 0, -(kappac**2*(vx*np.cos(deltapsi) - vy*np.sin(deltapsi)))/(d*kappac - 1)**2, -(kappac*(vy*np.cos(deltapsi) + vx*np.sin(deltapsi)))/(d*kappac - 1),                     1,       (kappac*np.cos(deltapsi))/(d*kappac - 1), -(kappac*np.sin(deltapsi))/(d*kappac - 1)], \
[ 0,                                                                  0,                                                              0,  -(2*Cr*lr**2)/(Iz*vx),      -(2*Cr*lr*(vy - lr*psidot))/(Iz*vx**2),                      (2*Cr*lr)/(Iz*vx)], \
[ 0,                                                                  0,                                                              0,                     0,                                           0,                                      0], \
[ 0,                                                                  0,                                                              0, (2*Cr*lr)/(m*vx) - vx, (2*Cr*(vy - lr*psidot))/(m*vx**2) - psidot,                         -(2*Cr)/(m*vx)]
])

Bct = np.array([
[     0,   0,   0], \
[     0,   0,   0], \
[     0,   0,   0], \
[ lf/Iz,   0,   0], \
[     0, 1/m, 1/m], \
[   1/m,   0,   0]
])


# discretize by matrix exponential
Adt = expm(dt*Act);
Bdt = np.dot(np.linalg.pinv(Act), np.dot(Adt-np.eye(6),Bct))


#
# Compute K
#

P_,L_,G_ = control.dare(Adt, Bdt, Q, R)
#K_, S_, E_ = control.lqr(A, B, Q, R)

# ricatti recursions 
#P = P_ # init P to Q
#for i in range(10000):
#    #inv = np.linalg.inv(R +np.linalg.multi_dot([B.transpose(),P,B]))
#    #Pnew = Q + 
#    inv = np.linalg.inv(R + np.dot(B.transpose(), np.dot(P,B)))
#    Pnew = Q + np.dot(A.transpose(),np.dot(P,A)) - np.dot(A.transpose(), np.dot(P,np.dot(B, np.dot(inv, np.dot(B.transpose(),np.dot(P,A))))))
#    print np.mean(Pnew-P)
#    P=Pnew

# compute feedback 
K = np.linalg.multi_dot([np.linalg.inv(R + np.linalg.multi_dot([Bdt.transpose(),P_,Bdt])),Bdt.transpose(),P_,Adt])


#
# Evaluate result
#

# define reference state 
s_ref = 0.0
d_ref = 0.0
deltapsi_ref = 0.0
psidot_ref = 0.0
vx_ref = 16.0
vy_ref = 0.0


delta_x = np.array([s-s_ref,d-d_ref,deltapsi-deltapsi_ref,psidot-psidot_ref,vx-vx_ref,vy-vy_ref])
u = np.dot(-K,delta_x)

# print K
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f},".format(x)})
print "K:  ", K


