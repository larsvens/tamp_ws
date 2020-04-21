#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 20 17:51:36 2020

@author: larsvens
"""


import numpy as np
import matplotlib.pyplot as plt

# state
X = -11.930850
Y =  15.874927
psi = 0.426402

# trajstar
Xtrajstar = np.array([-12.746880, -11.713000, -10.679102, -9.712725, -8.808464, -7.942084, -7.113467, -6.339257, -5.634152, -5.005043, -4.448593])
Ytrajstar = np.array([15.818353, 15.877280, 15.662700, 15.265669, 14.792006, 14.270150, 13.694577, 13.058767, 12.361351, 11.606265, 10.801876])

# rotate trajstar to vehicle frame xy
deltaX = Xtrajstar-X
deltaY = Ytrajstar-Y
x = deltaX*np.cos(psi) + deltaY*np.sin(psi)
y = -deltaX*np.sin(psi) + deltaY*np.cos(psi)

#x = np.arange(10)
#y = np.array([0, 0.1, 0.2, 0.5, 1.0, 1.5, 2.0, 2.4 , 3.0, 3.2])

# fit 3rd order polynomial
X_poly = np.vstack((x ** 3, x ** 2, x ** 1))
poly_coeffs = np.linalg.lstsq(X_poly.T, y,rcond=None)[0]
y_fit = np.dot(poly_coeffs, X_poly)
kappa_0 = 2*poly_coeffs[1]/((1+poly_coeffs[2])**1.5)

# fit circle seg



X_b = np.c_[x ** 3, x ** 2, x]
theta_best = np.linalg.inv(X_b.T.dot(X_b)).dot(X_b.T).dot(y)


print kappa_0

plt.close('all')
plt.plot(x,y,'C0*')
plt.plot(x,y_fit,'C1')
plt.show()