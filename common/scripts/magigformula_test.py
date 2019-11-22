#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 22 12:11:40 2019

@author: larsvens
"""

import numpy as np
import matplotlib.pyplot as plt

plt.clf() 

alpha = np.linspace(-1,1,100)


# rajamani formula
# y = D * np.sin(C*np.arctan(B*alpha - E*(B*alpha - np.arctan(B * alpha))));
# and fssim formula
# y = D*np.sin(C*np.arctan(B*(1.0-E)*alpha+E*np.arctan(B*alpha)));
# are equivalent


# std params (dry)
B = 10.0
C = 1.9
D = -1.0
E = 0.97
y = D * np.sin(C*np.arctan(B*alpha - E*(B*alpha - np.arctan(B * alpha))));
plt.plot(alpha,y)

# gottard params
B = 12.56
C = -1.38
D = 1.60
E = -0.58
y = D * np.sin(C*np.arctan(B*alpha - E*(B*alpha - np.arctan(B * alpha))));
plt.plot(alpha,y)

plt.show()

