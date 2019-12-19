#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  9 11:49:03 2019

@author: larsvens
"""

import numpy as np
import matplotlib.pyplot as plt

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 10, 10


# params

mu_dry = 1.0
mu_wet = 0.8
mu_snow = 0.3

g = 9.82
ax_acc = g/2
ax_constspeed = 0
ax_brake = -g/2 

mu = np.array([mu_dry, mu_wet, mu_snow])
ax = np.array([ax_acc, ax_constspeed, ax_brake])

lf = 1.1936
lr = 1.7904
m = 2900
h_cg = 0.75
theta = 0

# for i
Fzf = np.zeros(3)
Fzr = np.zeros(3)
for i in range(ax.size):
    Fzf[i] = (1.0/(lf+lr))*(m*ax[i]*h_cg - m*g*h_cg*np.sin(theta) + m*g*lr*np.cos(theta));
    Fzr[i] = (1.0/(lf+lr))*(-m*ax[i]*h_cg + m*g*h_cg*np.sin(theta) + m*g*lf*np.cos(theta));





# configure plot window
cols = ['Front', 'Rear']
rows = ['Dry', 'Wet', 'Snow']

fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(6.2, 8),subplot_kw={'aspect': 'equal'})
plt.setp(axes[:,0].flat, xlabel='Fxf', ylabel='Fyf')
plt.setp(axes[:,1].flat, xlabel='Fxr', ylabel='Fyr')
plt.ticklabel_format(axis='y', style='sci')



pad = 5 # in points

for ax, col in zip(axes[0], cols):
    ax.annotate(col, xy=(0.5, 1), xytext=(0, pad),
                xycoords='axes fraction', textcoords='offset points',
                size='large', ha='center', va='baseline')

for ax, row in zip(axes[:,0], rows):
    ax.annotate(row, xy=(0, 0.5), xytext=(-ax.yaxis.labelpad - pad, 0),
                xycoords=ax.yaxis.label, textcoords='offset points',
                size='large', ha='right', va='center')

fig.tight_layout()
# tight_layout doesn't take these labels into account. We'll need 
# to make some room. These numbers are are manually tweaked. 
# You could automatically calculate them, but it's a pain.
fig.subplots_adjust(left=0.15, top=0.95)


patterns = ['-','.','x'] # , hatch=3*patterns[j]
linestyles = ['--','-',':']
colors = ['whitesmoke','gainsboro','darkgray']

# fill plots
for i in range(mu.size):
    for j in range(Fzf.size):
        Ff = mu[i]*Fzf[j]
        Fr = mu[i]*Fzr[j]
        theta = np.linspace(0, 2*np.pi, 100)
        Fxf = Ff*np.cos(theta)
        Fyf = Ff*np.sin(theta)
        Fxr = Fr*np.cos(theta)
        Fyr = Fr*np.sin(theta)
        axes[i,0].fill(Fxf,Fyf,facecolor=colors[j], edgecolor='black', linestyle=linestyles[j], linewidth=1.5)
        #axes[i,1].fill(Fxr,Fyr,facecolor=colors[j], edgecolor='black', linestyle=linestyles[j], linewidth=1.5)
    for j in reversed(range(Fzf.size)):
        Ff = mu[i]*Fzf[j]
        Fr = mu[i]*Fzr[j]
        theta = np.linspace(0, 2*np.pi, 100)
        Fxf = Ff*np.cos(theta)
        Fyf = Ff*np.sin(theta)
        Fxr = Fr*np.cos(theta)
        Fyr = Fr*np.sin(theta)
        axes[i,1].fill(Fxr,Fyr,facecolor=colors[j], edgecolor='black', linestyle=linestyles[j], linewidth=1.5)
# adjust ranges
plt.setp(axes, xlim=axes[0,0].get_xlim(), ylim=axes[0,0].get_ylim())

# scientific notation
for ax in axes.flat:
    ax.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
    ax.ticklabel_format(style='sci', axis='x', scilimits=(0, 0))

# set legend
axes[2,1].legend(['accelerating', 'constant speed', 'braking'])

plt.show()
plt.savefig('force_limits.pdf')  







