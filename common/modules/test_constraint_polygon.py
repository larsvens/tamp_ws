#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 10, 10

# inputs
n=8
Fxf_max = 2000
Fyf_max = 1000


# define vertices 
t_vertices = np.linspace(-np.pi,np.pi,n+1) # define one overlapping vertex to facilitate line computation
Fx_vertices = Fxf_max*np.cos(t_vertices);
Fy_vertices = Fyf_max*np.sin(t_vertices);

print
print 't_vertices', t_vertices
print 'Fx_vertices', Fx_vertices
print 'Fy_vertices', Fy_vertices
print

plt.plot(Fx_vertices,Fy_vertices,'.k')
plt.show

# compute line
a = np.zeros(n)
b = np.zeros(n)
c = np.zeros(n)
for i in range(n):
    x1 = Fx_vertices[i]
    y1 = Fy_vertices[i]
    x2 = Fx_vertices[i+1]
    y2 = Fy_vertices[i+1]
    
    a[i] = y1-y2
    b[i] = x2-x1
    c[i] = (x1-x2)*y1+(y2-y1)*x2
    
    
    #  visualize using wolfram-alpha
    # print single inequalities 
    print '%.2f'%(a[i]), 'x + ', '%.2f'%(b[i]), 'y', '>=' , '%.2f'%(-c[i])
    
# print double inequalities
print 
for i in range(n/2):
    print '%.2f'%(-c[i]), '<=' , '%.2f'%(a[i]) , 'x + ', '%.2f'%(b[i]), 'y <= ', '%.2f'%(c[i]), ','