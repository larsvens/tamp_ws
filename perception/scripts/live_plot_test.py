#import pylab
#import time
import numpy as np
#import random

import matplotlib
matplotlib.use('WX')
import matplotlib.pyplot as plt


dat = np.random.rand(100)
fig = plt.figure()
ax = fig.add_subplot(111)
Ln, = ax.plot(dat)
ax.set_xlim([0,100])
plt.ion()
plt.show()    
for i in range (1000):
    dat = np.random.rand(100)
    Ln.set_ydata(dat)
    Ln.set_xdata(range(len(dat)))
    plt.pause(0.1)

    print 'done with loop'
