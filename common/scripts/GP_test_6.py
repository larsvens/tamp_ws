#!/usr/bin/env python2

import numpy as np
import pylab as pb
import GPy 


import time

pb.close('all')

# Noisy training data

X_train = np.linspace(0.0, 50, 30)
X_train = np.concatenate((np.array([-3]),X_train)) # include local mu est history 
mu_est_at_s = 0.9
Y_train = np.concatenate((np.array([mu_est_at_s,mu_est_at_s]),0.8*np.ones(15),0.4*np.ones(14)))
abs_error = 0.025*np.ones_like(X_train)
abs_error[0] = 0.0001
abs_error[1] = 0.0001

#kern = GPy.kern.MLP(1) + GPy.kern.Bias(1)
kern = GPy.kern.RBF(input_dim=1, variance=1., lengthscale=15.)

t0= time.clock()

m = GPy.models.GPHeteroscedasticRegression(X_train[:,None],Y_train[:,None],kern)
m['.*het_Gauss.variance'] = abs_error[:,None] #Set the noise parameters to the error in Y

#m.het_Gauss.variance.fix() #We can fix the noise term, since we already know it
#m.optimize()




mu, var = m._raw_predict(m.X)

t1 = time.clock() - t0
print("Time elapsed: ", t1) # CPU seconds elapsed (floating point)


m.plot_f() #Show the predictive values of the GP.
#pb.errorbar(X,Y,yerr=np.array(m.likelihood.flattened_parameters).flatten(),fmt=None,ecolor='r',zorder=1)
pb.grid()
pb.plot(X_train,Y_train,'kx',mew=1.5)


# check 
pb.plot(m.X,mu,'r')
pb.plot(m.X,mu+1.96*np.sqrt(var),'r')
pb.plot(m.X,mu-1.96*np.sqrt(var),'r')

pb.show()

