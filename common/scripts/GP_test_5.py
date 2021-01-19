#!/usr/bin/env python2





import numpy as np
import pylab as pb
import GPy 


X = np.random.uniform(-10,20, 50)
X = X[~np.logical_and(X>-2,X<3)] #Remove points between -2 and 3 (just for illustration) 
X = np.hstack([np.random.uniform(-1,1,1),X]) #Prepend a point between -1 and 1  (just for illustration)
error = np.random.normal(0,.2,X.size)
Y = f(X) + error

#fig,ax = pb.subplots()
#ax.plot(np.linspace(-15,25),f(np.linspace(-10,20)),'r-')
#ax.plot(X,Y,'kx',mew=1.5)
#ax.grid()


kern = GPy.kern.MLP(1) + GPy.kern.Bias(1)


m = GPy.models.GPHeteroscedasticRegression(X[:,None],Y[:,None],kern)
m['.*het_Gauss.variance'] = abs(error)[:,None] #Set the noise parameters to the error in Y
m.het_Gauss.variance.fix() #We can fix the noise term, since we already know it
m.optimize()

m.plot_f() #Show the predictive values of the GP.
#pb.errorbar(X,Y,yerr=np.array(m.likelihood.flattened_parameters).flatten(),fmt=None,ecolor='r',zorder=1)
pb.grid()
pb.plot(X,Y,'kx',mew=1.5)
pb.show()