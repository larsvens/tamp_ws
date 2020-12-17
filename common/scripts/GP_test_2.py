#!/usr/bin/env python2


import numpy as np
import matplotlib.pylab as plt


def kernel(x, y, l2=0.1, sigma_f=1.0):
    sqdist = np.sum(x**2,1).reshape(-1,1) + \
             np.sum(y**2,1) - 2*np.dot(x, y.T)
    return sigma_f**2 * np.exp(-.5 * (1/l2) * sqdist)

def generate_noisy_points(n=10, noise_variance=1e-6):
    np.random.seed(777)
    X = np.random.uniform(-3., 3., (n, 1))
    y = 0.5 + 0.4*np.sin(X) + np.random.randn(n, 1) * noise_variance**0.5
    return X, y

def generate_noisy_points_2(n=10, noise_variance=1e-6):
    np.random.seed(777)
    X = np.random.uniform(-3., 5., (n, 1))
    y = 0.5 + 0.4*np.sin(X) + np.random.randn(n, 1) * noise_variance**0.5 
    return X, y

def posterior(Xtrain, ytrain, Xtest, l2=0.1, noise_var_train=1e-6,sigma_f=1.0):
    ytrain_zeromean = ytrain-np.mean(ytrain)
    
    # compute the mean at our test points.
    Ntrain = len(Xtrain)
    K = kernel(Xtrain, Xtrain, l2,sigma_f)
    L = np.linalg.cholesky(K + noise_var_train*np.eye(Ntrain))
    Lk = np.linalg.solve(L, kernel(Xtrain, Xtest, l2,sigma_f))
    mean = np.dot(Lk.T, np.linalg.solve(L, ytrain_zeromean)) + np.mean(ytrain)
    # compute the variance at our test points.
    K_ = kernel(Xtest, Xtest, l2,sigma_f)
    sd = np.sqrt(np.diag(K_) - np.sum(Lk**2, axis=0))
    return (mean, sd, K)


Xtrain, ytrain = generate_noisy_points()
#Xtest, ytest = generate_noisy_points_2(100)
#Xtest.sort(axis=0)
#Xtest = np.linspace(-3,5,100)
Ntest = 100
Xtest = np.reshape(np.linspace(-3,5,Ntest), (Ntest, 1))
mean, sd, K = posterior(Xtrain,ytrain, Xtest,3.5,0.001,0.25)
print(mean, sd)


plt.plot(Xtrain, ytrain, 'xb')
plt.plot(Xtest, mean.flatten()-1.96*sd,'k--' )
plt.plot(Xtest, mean.flatten()+1.96*sd,'k--' )


#plt.plot(Xtest, ytest, 'xr')
plt.show()

#plt.figure()
#plt.plot(x, f(x), 'r:', label=r'$f(x) = x\,\sin(x)$')
#plt.plot(x, y_pred, 'b-', label='Prediction')
#plt.fill(np.concatenate([x, x[::-1]]),
#         np.concatenate([y_pred - 1.9600 * sigma,
#                        (y_pred + 1.9600 * sigma)[::-1]]),
#         alpha=.5, fc='b', ec='None', label='95% confidence interval')
    