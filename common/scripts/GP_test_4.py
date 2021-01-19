#!/usr/bin/env python2

# GP with sequential update (based on krasser)

import numpy as np
import matplotlib.pylab as plt


def plot_gp(mu, cov, X, X_train=None, Y_train=None):
    X = X.ravel()
    mu = mu.ravel()
    #uncertainty = 1.65 * np.sqrt(np.diag(cov)) # 90%
    uncertainty = 1.96 * np.sqrt(np.diag(cov)) # 95%
    
    
    plt.fill_between(X, mu + uncertainty, mu - uncertainty, alpha=0.1)
    plt.plot(X, mu, label='Mean')
    #plt.plot(X, sample, lw=1, ls='--', label=f'Sample {i+1}')
    if X_train is not None:
        plt.plot(X_train, Y_train, 'rx')
    plt.legend()
    plt.show()

def kernel(X1, X2, l=1.0, sigma_f=1.0):
    """
    Isotropic squared exponential kernel.
    
    Args:
        X1: Array of m points (m x d).
        X2: Array of n points (n x d).

    Returns:
        (m x n) matrix.
    """
    sqdist = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * np.dot(X1, X2.T)
    return sigma_f**2 * np.exp(-0.5 / l**2 * sqdist)


def posterior(X_s, X_train, Y_train, l=1.0, sigma_f=1.0, sigma_y=1e-8):
    """
    Computes the suffifient statistics of the posterior distribution 
    from m training data X_train and Y_train and n new inputs X_s.
    
    Args:
        X_s: New input locations (n x d).
        X_train: Training locations (m x d).
        Y_train: Training targets (m x 1).
        l: Kernel length parameter.
        sigma_f: Kernel vertical variation parameter.
        sigma_y: Noise parameter.
    
    Returns:
        Posterior mean vector (n x d) and covariance matrix (n x n).
    """
    K = kernel(X_train, X_train, l, sigma_f) + sigma_y**2 * np.eye(len(X_train))
    K_s = kernel(X_train, X_s, l, sigma_f)
    K_ss = kernel(X_s, X_s, l, sigma_f) + sigma_y * np.eye(len(X_s))
    K_inv = np.linalg.inv(K)
    
    # Equation (7)
    mu_s = K_s.T.dot(K_inv).dot(Y_train)

    # Equation (8)
    cov_s = K_ss - K_s.T.dot(K_inv).dot(K_s)
    
    return mu_s, cov_s

def posterior_(X_s, X_train, Y_train, noise_vector, l=1.0, sigma_f=1.0, sigma_y=1e-8):
    """
    Computes the suffifient statistics of the posterior distribution 
    from m training data X_train and Y_train and n new inputs X_s.
    
    Args:
        X_s: New input locations (n x d).
        X_train: Training locations (m x d).
        Y_train: Training targets (m x 1).
        l: Kernel length parameter.
        sigma_f: Kernel vertical variation parameter.
        sigma_y: Noise parameter.
    
    Returns:
        Posterior mean vector (n x d) and covariance matrix (n x n).
    """
    K = kernel(X_train, X_train, l, sigma_f) + np.diag(noise_vector**2)
    K_s = kernel(X_train, X_s, l, sigma_f)
    K_ss = kernel(X_s, X_s, l, sigma_f) + sigma_y * np.eye(len(X_s))
    K_inv = np.linalg.inv(K)
    
    # Equation (7)
    mu_s = K_s.T.dot(K_inv).dot(Y_train)

    # Equation (8)
    cov_s = K_ss - K_s.T.dot(K_inv).dot(K_s)
    
    return mu_s, cov_s


# Finite number of points
X = np.arange(0.0, 40.0, 1).reshape(-1, 1)


# Noisy training data
noise = 0.2
X_train = np.arange(0.0, 40, 4).reshape(-1, 1)
X_train = np.linspace(0.0, np.max(X), 10).reshape(-1, 1)
mu_est_at_s = 0.6
Y_train = np.concatenate((np.array([mu_est_at_s]),0.8*np.ones(5),0.4*np.ones(4)))


noise_vector = 0.025*np.ones_like(X_train)
noise_vector[0] =0.01
# compute posterior dist
posterior_mean, posterior_cov = posterior_(X, 
                                           X_train, 
                                           Y_train,
                                           noise_vector,
                                           l=6, 
                                           sigma_f=1.0, 
                                           sigma_y=0.025
                                           )

plot_gp(posterior_mean, posterior_cov, X, X_train=X_train, Y_train=Y_train)








