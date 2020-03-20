import numpy as np
from util import angleToContinous
from util import angleToInterval

def ptsFrenetToCartesian(s,d,Xpath,Ypath,psipath,spath): 
    # inputs and outputs are np.array([x])
    
    Xc = np.interp(s,spath,Xpath) # if needed, investigate spline interp to improve precision
    Yc = np.interp(s,spath,Ypath)
    psipath_cont = angleToContinous(psipath)
    psic_cont = np.interp(s,spath,psipath_cont)
    psic = angleToInterval(psic_cont)
    
    X = Xc - d*np.sin(psic);
    Y = Yc + d*np.cos(psic);
    return X,Y 

def ptsCartesianToFrenet(X,Y,Xpath,Ypath,psipath,spath):
    # inputs and outputs are np.array([x])

    Npts = X.size
    s = np.zeros(Npts);
    d = np.zeros(Npts);
    
    # transform pts one at a time
    for k in range(Npts):
    
        # find closest pt on centerline
        dist = np.sqrt((X[k]-Xpath)**2 + (Y[k]-Ypath)**2 )
        idx = np.argmin(dist)

        # make sure deltas is positive
        deltas = -1
        maxiters = 5
        iters = 0
        while(deltas < 0 and iters <= maxiters):
        
            Xc = Xpath[idx] 
            Yc = Ypath[idx] 
            psic = psipath[idx] 
            sc = spath[idx] 
            
            # compute angles etc
            deltaX = X - Xc
            deltaY = Y - Yc
            alpha1 = -np.arctan2(deltaX,deltaY)
            alpha2 = psic - alpha1
            M = np.sqrt(deltaX**2 + deltaY**2)
            deltas = M*np.sin(alpha2)
            
            # iteratively reduce idx until deltas positive
            idx = idx - 1
            iters = iters + 1
            
        d[k] = M*np.cos(alpha2)
        s[k] = sc + deltas
        
    return s,d 
