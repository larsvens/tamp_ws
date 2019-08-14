import numpy as np

def ptsFrenetToCartesian(s,d,Xpath,Ypath,psipath,spath): 
    # inputs and outputs are np arrays
    
    Xc = np.interp(s,spath,Xpath) # if needed, investigate spline interp to improve precision
    Yc = np.interp(s,spath,Ypath)
    psic = np.interp(s,spath,psipath) 
    
    X = Xc - d*np.sin(psic);
    Y = Yc + d*np.cos(psic);
    return X,Y 

def ptsCartesianToFrenet(X,Y,Xpath,Ypath,psipath,spath):
    # inputs and outputs are np arrays
    Npath = spath.size
    Npts = X.size
    s = np.zeros(Npts);
    d = np.zeros(Npts);
    
    # transform pts one at a time
    for k in range(Npts):
    
        # find closest pt on centerline
        mindist = 1000000
        for i in range(Npath):
            dist = np.sqrt((X-Xpath[i])**2 + (Y-Ypath[i])**2 )
            if (dist<mindist):
                mindist = dist
                idx = i
                
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
        d[k] = M*np.cos(alpha2)
        s[k] = sc + deltas
        
    return s,d 