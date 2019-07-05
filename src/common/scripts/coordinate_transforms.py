import numpy as np

def ptsFrenetToCartesian(Xc,Yc,psic,d): 
    # inputs and outputs are np arrays
    X = Xc - d*np.sin(psic);
    Y = Yc + d*np.cos(psic);
    return X,Y 

#def ptsCartesianToFrenet(X,Y,path)
