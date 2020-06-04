import numpy as np
import copy

# returns interval representation of angle ( [-pi pi] by default )
def angleToInterval(psi):
    try:         
        for i in range(psi.size):
            while(psi[i] > np.pi):
                psi[i] = psi[i] -2*np.pi
            while(psi[i] <= -np.pi):
                psi[i] = psi[i] +2*np.pi
        return psi
    except ValueError:
        print("Error in angleToInterval, psi.size = %i", psi.size)

# returns continous representation of discontinous angle (e.g. heading)
def angleToContinous(psi):
    psi_out = copy.deepcopy(psi)
    offset = 0
    for i in range(psi.size-1):
        psi_out[i] = psi[i] + offset
        if((psi[i+1]-psi[i]) > np.pi):  # detecting up-flip
            offset = offset - 2*np.pi
        if((psi[i+1]-psi[i]) < -np.pi): # detecting down-flip
            offset = offset + 2*np.pi
    psi_out[-1] = psi[-1] + offset
    return psi_out

def getVehicleCorners(X, Y, psi, lf, lr, w):
    
    R = np.array([[np.cos(psi), np.sin(psi)],
                  [-np.sin(psi), np.cos(psi)]])
    
    dims = np.array([[lf, w/2],
                     [-lr, w/2],
                     [-lr, -w/2],
                     [lf, -w/2]])
    
    
    corners = dims.dot(R);
    
    corners[:,0] = corners[:,0] + X
    corners[:,1] = corners[:,1] + Y
    
    return corners[:,0], corners[:,1]