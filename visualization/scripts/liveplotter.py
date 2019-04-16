#!/usr/bin/env python
import time
import numpy as np
#import matplotlib.patches as patches
import matplotlib.pyplot as plt
import rospy
from common.msg import PathLocal
from common.msg import Obstacles
from common.msg import Trajectory
from common.msg import State
from common.msg import StaticVehicleParams
from common.msg import DynamicVehicleParams

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 15, 5


def getCirclePts(x,y,r,n):
    #n = nr of vertices
    t_vertices = np.arange(-np.pi, np.pi, (2*np.pi)/n)
    xpts = r*np.cos(t_vertices) + x;
    ypts = r*np.sin(t_vertices) + y;
    return xpts, ypts

def ptsFrenetToCartesian(Xc,Yc,psic,d):
    # inputs and outputs are np arrays
    X = Xc - d*np.sin(psic);
    Y = Yc + d*np.cos(psic);
    return X,Y  
    

class LivePlotter:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('liveplotter', anonymous=True)
        self.pathlocalsub = rospy.Subscriber("pathlocal", PathLocal, self.pathlocal_callback)
        self.obstaclesub = rospy.Subscriber("obstacles", Obstacles, self.obstacles_callback)
        self.trajhatsub = rospy.Subscriber("trajhat", Trajectory, self.trajhat_callback)
        self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
        self.statesub = rospy.Subscriber("state", State, self.state_callback)
        self.staticparamsub = rospy.Subscriber("static_vehicle_params", StaticVehicleParams, self.staticparams_callback)
        self.dynamicparamsub = rospy.Subscriber("dynamic_vehicle_params", DynamicVehicleParams, self.dynamicparams_callback)
        self.dt = 1.0
        self.rate = rospy.Rate(1.0/self.dt) 
        # init internal variables
        self.counter = 0 # use this to reduce plot update rate
        self.pathlocal = PathLocal()
        self.obstacles = Obstacles()
        self.trajhat = Trajectory()
        self.trajstar = Trajectory()
        self.state = State()
        self.sp = StaticVehicleParams()
        self.dp = DynamicVehicleParams()
        
        # init plot window
        plt.ion()    
        f, (a0, a1) = plt.subplots(1,2, gridspec_kw = {'width_ratios':[3, 1]})
        a0.axis("equal")
        a1.axis("equal")
         
        # Main loop
        while not rospy.is_shutdown():
            tm = time.time()

            
            N = len(self.trajstar.s)
            
            # plot lane lines
            nplot = int(0.4*len(self.pathlocal.s))                   
            llX, llY = ptsFrenetToCartesian(np.array(self.pathlocal.X[0:nplot]), \
                                            np.array(self.pathlocal.Y[0:nplot]), \
                                            np.array(self.pathlocal.psi_c[0:nplot]), \
                                            np.array(self.pathlocal.dub[0:nplot]) )
            
            rlX, rlY = ptsFrenetToCartesian(np.array(self.pathlocal.X[0:nplot]), \
                                            np.array(self.pathlocal.Y[0:nplot]), \
                                            np.array(self.pathlocal.psi_c[0:nplot]), \
                                            np.array(self.pathlocal.dlb[0:nplot]) )         
            a0.plot(llX,llY,'k') 
            a0.plot(rlX,rlY,'k') 
            
            # plot obstacles
            Nobs = len(self.obstacles.s)
            for i in range(Nobs):
                spt = self.obstacles.s[i]
                dpt = self.obstacles.d[i]
                # transform to cartesian
                Xcpt = np.interp(spt,self.pathlocal.s,self.pathlocal.X)
                Ycpt = np.interp(spt,self.pathlocal.s,self.pathlocal.Y)
                psicpt = np.interp(spt,self.pathlocal.s,self.pathlocal.psi_c)                 
                Xobs,Yobs = ptsFrenetToCartesian(Xcpt,Ycpt,psicpt,dpt)
                # define obstacle circle
                Robs = self.obstacles.R[i]
                Rmgnobs = self.obstacles.Rmgn[i]
                Xpts, Ypts = getCirclePts(Xobs,Yobs,Rmgnobs,20)
                a0.plot(Xpts, Ypts, 'r')
                Xpts, Ypts = getCirclePts(Xobs,Yobs,Robs,20)
                a0.plot(Xpts, Ypts, 'r')
                
            # plot trajhat
            a0.plot(self.trajhat.X, self.trajhat.Y, '.b')           
            
            # plot state constraint
            for k in range(N):
                slb = self.trajhat.slb[k]
                sub = self.trajhat.sub[k]
                dlb = self.trajhat.dlb[k]
                dub = self.trajhat.dub[k]
                n_intp = 3
                args = (np.linspace(sub,sub,n_intp), np.linspace(sub,slb,n_intp), np.linspace(slb,slb,n_intp), np.linspace(slb,sub,n_intp))
                spts = np.concatenate(args)
                args = (np.linspace(dub,dlb,n_intp), np.linspace(dlb,dlb,n_intp), np.linspace(dlb,dub,n_intp), np.linspace(dub,dub,n_intp))
                dpts = np.concatenate(args)
                Xcpts = np.interp(spts,self.pathlocal.s,self.pathlocal.X)
                Ycpts = np.interp(spts,self.pathlocal.s,self.pathlocal.Y)
                psicpts = np.interp(spts,self.pathlocal.s,self.pathlocal.psi_c)               
                Xpts, Ypts = ptsFrenetToCartesian(Xcpts, Ycpts, psicpts,dpts)
                a0.plot(Xpts,Ypts,'m')
                
            # plot trajstar
            a0.plot(self.trajstar.X, self.trajstar.Y, '*m')       
            
            # plot ego vehicle pose
            length_front = self.sp.lf+0.75;
            length_rear = self.sp.lr+0.75;
            width = self.sp.w;
            R = np.matrix([[np.cos(self.state.psi),   np.sin(self.state.psi)], \
                           [-np.sin(self.state.psi),  np.cos(self.state.psi)]])

            corners = np.matrix([[length_front,  width/2], \
                                 [-length_rear,  width/2], \
                                 [-length_rear, -width/2], \
                                 [length_front, -width/2], \
                                 [length_front,  width/2]])
            corners = corners*R;
            corners[:,0] = corners[:,0] + self.state.X;
            corners[:,1] = corners[:,1] + self.state.Y;
            
            a0.plot(corners[:,0],corners[:,1], 'k')           
            
            # plot real friction circle
            
            # plot algo friction circle
            Fxfpts, Fyfpts = getCirclePts(0,0,self.dp.mu_alg*self.dp.Fzf,100)
            a1.plot(Fxfpts,Fyfpts,'k')
            
            # plot trajhat forces
            a1.plot(self.trajhat.Fxf, self.trajstar.Fyf, 'xr')
            
            # plot trajstar forces
            a1.plot(self.trajstar.Fxf, self.trajstar.Fyf, 'xb')
                      
            # redraw plot
            plt.draw() 
            plt.pause(0.001)
            
            # print plot time 
            elapsed = time.time() - tm
            print("dt = " + str(self.dt) + ", plot time = " + str(elapsed))
            if(self.dt < elapsed):
                print("WARNING: plot time is larger than dt")
            
            self.rate.sleep()

    def pathlocal_callback(self, msg):
        #print("in pathlocal callback")
        self.pathlocal = msg
    
    def obstacles_callback(self, msg):
        #print("in obstacles callback")
        self.obstacles = msg
        
    def trajhat_callback(self, msg):
        #print("in trajhat callback")
        self.trajhat = msg
        
    def trajstar_callback(self, msg):
        #print("in trajstar callback")
        self.trajstar = msg
    
    def state_callback(self, msg):
        #print("in state callback")
        self.state = msg
        
    def staticparams_callback(self, msg):
        #print("in static params callback")
        self.sp = msg
        
    def dynamicparams_callback(self, msg):
        #print("in static params callback")
        self.dp = msg
    


    
if __name__ == '__main__':
    lp = LivePlotter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
    
    



    
    