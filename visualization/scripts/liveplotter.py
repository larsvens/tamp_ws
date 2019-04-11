#!/usr/bin/env python
import numpy as np
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import rospy
from common.msg import PathLocal
from common.msg import Trajectory
from common.msg import State
from common.msg import StaticVehicleParams

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100

class LivePlotter:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('liveplotter', anonymous=True)
        self.pathlocalsub = rospy.Subscriber("pathlocal", PathLocal, self.pathlocal_callback)
        self.trajhatsub = rospy.Subscriber("trajhat", Trajectory, self.trajhat_callback)
        self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
        self.statesub = rospy.Subscriber("state", State, self.state_callback)
        self.staticparamsub = rospy.Subscriber("static_vehicle_params", StaticVehicleParams, self.staticparams_callback)
        self.rate = rospy.Rate(10) # 10hz
        # init internal variables
        self.counter = 0 # use this to reduce plot update rate
        self.pathlocal = PathLocal()
        self.trajhat = Trajectory()
        self.trajstar = Trajectory()
        self.state = State()
        self.sp = StaticVehicleParams()
        
        # init plot window
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.axis("equal")
         
        # Main loop
        while not rospy.is_shutdown():
            # plot lane lines
            nplot = int(0.4*len(self.pathlocal.s))            
            llX = []
            llY = []
            rlX = []
            rlY = []
            for i in range(0,nplot):
                llX.append(self.pathlocal.X[i] - self.pathlocal.dub[i]*np.sin(self.pathlocal.psi_c[i]))
                llY.append(self.pathlocal.Y[i] + self.pathlocal.dub[i]*np.cos(self.pathlocal.psi_c[i]))
                rlX.append(self.pathlocal.X[i] - self.pathlocal.dlb[i]*np.sin(self.pathlocal.psi_c[i]))
                rlY.append(self.pathlocal.Y[i] + self.pathlocal.dlb[i]*np.cos(self.pathlocal.psi_c[i]))            
            ax.plot(llX,llY,'k') 
            ax.plot(rlX,rlY,'k') 
            
            # plot trajhat
            ax.plot(self.trajhat.X, self.trajhat.Y, '.b')           
            
            # plot trajstar
            ax.plot(self.trajstar.X, self.trajstar.Y, '*m')       
            
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
            
            ax.plot(corners[:,0],corners[:,1], 'k')           
            
            
            # redraw plot
            plt.draw() 
            plt.pause(0.001)
            self.rate.sleep()

    def pathlocal_callback(self, msg):
        #print("in pathlocal callback")
        self.pathlocal = msg
        
    def trajhat_callback(self, msg):
        #print("in trajhat callback")
        self.trajhat = msg
        
    def trajstar_callback(self, msg):
        print("in trajstar callback")
        self.trajstar = msg
    
    def state_callback(self, msg):
        #print("in state callback")
        self.state = msg
    def staticparams_callback(self, msg):
        #print("in static params callback")
        self.sp = msg
    
if __name__ == '__main__':
    lp = LivePlotter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
    
    



    
    