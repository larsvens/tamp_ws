#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from common.msg import Trajectory
from common.msg import State

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100

class LivePlotter:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('liveplotter', anonymous=True)
        self.trajhatsub = rospy.Subscriber("trajhat", Trajectory, self.trajhat_callback)
        self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
        self.statesub = rospy.Subscriber("state", State, self.state_callback)
        self.rate = rospy.Rate(10) # 10hz
        # init internal variables
        self.counter = 0 # use this to reduce plot update rate
        self.trajhat_latest = Trajectory()
        self.trajstar_latest = Trajectory()
        self.state_latest = State()
        # init plot window
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.axis("equal")
         
        # redraw plot repeatedly
        while not rospy.is_shutdown():
            ax.plot(self.trajhat_latest.s, self.trajhat_latest.d, '.b')           
            ax.plot(self.trajstar_latest.s, self.trajstar_latest.d, '*m')       
            ax.plot(self.state_latest.s, self.state_latest.d,'o r')
            
            plt.draw() 
            plt.pause(0.001)
            self.rate.sleep()

    def trajhat_callback(self, msg):
        #print("in trajhat callback")
        self.trajhat_latest = msg
        
    def trajstar_callback(self, msg):
        print("in trajstar callback")
        self.trajstar_latest = msg
    
    def state_callback(self, msg):
        #print("in state callback")
        self.state_latest = msg


if __name__ == '__main__':
    lp = LivePlotter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
    
    



    
    