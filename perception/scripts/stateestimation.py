#!/usr/bin/env python

import numpy as np
import rospy
from common.msg import Path
from fssim_common.msg import State as fssimState
from common.msg import State as saartiState
from coordinate_transforms import ptsCartesianToFrenet
from util import angleToInterval

class StateEst:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('state_est', anonymous=True)
        self.pathglobalsub = rospy.Subscriber("pathglobal", Path, self.pathglobal_callback)
        self.vehicle_out_sub = rospy.Subscriber("/fssim/base_pose_ground_truth", fssimState, self.vehicle_out_callback)
        self.statepub = rospy.Publisher('state', saartiState, queue_size=10)
        
        # init local vars
        self.pathglobal = Path()
        self.state_out = saartiState()
        self.state_in = fssimState()

        # node params
        self.dt = 0.01
        self.rate = rospy.Rate(1/self.dt) # 100hz
        self.received_vehicle_out = False
        self.received_pathglobal = False
    
        # wait for messages before entering main loop
        while(not self.received_pathglobal):
            print "state est: waiting for pathglobal"
            self.rate.sleep()
        
        while(not self.received_vehicle_out):
            print "state est: waiting for vehicle_out"
            self.rate.sleep()
    
        print "state est: running main "

        # Main loop
        while not rospy.is_shutdown():
            self.updateState()
            self.statepub.publish(self.state_out)
            self.rate.sleep()   
            
    def updateState(self):
        self.state_out.X = self.state_in.x
        self.state_out.Y = self.state_in.y
        self.state_out.psi = self.state_in.yaw
        self.state_out.psidot = self.state_in.r
        self.state_out.vx = self.state_in.vx
        self.state_out.vy = self.state_in.vy

        # get s, d and deltapsi
        s,d = ptsCartesianToFrenet(np.array(self.state_out.X), \
                                   np.array(self.state_out.Y), \
                                   np.array(self.pathglobal.X), \
                                   np.array(self.pathglobal.Y), \
                                   np.array(self.pathglobal.psi_c), \
                                   np.array(self.pathglobal.s))
        self.state_out.s = s[0]
        self.state_out.d = d[0]
        psi_c = np.interp(s,self.pathglobal.s,self.pathglobal.psi_c)
        self.state_out.deltapsi = self.state_out.psi - psi_c
        # correction of detapsi @ psi flips
        self.state_out.deltapsi = angleToInterval(self.state_out.deltapsi)
        
    def vehicle_out_callback(self, msg):
        self.state_in = msg
        self.received_vehicle_out = True
        
    def pathglobal_callback(self, msg):
        self.pathglobal = msg      
        self.received_pathglobal = True

if __name__ == '__main__':
    lse = StateEst()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    