#!/usr/bin/env python

# Descrition: Adds frenet coordinates to state

# subscribes:
# cartesian state from state_est_cart
# global path from track interface (topic /pathglobal)

# publishes: 
# state (topic /state) - full state containing both frenet and cartesian coordinates


import numpy as np
import rospy
from common.msg import Path
from common.msg import State
from coordinate_transforms import ptsCartesianToFrenet
from util import angleToInterval
from util import angleToContinous
import time

class StateEstFrenet:
    # constructor
    def __init__(self):
        # init node
        rospy.init_node('state_est_frenet', anonymous=True)
        self.dt = 0.02
        self.rate = rospy.Rate(1/self.dt) # 50hz   
        
        # load rosparams
        self.robot_name = rospy.get_param('/robot_name')
        
        # init subs
        self.pathglobalsub = rospy.Subscriber("pathglobal", Path, self.pathglobal_callback)
        self.pathglobal = Path()
        self.received_pathglobal = False
        self.state_cart_sub = rospy.Subscriber("/state_cart", State, self.state_cart_callback)
        self.state_cart = State()
        self.received_state_cart = False
        
        # init pubs
        self.statepub = rospy.Publisher('state', State, queue_size=10)
        self.state_out = State()        
           
        # init local vars
        self.passed_halfway = False
        self.lapcounter = 0
    
        # wait for messages before entering main loop
        while(not self.received_pathglobal):
            rospy.loginfo_throttle(1, "state_est_frenet: waiting for pathglobal")
            self.rate.sleep()
        
        # compute length of track
        stot_global = self.pathglobal.s[-1]
        dist_sf = np.sqrt( (self.pathglobal.X[0]-self.pathglobal.X[-1])**2 + (self.pathglobal.Y[0]-self.pathglobal.Y[-1])**2)
        self.s_lap = stot_global + dist_sf  
        
        # wait for cartesian state
        while(not self.received_state_cart):
            rospy.loginfo_throttle(1, "state_est_frenet: waiting for state_cart")
            self.rate.sleep()            
            
        rospy.logwarn("state_est_frenet: started")

        # Main loop
        while not rospy.is_shutdown():
            
            # timing
            start = time.time()
            
            # state est
            start_statest = time.time()
            self.update_frenet_state()
            self.statepub.publish(self.state_out)
            end_statest = time.time()
            comptime_statest = end_statest-start_statest
            
            # timing: check wrt dt
            end = time.time()
            comptime = end-start
            if (comptime > self.dt):
                rospy.logwarn("state_est_frenet: compute time exceeding dt!")
                rospy.logwarn("state_est_frenet: total comptime =        " + str(comptime))
                rospy.logwarn("state_est_frenet: comptime statest =      " + str(comptime_statest))
            
            self.rate.sleep()   
            
    def update_frenet_state(self):
      
        self.state_out.X = self.state_cart.X
        self.state_out.Y = self.state_cart.Y
        self.state_out.psi = self.state_cart.psi
        self.state_out.psidot = self.state_cart.psidot
        self.state_out.vx = self.state_cart.vx
        self.state_out.vy = self.state_cart.vy

        # get s, d and deltapsi
        s,d = ptsCartesianToFrenet(np.array([self.state_out.X]), \
                                   np.array([self.state_out.Y]), \
                                   np.array(self.pathglobal.X), \
                                   np.array(self.pathglobal.Y), \
                                   np.array(self.pathglobal.psi_c), \
                                   np.array(self.pathglobal.s))
        s_this_lap = s[0]
        
        # checks for lap counter
        if (s_this_lap > 0.45*self.s_lap and s_this_lap < 0.55*self.s_lap and not self.passed_halfway):
            self.passed_halfway = True
            print "state est: passed halfway mark"
        if (s_this_lap > 0.0 and s_this_lap < 10.0 and self.passed_halfway):
            self.lapcounter = self.lapcounter + 1
            self.passed_halfway = False
            print "state est: completed lap, lap count = ", self.lapcounter
        
        # make sure s is >= 0 on first lap
        if(self.lapcounter == 0 and s_this_lap > 0.75*self.s_lap and not self.passed_halfway):
            s_this_lap = 0.0
        
        self.state_out.s = s_this_lap + self.lapcounter*self.s_lap                     
        self.state_out.d = d[0]
        
        psi_c = np.interp(s,self.pathglobal.s,self.pathglobal_psic_cont)
        angleToInterval(psi_c)
        
        self.state_out.deltapsi = self.state_out.psi - psi_c
        # correction of detapsi @ psi flips
        self.state_out.deltapsi = angleToInterval(self.state_out.deltapsi)
        self.state_out.deltapsi = self.state_out.deltapsi[0]
        self.state_out.header.stamp = rospy.Time.now()

        
    def state_cart_callback(self, msg):
        self.state_cart = msg
        self.received_state_cart = True
    
    def pathglobal_callback(self, msg):
        self.pathglobal = msg      
        self.pathglobal_psic_cont = angleToContinous(np.array(self.pathglobal.psi_c))
        self.received_pathglobal = True

if __name__ == '__main__':
    sef = StateEstFrenet()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    