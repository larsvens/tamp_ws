#!/usr/bin/env python

# Descrition: Publishes local path along with estimated friction along it
# inputs:
# state from fssim (topic /fssim/base_pose_ground_truth)
# global path from track interface (topic pathglobal)
# outputs: 
# /pathlocal
# + rviz visualizations

import numpy as np
import rospy
from common.msg import State
from common.msg import Path

from util import angleToInterval
from util import angleToContinous
from coordinate_transforms import ptsFrenetToCartesian
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as navPath
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
import time



class Perception:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('perception', anonymous=True)
        self.pathglobalsub = rospy.Subscriber("pathglobal", Path, self.pathglobal_callback)
        self.state_sub = rospy.Subscriber("state", State, self.state_callback)
        self.pathlocalpub = rospy.Publisher('pathlocal', Path, queue_size=10)
        #self.pathlocalvispub_old = rospy.Publisher('pathlocal_vis', navPath, queue_size=10)
        self.pathlocalvispub = rospy.Publisher('pathlocal_vis', PolygonArray, queue_size=1)

        # node params
        self.dt = 0.1
        self.rate = rospy.Rate(1/self.dt) # 10hz
        
        # params of local path
        self.N = 300
        self.ds = 0.5

        # set static vehicle params
        self.setRosParams()

        # init local vars
        self.pathglobal = Path()
        self.pathrolling = Path() # used to run several laps
        self.pathlocal = Path()
        self.state = State()
               
        # msg receive checks
        self.received_pathglobal = False
        self.received_state = False
        
        # wait for messages before entering main loop
        while(not self.received_pathglobal):
            print "perception: waiting for pathglobal"
            self.rate.sleep()
        
        while(not self.received_state):
            print "perception: waiting for state"
            self.rate.sleep()

        print "perception: running main with: "
        print "perception: lap length: ", self.s_lap
        print "perception: length of local path: ", self.N*self.ds
        
        # Main loop
        while not rospy.is_shutdown():
            
            # check timing wrt dt
            start = time.time()
 
            # update pathrolling to handle multiple laps
            if (self.state.s > self.pathrolling.s[0]+self.s_lap + 25): # if we're on the second lap of pathrolling
                self.pathrolling.s = self.pathrolling.s + self.s_lap
           
            # update local path 
            self.updateLocalPath()
            self.pathlocalpub.publish(self.pathlocal)
            
            # visualize local path in rviz
            pathlocal_vis = navPath()
            pathlocal_vis.header.stamp = rospy.Time.now()
            pathlocal_vis.header.frame_id = "map"
            for i in range(self.N):
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = self.pathlocal.X[i]
                pose.pose.position.y = self.pathlocal.Y[i]       
                pathlocal_vis.poses.append(pose)
            #self.pathlocalvispub_old.publish(pathlocal_vis)


            pa = self.pathLocalToPolArr()
            self.pathlocalvispub.publish(pa)

            end = time.time()
            comptime = end-start
            #print("perception: compute took ", comptime)
            if (comptime > self.dt):
                rospy.logwarn("perception: compute time exceeding dt!")

            self.rate.sleep()   
            

    def updateLocalPath(self):
       
        stot_local = self.N*self.ds
        self.smin_local = max(self.state.s - 1,0)
        #print "smin_local = ", self.smin_local
        smax_local = self.smin_local+stot_local
        
        s = np.linspace(self.smin_local,smax_local,self.N)
        
        # interpolate on global path
        
        if (s[0] < self.pathrolling.s[0] or s[-1] > self.pathrolling.s[-1]):
            rospy.logerr("perception: pathlocal.s not on pathrolling interval!")
            print "state.s       ", self.state.s
            print "pathlocal.s   ", s
            print "pathrolling.s ", self.pathrolling.s
        
        self.pathlocal.header.stamp = rospy.Time.now()
        self.pathlocal.header.frame_id = "map"
        self.pathlocal.X =              np.interp(s,self.pathrolling.s,self.pathrolling.X)
        self.pathlocal.Y =              np.interp(s,self.pathrolling.s,self.pathrolling.Y)
        self.pathlocal.s =              s
        self.pathlocal.psi_c =          np.interp(s,self.pathrolling.s,angleToContinous(self.pathrolling.psi_c)) # interpolating on continous psic
        self.pathlocal.psi_c =          angleToInterval(self.pathlocal.psi_c)
        self.pathlocal.theta_c =        np.interp(s,self.pathrolling.s,self.pathrolling.theta_c)
        self.pathlocal.kappa_c =        np.interp(s,self.pathrolling.s,self.pathrolling.kappa_c)
        self.pathlocal.kappaprime_c =   np.interp(s,self.pathrolling.s,self.pathrolling.kappaprime_c)
        self.pathlocal.mu =             np.interp(s,self.pathrolling.s,self.pathrolling.mu)
        self.pathlocal.dub =            np.interp(s,self.pathrolling.s,self.pathrolling.dub)
        self.pathlocal.dlb =            np.interp(s,self.pathrolling.s,self.pathrolling.dlb)
        
    def pathLocalToPolArr(self):
        pa = PolygonArray()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = "map"
        for i in range(self.N-1):

            spoly = np.array([self.pathlocal.s[i], self.pathlocal.s[i+1],self.pathlocal.s[i+1],self.pathlocal.s[i]])
            dpoly = np.array([self.pathlocal.dub[i], self.pathlocal.dub[i+1],self.pathlocal.dlb[i+1],self.pathlocal.dlb[i]])
            Xpoly, Ypoly = ptsFrenetToCartesian(spoly,dpoly,self.pathlocal.X,self.pathlocal.Y,self.pathlocal.psi_c,self.pathlocal.s)

            p = PolygonStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "map"
            p.polygon.points = [Point32(x=Xpoly[0], y=Ypoly[0], z=0.0),
                                Point32(x=Xpoly[1], y=Ypoly[1], z=0.0),
                                Point32(x=Xpoly[2], y=Ypoly[2], z=0.0),
                                Point32(x=Xpoly[3], y=Ypoly[3], z=0.0)]


            pa.polygons.append(p)
            #pa.Color.r = 1.0
            #p.color.g = 0.0
            #p.color.b = 0.0        
        
        # color for mu
        pa.likelihood = self.pathlocal.mu[0:-1]*(0.2/np.max(self.pathlocal.mu)) # discarding final value
               
        #pa.color.r = 1.0
        #pa.color.g = 0.0
        #pa.color.b = 0.0
        
        return pa
        
    def setRosParams(self):
        self.m = rospy.get_param('/car/inertia/m')
        self.g = rospy.get_param('/car/inertia/g')
        self.lf = rospy.get_param('/car/kinematics/b_F')
        self.lr = rospy.get_param('/car/kinematics/b_R')
              
    def pathglobal_callback(self, msg):
        self.pathglobal = msg
        
        # get s of one lap
        stot_global = self.pathglobal.s[-1]
        dist_sf = np.sqrt( (self.pathglobal.X[0]-self.pathglobal.X[-1])**2 + (self.pathglobal.Y[0]-self.pathglobal.Y[-1])**2)
        self.s_lap = stot_global + dist_sf  
        
        # start pathrolling as two first laps
        self.pathrolling.X = np.concatenate((np.array(self.pathglobal.X),np.array(self.pathglobal.X)),axis=0)
        self.pathrolling.Y = np.concatenate((np.array(self.pathglobal.Y),np.array(self.pathglobal.Y)),axis=0)
        self.pathrolling.s = np.concatenate((np.array(self.pathglobal.s),np.array(self.pathglobal.s) + self.s_lap ),axis=0)
        self.pathrolling.psi_c = np.concatenate((np.array(self.pathglobal.psi_c),np.array(self.pathglobal.psi_c)),axis=0)
        self.pathrolling.theta_c = np.concatenate((np.array(self.pathglobal.theta_c),np.array(self.pathglobal.theta_c)),axis=0)
        self.pathrolling.kappa_c = np.concatenate((np.array(self.pathglobal.kappa_c),np.array(self.pathglobal.kappa_c)),axis=0)
        self.pathrolling.kappaprime_c = np.concatenate((np.array(self.pathglobal.kappaprime_c),np.array(self.pathglobal.kappaprime_c)),axis=0)
        self.pathrolling.mu = np.concatenate((np.array(self.pathglobal.mu),np.array(self.pathglobal.mu)),axis=0)
        self.pathrolling.dub = np.concatenate((np.array(self.pathglobal.dub),np.array(self.pathglobal.dub)),axis=0)
        self.pathrolling.dlb = np.concatenate((np.array(self.pathglobal.dlb),np.array(self.pathglobal.dlb)),axis=0)
        
        self.received_pathglobal = True
    
    def state_callback(self, msg):
        self.state = msg
        self.received_state = True

if __name__ == '__main__':
    lse = Perception()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
