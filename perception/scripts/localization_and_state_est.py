#!/usr/bin/env python

# Descrition: Publishes state, local path and dynamic params
# inputs:
# state from fssim (topic /fssim/base_pose_ground_truth)
# global path from track interface (topic pathglobal)
# outputs: 
# /state (cartesian + frenet)
# /pathlocal
# + rviz visualizations

import numpy as np
import rospy
from common.msg import State
#from common.msg import VehicleOut
from common.msg import Path
from common.msg import DynamicVehicleParams
from common.msg import StaticVehicleParams
from coordinate_transforms import ptsCartesianToFrenet
from fssim_common.msg import State as fssimState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as navPath

class LocAndStateEst:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('loc_est', anonymous=True)
        self.pathglobalsub = rospy.Subscriber("pathglobal", Path, self.pathglobal_callback)
        self.vehicle_out_sub = rospy.Subscriber("/fssim/base_pose_ground_truth", fssimState, self.vehicle_out_callback)
        self.dynamic_param_pub = rospy.Publisher('dynamic_vehicle_params', DynamicVehicleParams, queue_size=10)
        self.statepub = rospy.Publisher('state', State, queue_size=10)
        self.pathlocalpub = rospy.Publisher('pathlocal', Path, queue_size=10)
        self.pathlocalvispub = rospy.Publisher('pathlocal_vis', navPath, queue_size=10)

        # node params
        self.dt = 0.1
        self.rate = rospy.Rate(1/self.dt) # 10hz
        
        # params of local path
        self.N = 200
        self.ds = 0.5

        # set static vehicle params
        self.setStaticParams()

        # init local vars
        self.pathglobal = Path()
        self.pathlocal = Path()
        self.state = State()
        self.vehicle_out = fssimState()
        
        # init dynamic params    
        self.dynamic_params = DynamicVehicleParams()
        self.dynamic_params.mu_alg = 0.7
        self.dynamic_params.mu_real = 0.7
        self.dynamic_params.Fz = self.sp.m * self.sp.g
        self.dynamic_params.Fzf = self.dynamic_params.Fz*self.sp.lr/(self.sp.lf+self.sp.lr); # moment balance at 0 acc
        self.dynamic_params.Fzr = self.dynamic_params.Fz*self.sp.lf/(self.sp.lf+self.sp.lr);
        self.dynamic_params.theta = 0.0
        self.dynamic_params.phi = 0.0
        
        # msg receive checks
        self.received_pathglobal = False
        self.received_vehicle_out = False
        
        # wait for messages before entering main loop
        while(not self.received_pathglobal):
            print "locandstateest: waiting for pathglobal"
            self.rate.sleep()
        
        while(not self.received_vehicle_out):
            print "locandstateest: waiting for vehicle_out"
            self.rate.sleep()

        print "locandstateest: running main"
        # Main loop
        while not rospy.is_shutdown():

            # update dynamic params
            self.dynamic_params.header.stamp = rospy.Time.now()
            self.dynamic_param_pub.publish(self.dynamic_params)
            
            # update local path around new state 
            self.updateLocalPath()
            self.pathlocal.header.stamp = rospy.Time.now()
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
            self.pathlocalvispub.publish(pathlocal_vis)

            # compute vehicle state from vehicle_out info
            self.updateState()
            self.statepub.publish(self.state)

            self.rate.sleep()          


    def updateState(self):
        self.state.X = self.vehicle_out.x
        self.state.Y = self.vehicle_out.y
        self.state.psi = self.vehicle_out.yaw
        self.state.psidot = self.vehicle_out.r
        self.state.vx = self.vehicle_out.vx
        self.state.vy = self.vehicle_out.vy

        # get s, d and deltapsi
        s,d = ptsCartesianToFrenet(np.array(self.state.X), \
                                   np.array(self.state.Y), \
                                   np.array(self.pathlocal.X), \
                                   np.array(self.pathlocal.Y), \
                                   np.array(self.pathlocal.psi_c), \
                                   np.array(self.pathlocal.s))
        self.state.s = s[0]
        self.state.d = d[0]
        psi_c = np.interp(s,self.pathlocal.s,self.pathlocal.psi_c)
        self.state.deltapsi = self.state.psi - psi_c


    def updateLocalPath(self):
        # todo make sure s is continous when running several laps
        
        # define length of local path and params for interp
        #N = 200
        #ds = 0.5
        stot = self.N*self.ds
        smin = self.state.s - 1
        smax = smin+stot
        
        s = np.linspace(smin,smax,self.N)
        
        # interpolate on global path
        self.pathlocal.X =              np.interp(s,self.pathglobal.s,self.pathglobal.X)
        self.pathlocal.Y =              np.interp(s,self.pathglobal.s,self.pathglobal.Y)
        self.pathlocal.s =              s
        self.pathlocal.psi_c =          np.interp(s,self.pathglobal.s,self.pathglobal.psi_c)
        self.pathlocal.theta_c =        np.interp(s,self.pathglobal.s,self.pathglobal.theta_c)
        self.pathlocal.kappa_c =        np.interp(s,self.pathglobal.s,self.pathglobal.kappa_c)
        self.pathlocal.kappaprime_c =   np.interp(s,self.pathglobal.s,self.pathglobal.kappaprime_c)
        self.pathlocal.dub =            np.interp(s,self.pathglobal.s,self.pathglobal.dub)
        self.pathlocal.dlb =            np.interp(s,self.pathglobal.s,self.pathglobal.dlb)
        

    def setStaticParams(self):
        self.sp = StaticVehicleParams()
        self.sp.g = rospy.get_param('/g')
        self.sp.l = rospy.get_param('/l')
        self.sp.w = rospy.get_param('/w')
        self.sp.m = rospy.get_param('/m')
        self.sp.Iz = rospy.get_param('/Iz')
        self.sp.lf = rospy.get_param('/lf')
        self.sp.lr = rospy.get_param('/lr')
        self.sp.Cf = rospy.get_param('/Cf')
        self.sp.Cr = rospy.get_param('/Cr')
        self.sp.Da = rospy.get_param('/Da')
        self.sp.deltamin = rospy.get_param('/deltamin')
        self.sp.deltamax = rospy.get_param('/deltamax')
        
    def pathglobal_callback(self, msg):
        self.pathglobal = msg
        self.received_pathglobal = True
    
    def vehicle_out_callback(self, msg):
        self.vehicle_out = msg
        self.received_vehicle_out = True

if __name__ == '__main__':
    lse = LocAndStateEst()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
