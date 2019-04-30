#!/usr/bin/env python

# Descrition: Publishes state, local path and dynamic params

import numpy as np
import rospy
from common.msg import State
from common.msg import PathLocal
from common.msg import DynamicVehicleParams
from common.msg import StaticVehicleParams

class LocAndStateEst:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('loc_est', anonymous=True)
        self.sp = StaticVehicleParams()
        self.staticparamsub = rospy.Subscriber("static_vehicle_params", StaticVehicleParams, self.staticparams_callback)
        self.statepub = rospy.Publisher('state', State, queue_size=10)
        self.pathlocalpub = rospy.Publisher('pathlocal', PathLocal, queue_size=10)
        self.dynamic_param_pub = rospy.Publisher('dynamic_vehicle_params', DynamicVehicleParams, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        # init state
        self.state = State()
        self.state.X = 301.4733
        self.state.Y = -299.9720
        self.state.psi = 0.0920
        self.state.s = 515.2127
        self.state.d = -0.9293
        self.state.deltapsi = 0.0163
        self.state.psidot = 0.1656
        self.state.vx = 14.5600
        self.state.vy = 0.2352
        self.state.ax = -1.6650
        self.state.ay = 2.4374
        self.state.stop = False
        
        # init local and global path
        self.loadPathGlobalFromFile()
        self.pathlocal = PathLocal()

        # init dynamic params
        while(self.sp.m <= 0):
            print("waiting for static params")
            self.rate.sleep()
            
        self.dynamic_params = DynamicVehicleParams()
        self.dynamic_params.mu_alg = 0.7
        self.dynamic_params.mu_real = 0.7
        self.dynamic_params.Fz = self.sp.m * self.sp.g
        self.dynamic_params.Fzf = self.dynamic_params.Fz*self.sp.lr/(self.sp.lf+self.sp.lr); # moment balance at 0 acc
        self.dynamic_params.Fzr = self.dynamic_params.Fz*self.sp.lf/(self.sp.lf+self.sp.lr);
        self.dynamic_params.theta = 0.0
        self.dynamic_params.phi = 0.0
        
        # Main loop
        while not rospy.is_shutdown():
            # get new vehicle state
            self.propagateVehicle()
            self.state.header.stamp = rospy.Time.now()
            self.statepub.publish(self.state)

            # update dynamic vehicle params
            self.dynamic_params.header.stamp = rospy.Time.now()
            self.dynamic_param_pub.publish(self.dynamic_params)
            
            # update local path around new state 
            self.updateLocalPath()
            self.pathlocal.header.stamp = rospy.Time.now()
            self.pathlocalpub.publish(self.pathlocal)
            

            self.rate.sleep()          


    def propagateVehicle(self):
        print("todo: propagate vehicle")
        #return state1

    def loadPathGlobalFromFile(self):
        filename = '/home/larsvens/ros/saasqp_ws/src/perception/data/pathglobal.npy'
        pathglobal_npy = np.load(filename)
        self.pathglobal = pathglobal_npy.item()

    def updateLocalPath(self): # todo make sure s is continous when running several laps
        #print("updating local path")
        
        # define length of local path and params for interp
        smin = self.state.s - 1
        smax = smin+100
        N = 200
        s = np.linspace(smin,smax,N)
        
        # interpolate on global path
        self.pathlocal.X =              np.interp(s,self.pathglobal['s'],self.pathglobal['X'])
        self.pathlocal.Y =              np.interp(s,self.pathglobal['s'],self.pathglobal['Y'])
        self.pathlocal.s =              s
        self.pathlocal.psi_c =          np.interp(s,self.pathglobal['s'],self.pathglobal['psi_c'])
        self.pathlocal.theta_c =        np.interp(s,self.pathglobal['s'],self.pathglobal['theta_c'])
        self.pathlocal.kappa_c =        np.interp(s,self.pathglobal['s'],self.pathglobal['kappa_c'])
        self.pathlocal.kappaprime_c =  np.interp(s,self.pathglobal['s'],self.pathglobal['kappaprime_c'])
        self.pathlocal.dub =            np.interp(s,self.pathglobal['s'],self.pathglobal['dub'])
        self.pathlocal.dlb =            np.interp(s,self.pathglobal['s'],self.pathglobal['dlb'])
        

    #callbacks
    def staticparams_callback(self, msg):
        #print("in static params callback")
        self.sp = msg

if __name__ == '__main__':
    lse = LocAndStateEst()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    