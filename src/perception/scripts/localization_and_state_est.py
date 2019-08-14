#!/usr/bin/env python

# Descrition: Publishes state, local path and dynamic params

import numpy as np
import rospy
from common.msg import State
from common.msg import VehicleOut
from common.msg import PathLocal
from common.msg import DynamicVehicleParams
from common.msg import StaticVehicleParams

from coordinate_transforms import ptsCartesianToFrenet

class LocAndStateEst:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('loc_est', anonymous=True)
        self.pathlocalpub = rospy.Publisher('pathlocal', PathLocal, queue_size=10)
        self.dynamic_param_pub = rospy.Publisher('dynamic_vehicle_params', DynamicVehicleParams, queue_size=10)
        self.statepub = rospy.Publisher('state', State, queue_size=10)
        self.vehicle_out_sub = rospy.Subscriber("vehicle_out", VehicleOut, self.vehicle_out_callback)

        self.dt = 0.1
        self.rate = rospy.Rate(1/self.dt) # 10hz

        # set static vehicle params
        self.setStaticParams()

        # init local vars
        self.loadPathGlobalFromFile()
        self.pathlocal = PathLocal()
        self.state = State()
        self.vehicle_out = VehicleOut()
        
        # init dynamic params    
        self.dynamic_params = DynamicVehicleParams()
        self.dynamic_params.mu_alg = 0.7
        self.dynamic_params.mu_real = 0.7
        self.dynamic_params.Fz = self.sp.m * self.sp.g
        self.dynamic_params.Fzf = self.dynamic_params.Fz*self.sp.lr/(self.sp.lf+self.sp.lr); # moment balance at 0 acc
        self.dynamic_params.Fzr = self.dynamic_params.Fz*self.sp.lf/(self.sp.lf+self.sp.lr);
        self.dynamic_params.theta = 0.0
        self.dynamic_params.phi = 0.0
        
        # wait for messages before entering main loop
        while(not self.vehicle_out.X):
            print("waiting for vehicle_out")
            self.rate.sleep()

        # Main loop
        while not rospy.is_shutdown():

            # update dynamic params
            self.dynamic_params.header.stamp = rospy.Time.now()
            self.dynamic_param_pub.publish(self.dynamic_params)
            
            # update local path around new state 
            self.updateLocalPath()
            self.pathlocal.header.stamp = rospy.Time.now()
            self.pathlocalpub.publish(self.pathlocal)

            # compute vehicle state from vehicle_out info
            self.updateState()
            self.statepub.publish(self.state)


            self.rate.sleep()          


    def updateState(self):
        self.state.X = self.vehicle_out.X
        self.state.Y = self.vehicle_out.Y
        self.state.psi = self.vehicle_out.psi
        self.state.psidot = self.vehicle_out.psidot
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
        self.state.ax = self.vehicle_out.ax
        self.state.ay = self.vehicle_out.ay



    def loadPathGlobalFromFile(self):
        pathglobal_filepath = rospy.get_param('/pathglobal_filepath')
        pathglobal_npy = np.load(pathglobal_filepath,allow_pickle=True)
        self.pathglobal = pathglobal_npy.item()


    def updateLocalPath(self):
        # todo make sure s is continous when running several laps
        
        # define length of local path and params for interp
        smin = self.state.s - 1
        smax = smin+300
        N = 200
        s = np.linspace(smin,smax,N)
        
        # interpolate on global path
        self.pathlocal.X =              np.interp(s,self.pathglobal['s'],self.pathglobal['X'])
        self.pathlocal.Y =              np.interp(s,self.pathglobal['s'],self.pathglobal['Y'])
        self.pathlocal.s =              s
        self.pathlocal.psi_c =          np.interp(s,self.pathglobal['s'],self.pathglobal['psi_c'])
        self.pathlocal.theta_c =        np.interp(s,self.pathglobal['s'],self.pathglobal['theta_c'])
        self.pathlocal.kappa_c =        np.interp(s,self.pathglobal['s'],self.pathglobal['kappa_c'])
        self.pathlocal.kappaprime_c =   np.interp(s,self.pathglobal['s'],self.pathglobal['kappaprime_c'])
        self.pathlocal.dub =            np.interp(s,self.pathglobal['s'],self.pathglobal['dub'])
        self.pathlocal.dlb =            np.interp(s,self.pathglobal['s'],self.pathglobal['dlb'])
        

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
        
    def vehicle_out_callback(self, msg):
        #print("in static params callback")
        self.vehicle_out = msg

if __name__ == '__main__':
    lse = LocAndStateEst()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
