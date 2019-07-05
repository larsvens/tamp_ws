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
        self.pathlocalpub = rospy.Publisher('pathlocal', PathLocal, queue_size=10)
        self.dynamic_param_pub = rospy.Publisher('dynamic_vehicle_params', DynamicVehicleParams, queue_size=10)
        self.state_sub = rospy.Subscriber("state", State, self.state_callback)
        self.dt = 0.1
        self.rate = rospy.Rate(1/self.dt) # 10hz

        # set static vehicle params
        self.setStaticParams()

        # init local vars
        self.loadPathGlobalFromFile()
        self.pathlocal = PathLocal()
        
        # init state
        self.setInitState()
        
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

            # update dynamic vehicle params
            self.dynamic_params.header.stamp = rospy.Time.now()
            self.dynamic_param_pub.publish(self.dynamic_params)
            
            # update local path around new state 
            self.updateLocalPath()
            self.pathlocal.header.stamp = rospy.Time.now()
            self.pathlocalpub.publish(self.pathlocal)

            self.rate.sleep()          


    def setInitState(self): # tmp
        self.state = State()
        self.state.X = rospy.get_param('/initstate_X')
        self.state.Y = rospy.get_param('/initstate_Y')
        self.state.psi = rospy.get_param('/initstate_psi')
        self.state.s = rospy.get_param('/initstate_s')
        self.state.d = rospy.get_param('/initstate_d')
        self.state.deltapsi = rospy.get_param('/initstate_deltapsi')
        self.state.psidot = rospy.get_param('/initstate_psidot')
        self.state.vx = rospy.get_param('/initstate_vx')
        self.state.vy = rospy.get_param('/initstate_vy')
        self.state.ax = rospy.get_param('/initstate_ax')
        self.state.ay = rospy.get_param('/initstate_ay')
        self.state.stop = rospy.get_param('/initstate_stop')
            
            
    def loadPathGlobalFromFile(self):
        pathglobal_filepath = rospy.get_param('/pathglobal_filepath')
        pathglobal_npy = np.load(pathglobal_filepath)
        self.pathglobal = pathglobal_npy.item()

    def updateLocalPath(self): # todo make sure s is continous when running several laps
        #print("updating local path")
        
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
        
    def state_callback(self, msg):
        #print("in static params callback")
        self.state = msg

if __name__ == '__main__':
    lse = LocAndStateEst()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    