#!/usr/bin/env python

# Descrition: Publishes state, local path and dynamic params

import scipy.io as sio # for loading .mat file
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
        
        # init global path
        self.loadPathGlobalFromFile()
        
        # init local path
        self.pathlocal = PathLocal()
        self.loadPathLocalFromFile()

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
            #print("in main loop")
            self.state.header.stamp = rospy.Time.now()
            self.statepub.publish(self.state)
            self.pathlocal.header.stamp = rospy.Time.now()
            self.pathlocalpub.publish(self.pathlocal)        
            self.dynamic_params.header.stamp = rospy.Time.now()
            self.dynamic_param_pub.publish(self.dynamic_params)

            self.rate.sleep()          


    def propagateVehicle(state0):
        print("propagating vehicle")
        #return state1

    def loadPathGlobalFromFile(self):
        filename = '/home/larsvens/ros/saasqp_ws/src/perception/data/pathglobal_th.mat'
        mat = sio.loadmat(filename,struct_as_record=False, squeeze_me=True)
        
        self.pathglobal =	{
          "X":          mat['path_global'].X,
          "Y":          mat['path_global'].Y,
          "s":          mat['path_global'].s,
          "psi_c":      mat['path_global'].psi_c,
          "theta_c":    mat['path_global'].theta_c,
          "phi_c":      mat['path_global'].phi_c,
          "dub":        mat['path_global'].dub,
          "dlb":        mat['path_global'].dlb
        }
        
        
#        self.pathglobal.X =              mat['path_global'].X
#        self.pathglobal.Y =              mat['path_global'].Y
#        self.pathglobal.s =              mat['path_global'].s
#        self.pathglobal.psi_c =          mat['path_global'].psi_c
#        self.pathglobal.theta_c =        mat['path_global'].theta_c
#        self.pathglobal.phi_c =          mat['path_global'].phi_c
#        self.pathglobal.dub =            mat['path_global'].dub
#        self.pathglobal.dlb =            mat['path_global'].dlb               
        
    def loadPathLocalFromFile(self): # todo, replace
        filename = '/home/larsvens/ros/saasqp_ws/src/perception/data/path_local_example.mat'
        mat = sio.loadmat(filename,struct_as_record=False, squeeze_me=True)
        self.pathlocal.X =              mat['path_local'].X
        self.pathlocal.Y =              mat['path_local'].Y
        self.pathlocal.s =              mat['path_local'].s
        self.pathlocal.psi_c =          mat['path_local'].psi_c
        self.pathlocal.theta_c =        mat['path_local'].theta_c
        self.pathlocal.kappa_c =        mat['path_local'].kappa_c
        self.pathlocal.kappa_c_prime =  mat['path_local'].kappa_c_prime
        self.pathlocal.dub =            mat['path_local'].dub
        self.pathlocal.dlb =            mat['path_local'].dlb

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
    
    