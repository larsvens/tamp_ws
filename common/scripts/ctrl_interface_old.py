#!/usr/bin/env python

# Node description:
# translate trajstar into specific control inputs for the vehicle
# runs at 100 Hz

import numpy as np
import rospy
from common.msg import Trajectory
from common.msg import VehicleIn
from common.msg import VehicleOut
from common.msg import StaticVehicleParams

class CtrlInterface:
    def __init__(self):
        # init node subs pubs
        rospy.init_node('ctrl_interface', anonymous=True)
        self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
        self.vehicle_out_sub = rospy.Subscriber("vehicle_out", VehicleOut, self.vehicle_out_callback)
        self.vehicleinpub = rospy.Publisher('vehicle_in', VehicleIn, queue_size=10)
        self.rate = rospy.Rate(100)

        # set static vehicle params
        self.setStaticParams()

        # init msgs
        self.vehicle_in = VehicleIn()
        self.vehicle_out = VehicleOut()
        self.trajstar = Trajectory()

        # wait for messages before entering main loop # tmp commented out!!
        while(not self.trajstar.Fyf):
            print("waiting for trajstar")
            self.rate.sleep()

        # main loop
        while not rospy.is_shutdown():
            
            # compute delta corresponding to Fyf request (linear tire, Rajamani) 
            Fyf_request = self.trajstar.Fyf[0]
            alpha_f = Fyf_request/(2*self.sp.Cr)
            psidot = self.vehicle_out.psidot
            vx = self.vehicle_out.vx
            vy = self.vehicle_out.vy

            if(self.vehicle_out.vx > 0.1):
                delta = alpha_f + np.arctan2(vy+self.sp.lf*psidot,vx)
            else:
                delta = 0
            
            self.vehicle_in.delta = delta
            #self.vehicle_in.Fyf = self.trajstar.Fyf[0]
            self.vehicle_in.Fx = self.trajstar.Fx[0]
            self.vehicle_in.header.stamp = rospy.Time.now()
            self.vehicleinpub.publish(self.vehicle_in)

    def trajstar_callback(self, msg):
        self.trajstar = msg
    
    def vehicle_out_callback(self, msg):
        self.vehicle_out = msg

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

if __name__ == '__main__':
    vm = CtrlInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
