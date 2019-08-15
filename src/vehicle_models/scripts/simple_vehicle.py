#!/usr/bin/env python

# Node description:
# propagate vehicle model 
# runs at 100Hz


import numpy as np
import rospy
from common.msg import VehicleIn
from common.msg import VehicleOut
from common.msg import StaticVehicleParams
from common.msg import DynamicVehicleParams

class VehicleModel:
    def __init__(self):
        # init node subs pubs
        rospy.init_node('vehiclemodel', anonymous=True)
        self.dynamicparamsub = rospy.Subscriber("dynamic_vehicle_params", DynamicVehicleParams, self.dynamicparams_callback)
        self.vehicleinsub = rospy.Subscriber("vehicle_in", VehicleIn, self.vehicle_in_callback)
        self.vehicleoutpub = rospy.Publisher('vehicle_out', VehicleOut, queue_size=10)
        self.dt = 0.01
        self.rate = rospy.Rate(1.0/self.dt) 
        
        # set static params
        self.setStaticParams()
        
        # set init state
        self.setInitState()

        self.dp = DynamicVehicleParams()
        self.vehicle_in = VehicleIn()
                    
        # main loop
        while not rospy.is_shutdown():
            
            if(self.vehicle_in.Fyf):
                self.propagateVehicle()
                   
            self.vehicle_out = VehicleOut()
            self.vehicle_out.X = self.X
            self.vehicle_out.Y = self.Y
            self.vehicle_out.psi = self.psi
            self.vehicle_out.psidot = self.psidot
            self.vehicle_out.vx = self.vx
            self.vehicle_out.vy = self.vy
            self.vehicle_out.ax = self.ax
            self.vehicle_out.ay = self.ay
            self.vehicle_out.header.stamp = rospy.Time.now()
            self.vehicleoutpub.publish(self.vehicle_out)
            
            # end of main loop
            self.rate.sleep()
  
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
        
    def setInitState(self):
        self.X = rospy.get_param('/initstate_X')
        self.Y = rospy.get_param('/initstate_Y')
        self.psi = rospy.get_param('/initstate_psi')
        self.psidot = rospy.get_param('/initstate_psidot')
        self.vx = rospy.get_param('/initstate_vx')
        self.vy = rospy.get_param('/initstate_vy')
        self.ax = 0
        self.ay = 0

    def propagateVehicle(self):
        if (self.vx > 0.1):
            
            Fyf = self.vehicle_in.Fyf
            Fx  = self.vehicle_in.Fx
            print "Fyf = ", Fyf, " Fx = ", Fx, "vx = ", self.vx
            
            scalefactor = 100
            dt = self.dt/scalefactor
             
            for i in range(scalefactor):
                
                X = self.X
                Y = self.Y
                psi = self.psi
                psidot = self.psidot
                vx = self.vx
                vy = self.vy
    
                alpha_r = (self.sp.lr*psidot-vy)/vx;
                Fyr = 2*alpha_r*self.sp.Cr;

                self.X = X + dt*(vx*np.cos(psi) + vy*np.sin(psi));
                self.Y = Y + dt*(vx*np.sin(psi) - vy*np.cos(psi));
                self.psi = psi + dt*psidot
                self.psidot = psidot + dt*(1/self.sp.Iz)*(self.sp.lf*Fyf - self.sp.lr*Fyr)
                self.vx = vx + dt*(1/self.sp.m)*(Fx - self.sp.m*self.sp.g*np.sin(self.dp.theta));
                self.vy = vy + dt*((1/self.sp.m)*(Fyf + Fyr + self.sp.m*self.sp.g*np.sin(self.dp.phi))-vx*psidot);
                self.ax = (self.vx - vx)/self.dt
                self.ay = (self.vy - vy)/self.dt
    
    
    def vehicle_in_callback(self, msg):
        self.vehicle_in = msg
        
    def dynamicparams_callback(self, msg):
        self.dp = msg
        

if __name__ == '__main__':
    vm = VehicleModel()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
