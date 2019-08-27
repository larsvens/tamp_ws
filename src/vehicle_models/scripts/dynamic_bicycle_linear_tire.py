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
            
            #if(self.vehicle_in.Fyf):
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
            
            # get control input            
            delta = self.vehicle_in.delta
            Fx  = self.vehicle_in.Fx            
            
            # scaling to mitigate discretization error
            scalefactor = 100
            dt = self.dt/float(scalefactor)   

            X0 = self.X
            Y0 = self.Y
            psi0 = self.psi
            psidot0 = self.psidot
            vx0 = self.vx
            vy0 = self.vy     
            for i in range(scalefactor):
                # lateral dynamics (Rajamani)
                alpha_f = delta - np.arctan2(vy0+self.sp.lf*psidot0,vx0)
                Fyf = 2*alpha_f*self.sp.Cf
                
                print "Fyf = ", Fyf, " Fx = ", Fx, "vx = ", self.vx
                
                # longitudinal dynamics
                alpha_r = np.arctan2(self.sp.lr*psidot0-vy0,vx0)
                Fyr = 2*alpha_r*self.sp.Cr

                X1      = X0 + dt*(vx0*np.cos(psi0) + vy0*np.sin(psi0))
                Y1      = Y0 + dt*(vx0*np.sin(psi0) - vy0*np.cos(psi0))
                psi1    = psi0 + dt*psidot0
                psidot1 = psidot0 + dt*(1.0/self.sp.Iz)*(self.sp.lf*Fyf - self.sp.lr*Fyr)
                vx1     = vx0 + dt*(1.0/self.sp.m)*(Fx - self.sp.m*self.sp.g*np.sin(self.dp.theta))
                vy1     = vy0 + dt*((1.0/self.sp.m)*(Fyf + Fyr + self.sp.m*self.sp.g*np.sin(self.dp.phi))-vx0*psidot0)

                #print "vx1-vx0 = ", vx1-vx0
                #print " vx0 = ", vx0, " dt = ", dt, " m = ", self.sp.m, " Fx = ", Fx ," g = ", self.sp.g, " phi = ", self.dp.phi, " psidot0 = ", psidot0
                
                X0 = X1
                Y0 = Y1
                psi0 = psi1
                psidot0 = psidot1
                vx0 = vx1
                vy0 = vy1 
      
            self.ax = (self.vx - vx1)/self.dt
            self.ay = (self.vy - vy1)/self.dt
            
            self.X = X1
            self.Y = Y1
            self.psi = psi1
            self.psidot = psidot1
            self.vx = vx1
            self.vy = vy1
    
    
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
    
