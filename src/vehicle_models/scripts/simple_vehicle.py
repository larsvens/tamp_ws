#!/usr/bin/env python

# Node description:
# propagate vehicle model


import numpy as np
import rospy
from common.msg import State
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
        self.dt = 0.1
        self.rate = rospy.Rate(1.0/self.dt) 
        
        # set static params
        self.setStaticParams()
        
        # set init state
        self.setInitState()

        self.dp = DynamicVehicleParams()
        self.vehicle_in = VehicleIn()
        
        # wait for messages before entering main loop # tmp commented out!!
        #while(not self.vehicle_in.Fyf):
        #    print("waiting for control input")
        #    #self.statepub.publish(self.state)
        #    self.rate.sleep()
            
        # main loop
        while not rospy.is_shutdown():
            # get new vehicle state
            self.propagateVehicle() 
            self.vehicle_out = VehicleOut()
            self.vehicle_out.X = self.state.X
            self.vehicle_out.Y = self.state.Y
            self.vehicle_out.psi = self.state.psi
            self.vehicle_out.psidot = self.state.psidot
            self.vehicle_out.vx = self.state.vx
            self.vehicle_out.vy = self.state.vy
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
        self.state = State()
        self.state.X = rospy.get_param('/initstate_X')
        self.state.Y = rospy.get_param('/initstate_Y')
        self.state.psi = rospy.get_param('/initstate_psi')
        #self.state.s = rospy.get_param('/initstate_s')
        #self.state.d = rospy.get_param('/initstate_d')
        #self.state.deltapsi = rospy.get_param('/initstate_deltapsi')
        self.state.psidot = rospy.get_param('/initstate_psidot')
        self.state.vx = rospy.get_param('/initstate_vx')
        self.state.vy = rospy.get_param('/initstate_vy')
        #self.state.ax = rospy.get_param('/initstate_ax')
        #self.state.ay = rospy.get_param('/initstate_ay')
        #self.state.stop = rospy.get_param('/initstate_stop')

    def propagateVehicle(self):
        if (self.state.vx > 0.1):
            
            Fyf = self.vehicle_in.Fyf
            Fx  = self.vehicle_in.Fx
            print "Fyf = ", Fyf, " Fx = ", Fx, "vx = ", self.state.vx
            
            scalefactor = 1000
            dt = self.dt/scalefactor
             
            for i in range(scalefactor):
                
                X = self.state.X
                Y = self.state.Y
                psi = self.state.psi
                psidot = self.state.psidot
                vx = self.state.vx
                vy = self.state.vy
    
                alpha_r = (self.sp.lr*psidot-vy)/vx;
                Fyr = 2*alpha_r*self.sp.Cr;

                self.state.X = X + dt*(vx*np.cos(psi) + vy*np.sin(psi));
                self.state.Y = Y + dt*(vx*np.sin(psi) - vy*np.cos(psi));
                self.state.psi = psi + dt*psidot
                self.state.psidot = psidot + dt*(1/self.sp.Iz)*(self.sp.lf*Fyf - self.sp.lr*Fyr)
                self.state.vx = vx + dt*(1/self.sp.m)*(Fx - self.sp.m*self.sp.g*np.sin(self.dp.theta));
                self.state.vy = vy + dt*((1/self.sp.m)*(Fyf + Fyr + self.sp.m*self.sp.g*np.sin(self.dp.phi))-vx*psidot);
    
    
    def vehicle_in_callback(self, msg):
        #print("in static params callback")
        self.vehicle_in = msg
        
    def dynamicparams_callback(self, msg):
        #print("in static params callback")
        self.dp = msg
        

if __name__ == '__main__':
    vm = VehicleModel()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
