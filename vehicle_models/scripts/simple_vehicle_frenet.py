#!/usr/bin/env python
import numpy as np
import rospy
from common.msg import State
#from common.msg import CtrlCmd
from common.msg import Trajectory
from common.msg import PathLocal
from common.msg import StaticVehicleParams
from common.msg import DynamicVehicleParams

class VehicleModel:
    def __init__(self):
        # init node subs pubs
        rospy.init_node('vehiclemodel', anonymous=True)
        self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
        self.dynamicparamsub = rospy.Subscriber("dynamic_vehicle_params", DynamicVehicleParams, self.dynamicparams_callback)
        self.pathlocalsub = rospy.Subscriber("pathlocal", PathLocal, self.pathlocal_callback)
        self.statepub = rospy.Publisher('state', State, queue_size=10)
        self.dt = 0.1
        self.rate = rospy.Rate(1.0/self.dt) 
        #self.rate = rospy.Rate(2)
        
        # set static params
        self.setStaticParams()
        
        # set init state
        self.setInitState()

        #self.ctrl_cmd = CtrlCmd()
        
        self.dp = DynamicVehicleParams()
        self.pathlocal = PathLocal()
        self.trajstar = Trajectory()
        
        # wait for messages before entering main loop
        while(not self.pathlocal.s):
            print("waiting for pathlocal")
            self.statepub.publish(self.state)
            self.rate.sleep()
        while(not self.sp.Iz): 
            print("waiting for static params")
            self.statepub.publish(self.state)
            self.rate.sleep()
        while(not self.trajstar.Fyf): 
            print("waiting for trajstar")
            self.statepub.publish(self.state)
            self.rate.sleep()
            
        # main loop
        while not rospy.is_shutdown():
            # get new vehicle state
            self.propagateVehicle() 
            self.state.header.stamp = rospy.Time.now()
            self.statepub.publish(self.state)
            
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
        self.state.s = rospy.get_param('/initstate_s')
        self.state.d = rospy.get_param('/initstate_d')
        self.state.deltapsi = rospy.get_param('/initstate_deltapsi')
        self.state.psidot = rospy.get_param('/initstate_psidot')
        self.state.vx = rospy.get_param('/initstate_vx')
        self.state.vy = rospy.get_param('/initstate_vy')
        self.state.ax = rospy.get_param('/initstate_ax')
        self.state.ay = rospy.get_param('/initstate_ay')
        self.state.stop = rospy.get_param('/initstate_stop')

    def propagateVehicle(self):
        if (self.state.vx > 0.1):
            
            Fyf = self.trajstar.Fyf[0]
            Fx  = self.trajstar.Fx[0]
            
            print "Fyf = ", Fyf, " Fx = ", Fx, "vx = ", self.state.vx
            
            
            scalefactor = 1000
            dt = self.dt/scalefactor
             
            for i in range(scalefactor):
                
                s = self.state.s
                d = self.state.d
                deltapsi = self.state.deltapsi
                psidot = self.state.psidot
                vx = self.state.vx
                vy = self.state.vy
    
                alpha_r = (self.sp.lr*psidot-vy)/vx;
                Fyr = 2*alpha_r*self.sp.Cr;
                #print(s)
                #print(self.pathlocal.s)
                kappa_c = np.interp(s,self.pathlocal.s,self.pathlocal.kappa_c)

                self.state.s = s + dt*(vx*np.cos(deltapsi) - vy*np.sin(deltapsi))/(1-d*kappa_c);
                self.state.d = d + dt*(vx*np.sin(deltapsi) + vy*np.cos(deltapsi));
                self.state.deltapsi = deltapsi + dt*(psidot-kappa_c*(vx*np.cos(deltapsi)-vy*np.sin(deltapsi))/(1-d*kappa_c))
                self.state.psidot = psidot + dt*(1/self.sp.Iz)*(self.sp.lf*Fyf - self.sp.lr*Fyr)
                self.state.vx = vx + dt*(1/self.sp.m)*(Fx - self.sp.m*self.sp.g*np.sin(self.dp.theta));
                self.state.vy = vy + dt*((1/self.sp.m)*(Fyf + Fyr + self.sp.m*self.sp.g*np.sin(self.dp.phi))-vx*psidot);


            # compute X Y psi
            Xc = np.interp(s,self.pathlocal.s,self.pathlocal.X)
            Yc = np.interp(s,self.pathlocal.s,self.pathlocal.Y)
            psi_c = np.interp(s,self.pathlocal.s,self.pathlocal.psi_c)
            self.state.X = Xc - self.state.d*np.sin(psi_c);
            self.state.Y = Yc + self.state.d*np.cos(psi_c);
            self.state.psi = self.state.deltapsi + psi_c;
    
    
    def trajstar_callback(self, msg):
        #print("in static params callback")
        self.trajstar = msg
        
    def dynamicparams_callback(self, msg):
        #print("in static params callback")
        self.dp = msg
        
    def pathlocal_callback(self, msg):
        #print("in pathlocal callback")
        self.pathlocal = msg

if __name__ == '__main__':
    vm = VehicleModel()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
