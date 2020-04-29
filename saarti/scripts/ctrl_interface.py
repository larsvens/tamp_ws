#!/usr/bin/env python

# Descrition: Translates trajstar into vehicle specific control inputs 

# subscribes:
# state from stateestimation node (topic /state)
# local path from perception (topic /pathlocal)
# trajstar from saarti node (topic /trajstar)
# ctrl_mode from experiment manager (topic /ctrl_mode)

# publishes: 
# vehicle specific ctrl command (topic /fssim/cmd for sim, topic **** for real opendlv)

# swithches controller and output topic based on the "system_setup" param
# system_setup = rhino_real -> /OpenDLV/ActuationRequest
# system_setup = rhino_fssim or gotthard_fssim -> /fssim/cmd

import numpy as np
from scipy.interpolate import interp1d
import rospy
from common.msg import Trajectory
from common.msg import Path
from common.msg import State
from fssim_common.msg import Cmd as FssimCmd
from opendlv_ros.msg import ActuationRequest 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from coordinate_transforms import ptsFrenetToCartesian
from std_msgs.msg import Int16

class CtrlInterface:
    def __init__(self):
        
        # params
        self.system_setup = rospy.get_param('/system_setup')
        self.dt = rospy.get_param('/dt_ctrl')
        
        # init node subs pubs
        rospy.init_node('ctrl_interface', anonymous=True)
        self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
        self.pathlocalsub = rospy.Subscriber("pathlocal", Path, self.pathlocal_callback)
        self.state_sub = rospy.Subscriber("/state", State, self.state_callback)
        self.ctrlmodesub = rospy.Subscriber("ctrl_mode", Int16, self.ctrl_mode_callback)
        self.vxrefsub = rospy.Subscriber("vxref", Float32, self.vxref_callback)
        self.lhptpub = rospy.Publisher('/lhpt_vis', Marker, queue_size=1)
        self.polyfitpub = rospy.Publisher('/polyfit_vis', Marker, queue_size=1)
        self.vx_errorpub = rospy.Publisher('/vx_error_vis', Float32, queue_size=1)
        self.rate = rospy.Rate(1/self.dt)
        if(self.system_setup == "rhino_real"):
            self.cmdpub = rospy.Publisher('/OpenDLV/ActuationRequest', ActuationRequest, queue_size=1)
            self.cmd = ActuationRequest()
        elif(self.system_setup == "rhino_fssim" or self.system_setup == "gotthard_fssim"):
            self.cmdpub = rospy.Publisher('/fssim/cmd', FssimCmd, queue_size=1)
            self.cmd = FssimCmd()
        
        # set static vehicle params
        self.setStaticParams()

        # init msgs
        self.state = State()
        self.trajstar = Trajectory()
        self.pathlocal = Path()
        self.ctrl_mode = 0 # 0: stop, 1: cruise_ctrl, 2: tamp 
        self.trajstar_received = False
        self.pathlocal_received = False
        self.state_received = False
        self.ctrl_mode_received = False
        self.vx_error = Float32()
        
        # get dref setpoint
        self.cc_dref = rospy.get_param('/cc_dref')
        
        # postproc tuning vars (TUNE LAT)
        self.delta_out_buffer_size = 20
        self.delta_out_buffer = np.zeros(self.delta_out_buffer_size)
        self.delta_out_ma_window_size = 20               # 1 deactivates        
        self.db_range = 0.5*(np.pi/180)                 # 0.0 deactivates
        self.delta_out_rate_max = 30.0*(np.pi/180)       # large value deactivates
        self.db_range_acc = 0.05                        # 0.0 deactivates
        
        # wait for messages before entering main loop
        while(not self.state_received):
            rospy.loginfo_throttle(1, "waiting for state")
            self.rate.sleep()

        while(not self.pathlocal_received):
            rospy.loginfo_throttle(1, "waiting for pathlocal")
            self.rate.sleep()

        while(not self.ctrl_mode_received):
            rospy.loginfo_throttle(1, "waiting for ctrl_mode")
            self.rate.sleep
            
        # main loop
        while not rospy.is_shutdown(): 
            # COMPUTE CONTROL
            if(self.ctrl_mode == 0):     # STOP (set by exp manager if outside of track)
                rospy.loginfo_throttle(1,"in stop mode")
                delta_out = self.delta_out_buffer[0]
                dc_out = 0.0
            elif(self.ctrl_mode == 1):   # CRUISE CTRL             
                delta_out, dc_out = self.cc_ctrl()                                      
            elif(self.ctrl_mode == 2):   # TAMP   
                while(not self.trajstar_received):
                    rospy.loginfo_throttle(1, "ctrl_interface: waiting for trajstar")
                    delta_out = 0
                    dc_out = 0
                    self.rate.sleep()         
                delta_out, dc_out = self.tamp_ctrl()
            else:
                rospy.logerr("ctrl_interface: invalid ctrl_mode! ctrl_mode = " + str(self.ctrl_mode))
    
            # POST PROCESSING
            self.delta_out_buffer = np.roll(self.delta_out_buffer,1)
            if(not np.isnan(delta_out)):
                self.delta_out_buffer[0] = delta_out
            else:
                delta_out = self.delta_out_buffer[1]
                self.delta_out_buffer[0] = delta_out
                rospy.logerr("ctrl_interface: nans in delta computation")
            # smoothing
            delta_out = np.mean(self.delta_out_buffer[0:self.delta_out_ma_window_size])
            # rate limit
            if(self.delta_out_buffer[0] > self.delta_out_buffer[1] + self.delta_out_rate_max*self.dt):
                delta_out = self.delta_out_buffer[1] + self.delta_out_rate_max*self.dt
                self.delta_out_buffer[0] = delta_out
            if(self.delta_out_buffer[0] < self.delta_out_buffer[1] - self.delta_out_rate_max*self.dt):
                delta_out = self.delta_out_buffer[1] - self.delta_out_rate_max*self.dt
                self.delta_out_buffer[0] = delta_out
            # deadband
            if(-self.db_range < delta_out < self.db_range):
                delta_out = 0
    
            # set platform specific cmd and publish
            if(self.system_setup == "rhino_real"):
                # map to throttle if pos acc
                if(dc_out >= self.db_range_acc):        # accelerating
                    self.cmd.acceleration = 20.0*dc_out # (TUNE LONG)
                elif(dc_out < -self.db_range_acc):      # braking
                    self.cmd.acceleration = dc_out
                else:                                   # deadband
                    self.cmd.acceleration = 0.0
 
    
                # saturate output for safety
                throttle_max = 40.0 # (TUNE LONG) (%) 
                brake_max = -2.0  # (TUNE LONG) (m/s^2)
                self.cmd.acceleration = float(np.clip(self.cmd.acceleration, a_min = brake_max, a_max = throttle_max))
                
                if(self.cmd.acceleration == throttle_max or self.cmd.acceleration == brake_max):
                    rospy.logwarn_throttle(1,"saturated logitudinal command in tamp_ctrl, self.cmd.acceleration = " + str(self.cmd.acceleration))
  
                # set steering and stamp
                self.cmd.steering = delta_out  
                self.cmd.header.stamp = rospy.Time.now()

             
            elif(self.system_setup == "rhino_fssim" or self.system_setup == "gotthard_fssim"):
                self.cmd.delta = delta_out
                self.cmd.dc = dc_out
            
            # publish
            self.cmdpub.publish(self.cmd)

            # publish tuning info
            #self.vx_errorpub.publish(self.vx_error)

            self.rate.sleep()
            
        
    def tamp_ctrl(self):
        rospy.loginfo_throttle(1, "Running TAMP control")

        # LATERAL CTRL
        kin_ff_method = "polyfit" # 0: pure pursuit, 1: polyfit 
        if(kin_ff_method == "pure_pursuit"):
            kin_ff_term = self.kinematic_ff_by_pp()
        elif(kin_ff_method == "polyfit"):
            kin_ff_term = self.kinematic_ff_by_polyfit()
        elif(kin_ff_method == "dpsids"):
            kin_ff_term = self.kinematic_ff_by_dpsids()
            
        # dynamic feedfwd term (platform dependent)        
        if(self.system_setup == "rhino_real"):
            dyn_ff_term = 1.0*self.trajstar.Fyf[0]/self.trajstar.Cf[0] #0.75 # 0.9 
        elif(self.system_setup == "rhino_fssim"):
            dyn_ff_term = 0.9*self.trajstar.Fyf[0]/self.trajstar.Cf[0]
        elif(self.system_setup == "gotthard_fssim"):
            dyn_ff_term = 0.1*self.trajstar.Fyf[0]/self.trajstar.Cf[0]
        else:
            rospy.logerr("ctrl_interface: invalid value of system_setup param, system_setup = " + self.system_setup)
        delta_out = kin_ff_term + dyn_ff_term
        # saturate output
        if(self.system_setup == "rhino_real"):
            delta_out = float(np.clip(delta_out, a_min = -0.50, a_max = 0.50)) # delta_max = kappa_max*(lf+lr) (TUNE LAT)
        
        # LONGITUDINAL CTRL
        # feedfwd
        Fx_request = self.trajstar.Fxf[0] + self.trajstar.Fxr[0]
        self.vx_error = self.trajstar.vx[1]-self.state.vx
        if(self.system_setup == "rhino_real"):
            feedfwd = Fx_request/self.m
            feedback = 6.0*self.vx_error # TUNE LONG (probably not needed) (before: 6.0)
            dc_out = feedfwd + feedback
        elif(self.system_setup == "rhino_fssim"):
            feedfwd = Fx_request
            feedback = 50000*self.vx_error
            dc_out = feedfwd + feedback
        elif(self.system_setup == "gotthard_fssim"):
            feedfwd = 1.1*Fx_request 
            feedback = 0.0*self.vx_error
            Cr0 = 180
            Cm1 = 5000            
            dc_out = ((feedfwd+feedback)+Cr0)/Cm1           
        else:
            dc_out = 0
            rospy.logerr("ctrl_interface: invalid value of system_setup param, system_setup = " + self.system_setup)        
        
        return delta_out, dc_out


    def cc_ctrl(self):
        rospy.loginfo_throttle(1, "ctrl_interface: running CC control with vxref = " + str(self.cc_vxref))
        
        if(self.state.vx > 0.1):
            # get lhpt
            lhdist = 10 # 15 # todo determine from velocity
            s_lh = self.state.s + lhdist
            d_lh = self.cc_dref
            
            Xlh, Ylh = ptsFrenetToCartesian(np.array([s_lh]), \
                                            np.array([d_lh]), \
                                            np.array(self.pathlocal.X), \
                                            np.array(self.pathlocal.Y), \
                                            np.array(self.pathlocal.psi_c), \
                                            np.array(self.pathlocal.s))
            
            rho_pp = self.pp_curvature(self.state.X,self.state.Y,self.state.psi,Xlh[0],Ylh[0])
            delta_out = rho_pp*(self.lf + self.lr) # kinematic feed fwd
        else:
            Xlh = self.state.X
            Ylh = self.state.Y
            delta_out = 0.0
        
        # publish vis marker
        m = self.getlhptmarker(Xlh,Ylh)
        self.lhptpub.publish(m)
        
        self.vx_error = self.cc_vxref - self.state.vx
        if(self.system_setup == "rhino_real"):
            k = 0.2
            dc_out_unsat = k*self.vx_error
            # saturate output
            dc_out = float(np.clip(dc_out_unsat, a_min = -1.0, a_max = 1.0))
            if (dc_out_unsat != dc_out):
                rospy.logwarn_throttle(1,"saturated logitudinal command in cc_ctrl")
        elif(self.system_setup == "rhino_fssim"):
            k = 1500
            dc_out = k*self.vx_error
        elif(self.system_setup == "gotthard_fssim"):
            k = 500
            dc_out = k*self.vx_error
        else: 
            k = 0
            rospy.logerr("ctrl_interface: invalid value of system_setup param, system_setup = " + self.system_setup)
        
        return delta_out, dc_out
   
    def kinematic_ff_by_polyfit(self):
        fitdist_min = 5.0
        fitdist_max = 15.0
        fitdist = float(np.clip((np.min(self.trajstar.vx)/10.0)*fitdist_max, a_min = fitdist_min, a_max = fitdist_max))            
        sfit = np.linspace(self.state.s, self.state.s + fitdist, num=10)
        # rotate trajstar to vehicle frame xy
        deltaX = np.interp(sfit, self.trajstar.s, self.trajstar.X) - self.state.X
        deltaY = np.interp(sfit, self.trajstar.s, self.trajstar.Y) - self.state.Y
        x = deltaX*np.cos(self.state.psi) + deltaY*np.sin(self.state.psi)
        y = -deltaX*np.sin(self.state.psi) + deltaY*np.cos(self.state.psi)    
        # fit 3rd order polynomial
        #X_poly = np.vstack((x ** 3, x ** 2, x ** 1))
        X_poly = np.vstack((x ** 3, x ** 2, np.zeros(x.shape)))
        poly_coeffs = np.linalg.lstsq(X_poly.T, y,rcond=None)[0]
        x_eval = 0.0
        rho = (6.0*poly_coeffs[0]*x_eval + 2.0*poly_coeffs[1])/((1.0 + (3.0*poly_coeffs[0]*x_eval**2.0 + 2.0*poly_coeffs[1]*x_eval + poly_coeffs[2]))**1.5)
        y_fit = np.dot(poly_coeffs, X_poly)        
        #dydx = 3.0*poly_coeffs[0]*x_eval**2 + 2.0*poly_coeffs[1]*x_eval + poly_coeffs[2]
        kin_ff_term = rho*(self.lf + self.lr)# + np.arctan(dydx) 
        # publish vis marker
        m = self.getpolyfitmarker(x, y_fit)
        self.polyfitpub.publish(m)
        return kin_ff_term

    def kinematic_ff_by_pp(self):
        lhdist_min = 6.0
        lhdist_max = 10.0
        lhdist = float(np.clip((self.state.vx/10.0)*lhdist_max, a_min = lhdist_min, a_max = lhdist_max))
        s_lh = self.state.s + lhdist
        Xlh = np.interp(s_lh, self.trajstar.s, self.trajstar.X)
        Ylh = np.interp(s_lh, self.trajstar.s, self.trajstar.Y) 
        f1 = interp1d(self.trajstar.s, self.trajstar.X, kind='cubic')
        Xlh = f1(s_lh)
        f2 = interp1d(self.trajstar.s, self.trajstar.Y, kind='cubic')
        Ylh = f2(s_lh)
        rho_pp = self.pp_curvature(self.trajstar.X[0],
                                   self.trajstar.Y[0],
                                   self.trajstar.psi[0],
                                   Xlh,
                                   Ylh)       
        kin_ff_term = rho_pp*(self.lf + self.lr) 
        # publish vis marker
        m = self.getlhptmarker(Xlh,Ylh)
        self.lhptpub.publish(m) 
        return kin_ff_term

    def kinematic_ff_by_dpsids(self):
        s0 = 0.
        s1 = 5.
        delta_s = s1-s0
        psi0 = np.interp(self.state.s+s0, self.trajstar.s, self.trajstar.psi)
        psi1 = np.interp(self.state.s+s0, self.trajstar.s, self.trajstar.psi)
        deltapsi = psi1 - psi0 # todo angle to interval
        rho = deltapsi/delta_s
        kin_ff_term = rho*(self.lf + self.lr)
        return kin_ff_term
        
    def pp_curvature(self,Xego,Yego,psiego,Xlh,Ylh):
        deltaX = (Xlh-Xego)
        deltaY = (Ylh-Yego)
        lh_dist = np.sqrt(deltaX**2 + deltaY**2)
        lh_angle = np.arctan2(deltaY,deltaX) - psiego
        rho_pp = 2*np.sin(lh_angle)/lh_dist     
        return rho_pp

    def menger_curvature(self,x0,y0,x1,y1,x2,y2):
        signedarea = (x0*(y1-y2) + x1*(y2-y0) + x2*(y0-y1))/2.0
        d0 = np.sqrt((x0-x1)**2 + (y0-y1)**2)
        d1 = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        d2 = np.sqrt((x2-x0)**2 + (y2-y0)**2)
        rho_mn = 4.0*signedarea/(d0*d1*d2)
        return rho_mn

    def getpolyfitmarker(self, x, y):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "base_link"        
        # assume pose initialized to zero
        m.type = m.LINE_STRIP
        m.points = []
        for i in range(x.size):
            p = Point()
            p.x = x[i]
            p.y = y[i]
            p.z = 0.2
            m.points.append(p)
        m.scale.x = 0.2;
        m.color.a = 1.0; 
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        return m

    def getlhptmarker(self,Xlh,Ylh):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.pose.position.x = Xlh;
        m.pose.position.y = Ylh;
        m.pose.position.z = 0.1;
        m.type = m.SPHERE;
        m.scale.x = 0.6;
        m.scale.y = 0.6;
        m.scale.z = 0.6;
        m.color.a = 1.0; 
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0;
        return m

    def trajstar_callback(self, msg):
        self.trajstar = msg
        self.trajstar_received = True
  
    def pathlocal_callback(self, msg):
        self.pathlocal = msg
        self.pathlocal_received = True

    def state_callback(self, msg):
        self.state = msg 
        self.state_received = True
        
    def ctrl_mode_callback(self, msg):
        self.ctrl_mode = msg.data
        self.ctrl_mode_received = True
        
    def vxref_callback(self, msg):
        self.cc_vxref = msg.data
    
    def setStaticParams(self):
        self.g = rospy.get_param('/car/inertia/g')
        self.m = rospy.get_param('/car/inertia/m')
        self.lf = rospy.get_param('/car/kinematics/b_F')
        self.lr = rospy.get_param('/car/kinematics/b_R')

if __name__ == '__main__':
    ci = CtrlInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
