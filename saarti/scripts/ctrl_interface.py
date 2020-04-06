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
import rospy
from common.msg import Trajectory
from common.msg import Path
from common.msg import State
from fssim_common.msg import Cmd
from opendlv_ros.msg import ActuationRequest 
from visualization_msgs.msg import Marker
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
        self.lhptpub = rospy.Publisher('/lhpt_vis', Marker, queue_size=1)
        self.vx_errorpub = rospy.Publisher('/vx_error_vis', Float32, queue_size=1)
        self.rate = rospy.Rate(1/self.dt)
        if(self.system_setup == "rhino_real"):
            self.cmdpub = rospy.Publisher('/OpenDLV/ActuationRequest', ActuationRequest, queue_size=10)
        elif(self.system_setup == "rhino_fssim" or self.system_setup == "gotthard_fssim"):
            self.cmdpub = rospy.Publisher('/fssim/cmd', Cmd, queue_size=10)
        
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
        
        # ctrl errors
        self.vx_error = Float32()
        
        # get cc setpoint
        self.cc_vxref = rospy.get_param('/cc_vxref')
        self.cc_dref = rospy.get_param('/cc_dref')
        
        # delay sim variable
        self.delta_out_FIFO = []

        # misc vars
        self.delta_out_last = 0
        self.dc_out_last = 0

        # wait for messages before entering main loop
        while(not self.state_received):
            rospy.loginfo_throttle(1, "waiting for state")
            self.rate.sleep()

        while(not self.pathlocal_received):
            rospy.loginfo_throttle(1, "waiting for pathlocal")
            self.rate.sleep()

        while(self.ctrl_mode == 0):
            rospy.loginfo_throttle(1, "waiting for activation from exp manager")
            self.rate.sleep
            
        # main loop
        while not rospy.is_shutdown(): 
            
            if(self.ctrl_mode == 0):     # STOP (set by exp manager if outside of track)
                if (self.state.vx > 0.1):
                    rospy.loginfo_throttle(1,"in stop mode")
                    delta_out = self.delta_out_last
                    if(self.system_setup == "rhino_real"):
                        dc_out = 0.0
                    else:
                        dc_out = -200000 # todo set for other platforms
                else:
                    delta_out = 0.0
                    dc_out = 0.0
            elif(self.ctrl_mode == 1):   # CRUISE CTRL             
                delta_out, dc_out, Xlh,Ylh = self.cc_ctrl()           
                           
            elif(self.ctrl_mode == 2):   # TAMP   
                while(not self.trajstar_received):
                    print("waiting for trajstar, stopping")
                    delta_out = 0
                    dc_out = 0
                    self.rate.sleep()         
                delta_out, dc_out, Xlh, Ylh = self.tamp_ctrl()
            else:
                rospy.logerr("invalid ctrl_mode! ctrl_mode = " + str(self.ctrl_mode))
    
            # publish ctrl cmd
            if(self.system_setup == "rhino_real"):
                self.cmd = ActuationRequest()
                self.cmd.steering = delta_out
                
                # acc_rec -> throttle mapping
                if(dc_out >= 0):
                    ax_20_percent = 1.0 # approx acceleration at 20% throttle (TUNE THIS VALUE)
                    throttle_out = (20.0/ax_20_percent)*dc_out
                    self.cmd.acceleration = float(np.clip(throttle_out, a_min = 0.0, a_max = 20.0))
                else:    
                    self.cmd.acceleration = dc_out
                #self.cmd.acceleration = dc_out
                
            elif(self.system_setup == "rhino_fssim" or self.system_setup == "gotthard_fssim"):
                self.cmd = Cmd()
                self.cmd.delta = delta_out
                self.cmd.dc = dc_out
            self.cmd.header.stamp = rospy.Time.now()
            self.cmdpub.publish(self.cmd)

            # publish tuning info
            self.vx_errorpub.publish(self.vx_error)
            if (self.ctrl_mode in [1,2]):
                m = self.getlhptmarker(Xlh,Ylh)
                self.lhptpub.publish(m)

            # store latest controls
            self.delta_out_last = delta_out
            self.dc_out_last = dc_out

            self.rate.sleep()
    
    def cc_ctrl(self):
        rospy.loginfo_throttle(1, "ctrl_interface: running CC control with vxref = " + str(self.cc_vxref))
        
        if(self.state.vx > 0.1):
            # get lhpt
            lhdist = 7 # todo determine from velocity
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
        
        return delta_out, dc_out, Xlh,Ylh
        
        
    def tamp_ctrl(self):
        rospy.loginfo_throttle(1, "Running TAMP control")
        # LATERAL CTRL

        # kinematic feedfwd term
        lhpt_idx = 7;
        Xlh = self.trajstar.X[lhpt_idx]
        Ylh = self.trajstar.Y[lhpt_idx]
        rho_pp = self.pp_curvature(self.trajstar.X[0],
                                   self.trajstar.Y[0],
                                   self.trajstar.psi[0],
                                   Xlh,
                                   Ylh)       
        kin_ff_term = rho_pp*(self.lf + self.lr)

        # dynamic feedfwd term (platform dependent)        
        if(self.system_setup == "rhino_real"):
            dyn_ff_term = 0.9*self.trajstar.Fyf[0]/self.trajstar.Cf[0]
        elif(self.system_setup == "rhino_fssim"):
            dyn_ff_term = 0.9*self.trajstar.Fyf[0]/self.trajstar.Cf[0]
        elif(self.system_setup == "gotthard_fssim"):
            dyn_ff_term = 0.1*self.trajstar.Fyf[0]/self.trajstar.Cf[0]
        else:
            rospy.logerr("ctrl_interface: invalid value of system_setup param, system_setup = " + self.system_setup)
        delta_out = kin_ff_term + dyn_ff_term


        # LONGITUDINAL CTRL
        # feedfwd
        Fx_request = self.trajstar.Fxf[0] + self.trajstar.Fxr[0]
        self.vx_error = self.trajstar.vx[1]-self.state.vx
        if(self.system_setup == "rhino_real"):
            feedfwd = Fx_request/self.m
            feedback = 6.0*self.vx_error
            dc_out_unsat = feedfwd + feedback
            # saturate output
            dc_out = float(np.clip(dc_out_unsat, a_min = -1.0, a_max = 1.0))
            if (dc_out_unsat != dc_out):
                rospy.logwarn_throttle(1,"saturated logitudinal command in tamp_ctrl")
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
        
        return delta_out, dc_out, Xlh, Ylh

    def pp_curvature(self,Xego,Yego,psiego,Xlh,Ylh):
        deltaX = (Xlh-Xego)
        deltaY = (Ylh-Yego)
        lh_dist = np.sqrt(deltaX**2 + deltaY**2)
        lh_angle = np.arctan2(deltaY,deltaX) - psiego
        #print "lh_angle = ", lh_angle
        rho_pp = 2*np.sin(lh_angle)/lh_dist     
        return rho_pp

    def menger_curvature(self,x0,y0,x1,y1,x2,y2):
        signedarea = (x0*(y1-y2) + x1*(y2-y0) + x2*(y0-y1))/2.0
        d0 = np.sqrt((x0-x1)**2 + (y0-y1)**2)
        d1 = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        d2 = np.sqrt((x2-x0)**2 + (y2-y0)**2)
        rho_mn = 4.0*signedarea/(d0*d1*d2)
        return rho_mn

#    shift = 1 # compensate for actuator delay
#    rho_menger = self.menger_curvature(self.trajstar.X[0+shift],
#                                       self.trajstar.Y[0+shift],
#                                       self.trajstar.X[1+shift],
#                                       self.trajstar.Y[1+shift],
#                                       self.trajstar.X[2+shift],
#                                       self.trajstar.Y[2+shift])


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
        #print "self.state.s = " , self.state.s
        self.state_received = True
        
    def ctrl_mode_callback(self, msg):
        self.ctrl_mode = msg.data

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
