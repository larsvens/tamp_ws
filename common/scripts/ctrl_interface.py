#!/usr/bin/env python

# Node description:
# translate trajstar into specific control inputs for the vehicle
# runs at 100 Hz

import numpy as np
import rospy
from common.msg import Trajectory
from fssim_common.msg import Cmd
from fssim_common.msg import State as fssimState
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

class CtrlInterface:
    def __init__(self):
        
        # params
        self.robot_name = rospy.get_param('/robot_name')
        
        # init node subs pubs
        rospy.init_node('ctrl_interface', anonymous=True)
        self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
        self.vehicle_out_sub = rospy.Subscriber("/fssim/base_pose_ground_truth", fssimState, self.vehicle_out_callback)
        self.vehicleinpub = rospy.Publisher('/fssim/cmd', Cmd, queue_size=10)
        self.lhptpub = rospy.Publisher('/lhpt_vis', Marker, queue_size=1)
        self.vx_errorpub = rospy.Publisher('/vx_error_vis', Float32, queue_size=1)
        self.rate = rospy.Rate(100)

        # set static vehicle params
        self.setStaticParams()

        # init msgs
        self.vehicle_in = Cmd()
        self.vehicle_out = fssimState()
        self.trajstar = Trajectory()
        # ctrl errors
        self.vx_error = Float32()

        # mode in state machine
        # 0: stop
        # 1: cruise_ctrl
        # 2: tamp 
        self.ctrl_mode = 0 
        
        # delay sim variable
        self.delta_out_FIFO = []

        # wait for messages before entering main loop
        while(not self.trajstar.Fyf):
            print("waiting for trajstar")
            self.rate.sleep()

        # check if we are to bypass drivetrain dynamics
        if(self.robot_name == "snowfox"):
            self.bypass_drivetrain_dynamics = True
        else:
            self.bypass_drivetrain_dynamics = False

        # main loop
        while not rospy.is_shutdown(): 

            if(self.ctrl_mode == 0):     # STOP
                delta_out = 0
                dc_out = 0
            elif(self.ctrl_mode == 1):   # CRUISE CTRL
                print "cruise control"
            elif(self.ctrl_mode == 2):   # TAMP            
                lhpt_idx = 5;
                lhpt = {"X": self.trajstar.X[lhpt_idx], "Y": self.trajstar.Y[lhpt_idx]}            
                delta_out, dc_out = self.compute_tamp_ctrl(lhpt)
            else:
                print "invalid ctrl_mode! ctrl_mode = ", self.ctrl_mode
    
            # publish ctrl cmd            
            self.vehicle_in.delta = delta_out
            self.vehicle_in.dc = dc_out
            self.vehicleinpub.publish(self.vehicle_in)

            # publish tuning info
            self.vx_errorpub.publish(self.vx_error)
            if (self.ctrl_mode in [1,2]):
                m = self.getlhptmarker(lhpt)
                self.lhptpub.publish(m)
            

            self.rate.sleep()

    def compute_tamp_ctrl(self,lhpt):
          
        #X = self.vehicle_out.x
        #Y = self.vehicle_out.y
        #psi = self.vehicle_out.yaw
        #psidot = self.vehicle_out.r
        vx = self.vehicle_out.vx
        #vy = self.vehicle_out.vy
        
        # LATERAL CTRL
        # feedfwd
        
        # compute local curvature of trajhat (rho)
        rho_pp = self.pp_curvature(lhpt)
        print "rho_pp =     ", rho_pp
        
        # compute control
        delta_out = rho_pp*(self.lf + self.lr) # kinematic feed fwd
        
        # feedback
        # todo add feedback from yawrate


        # LONGITUDINAL CTRL
        # feedfwd
        Fx_request = self.trajstar.Fxf[0] + self.trajstar.Fxr[0]
        if(self.bypass_drivetrain_dynamics):
            dc_out = Fx_request
        else:
            # feedfwd 
            Cr0 = 180
            Cm1 = 5000          
            dc_out = (Fx_request+Cr0)/Cm1 # not including aero
        
        # feedback (todo)
        self.vx_error = self.trajstar.vx[1]-vx
        
        return delta_out, dc_out



    def pp_curvature(self,lhpt):
        deltaX = (lhpt["X"]-self.trajstar.X[0])
        deltaY = (lhpt["Y"]-self.trajstar.Y[0])
        lh_dist = np.sqrt(deltaX**2 + deltaY**2)
        lh_angle = np.arctan2(deltaY,deltaX) - self.trajstar.psi[0]
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


    def getlhptmarker(lhpt):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.pose.position.x = lhpt["X"];
        m.pose.position.y = lhpt["Y"];
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
    
    def vehicle_out_callback(self, msg):
        self.vehicle_out = msg

    def setStaticParams(self):
        self.Cf = 1000000  # Todo compute from magic formula
        self.lf = rospy.get_param('/car/kinematics/b_F')
        self.lr = rospy.get_param('/car/kinematics/b_R')

if __name__ == '__main__':
    vm = CtrlInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
