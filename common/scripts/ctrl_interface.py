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
        
        # delay sim variable
        self.delta_out_FIFO = []

        # wait for messages before entering main loop
        while(not self.trajstar.Fyf):
            print("waiting for trajstar")
            self.rate.sleep()

        # main loop
        while not rospy.is_shutdown(): 
            X = self.vehicle_out.x
            Y = self.vehicle_out.y
            psi = self.vehicle_out.yaw
            psidot = self.vehicle_out.r
            vx = self.vehicle_out.vx
            vy = self.vehicle_out.vy
            
            #
            # LATERAL CTRL
            #
            
            # compute local curvature of trajhat (rho)
            lhpt_idx = 5;
            lhpt = {"X": self.trajstar.X[lhpt_idx], "Y": self.trajstar.Y[lhpt_idx]}
            deltaX = (lhpt["X"]-X)
            deltaY = (lhpt["Y"]-Y)
            lh_dist = np.sqrt(deltaX**2 + deltaY**2)
            lh_angle = np.arctan2(deltaY,deltaX) - psi 
            rho_pp = 2*np.sin(lh_angle)/lh_dist
            shift = 1 # compensate for actuator delay
            rho_menger = self.menger_curvature(self.trajstar.X[0+shift],
                                               self.trajstar.Y[0+shift],
                                               self.trajstar.X[1+shift],
                                               self.trajstar.Y[1+shift],
                                               self.trajstar.X[2+shift],
                                               self.trajstar.Y[2+shift])
           
            print "rho_pp =     ", rho_pp
            print "rho_menger = ", rho_menger
            
            

            

          
            # compute delta corresponding to Fyf request (linear tire, Rajamani) 
            Fyf_request = self.trajstar.Fyf[0]
            alpha_f = Fyf_request/(2*self.Cf)

            # compute angle of velocity vector at center of front axle 
            if(vx < 1.0):
                theta_Vf = np.arctan2(vy+self.lf*psidot,1.0)
            else:
                theta_Vf = np.arctan2(vy+self.lf*psidot,vx)

            # compute control
            #delta_out = rho_pp*(self.lf + self.lr) # kinematic feed fwd
            delta_out = rho_menger*(self.lf + self.lr) # kinematic feed fwd menger



            #
            # LONGITUDINAL CTRL
            #
            Fx_request = self.trajstar.Fxf[0] + self.trajstar.Fxr[0]
            # todo: const double Fx = dc * param_.driveTrain.cm1 - aero_.getFdrag(x) - param_.driveTrain.cr0;

            # compute velocity error
            self.vx_error = self.trajstar.vx[1]-vx
            
            # feedfwd 
            Cr0 = 180
            Cm1 = 5000          
            dc_out = (Fx_request+Cr0)/Cm1 # not including aero
            
            # old
            #dc_out = 0.00010*self.trajstar.Fx[0]
  

            # simulate actuatiuon delays (0.2-0.3s)
#            Ndelay = 20 
#            self.delta_out_FIFO.append(delta_out)
#            if len(self.delta_out_FIFO) >= Ndelay:
#                delta_out = self.delta_out_FIFO.pop(0)
                
                
            
            
            # prints 
            print ""
            print "params"
            print "lf =     ", str(self.lf)
            print "Cf       ", str(self.Cf)
            print ""
            print "state vars"
            print "vx =     ", str(vx)
            print "vy =     ", str(vy)
            print "psidot = ", str(psidot)
            print ""            
            print "computed vars"
            print "theta_Vf = ", str(theta_Vf)
            print "Fyf_request = ", str(Fyf_request)
            print "alpha_f_request = ", str(alpha_f)
            print "delta_out = ", str(delta_out) 
            print "dc_out = ", str(dc_out)                    

            # publish ctrl cmd            
            self.vx_errorpub.publish(self.vx_error)
            self.vehicle_in.delta = delta_out
            self.vehicle_in.dc = dc_out
            self.vehicleinpub.publish(self.vehicle_in)
            
            # publish visualization
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
            self.lhptpub.publish(m)
            self.rate.sleep()

    def menger_curvature(self,x0,y0,x1,y1,x2,y2):
        signedarea = (x0*(y1-y2) + x1*(y2-y0) + x2*(y0-y1))/2.0
        d0 = np.sqrt((x0-x1)**2 + (y0-y1)**2)
        d1 = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        d2 = np.sqrt((x2-x0)**2 + (y2-y0)**2)
        rho = 4.0*signedarea/(d0*d1*d2)
        return rho

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
