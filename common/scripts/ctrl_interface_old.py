#!/usr/bin/env python

# Node description:
# translate trajstar into specific control inputs for the vehicle
# runs at 100 Hz

import numpy as np
import rospy
from common.msg import Trajectory
from fssim_common.msg import Cmd
from fssim_common.msg import State as fssimState

class CtrlInterface:
    def __init__(self):
        # init node subs pubs
        rospy.init_node('ctrl_interface', anonymous=True)
        self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
        self.vehicle_out_sub = rospy.Subscriber("/fssim/base_pose_ground_truth", fssimState, self.vehicle_out_callback)
        self.vehicleinpub = rospy.Publisher('/fssim/cmd', Cmd, queue_size=10)
        self.rate = rospy.Rate(10) # TMP!

        # set static vehicle params
        self.setStaticParams()

        # init msgs
        self.vehicle_in = Cmd()
        self.vehicle_out = fssimState()
        self.trajstar = Trajectory()
        
        # misc
        self.printcounter = 0 

        # wait for messages before entering main loop # tmp commented out!!
        while(not self.trajstar.Fyf):
            print("waiting for trajstar")
            self.rate.sleep()

        # main loop
        while not rospy.is_shutdown():
            
            # compute delta corresponding to Fyf request (linear tire, Rajamani) 
            Fyf_request = self.trajstar.Fyf[0]
            alpha_f = Fyf_request/(2*self.Cf)
            psidot = self.vehicle_out.r
            vx = self.vehicle_out.vx
            vy = self.vehicle_out.vy
            
            # compute angle of velocity vector at center of front axle 
            if(vx < 1.0):
                theta_Vf = np.arctan2(vy+self.lf*psidot,1.0)
            else:
                theta_Vf = np.arctan2(vy+self.lf*psidot,vx)

            # compute controls
            delta_out = 1.0*(theta_Vf + alpha_f) 
            dc_out = 0.00006*self.trajstar.Fx[0]
            
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
            self.vehicle_in.delta = delta_out
            self.vehicle_in.dc = dc_out
            self.vehicleinpub.publish(self.vehicle_in)

    def trajstar_callback(self, msg):
        self.trajstar = msg
    
    def vehicle_out_callback(self, msg):
        self.vehicle_out = msg

    def setStaticParams(self):
        self.Cf = 1000000  # Todo compute from magic formula
        self.lf = rospy.get_param('/car/kinematics/b_F')

if __name__ == '__main__':
    vm = CtrlInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
