#!/usr/bin/env python

# Node description:
# translate trajstar into specific control inputs for the vehicle
# runs at 100 Hz

import rospy
from common.msg import Trajectory
from common.msg import VehicleIn

class CtrlInterface:
    def __init__(self):
        # init node subs pubs
        rospy.init_node('ctrl_interface', anonymous=True)
        self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
        self.vehicleinpub = rospy.Publisher('vehicle_in', VehicleIn, queue_size=10)
        self.rate = rospy.Rate(100)

        # init msgs
        self.vehicle_in = VehicleIn()
        self.trajstar = Trajectory()

        # wait for messages before entering main loop # tmp commented out!!
        while(not self.trajstar.Fyf):
            print("waiting for trajstar")
            self.rate.sleep()

        # main loop
        while not rospy.is_shutdown():
            # todo replace with delta, Fxf and Fxr
            self.vehicle_in.Fyf = self.trajstar.Fyf[0]
            self.vehicle_in.Fx = self.trajstar.Fx[0]
            self.vehicle_in.header.stamp = rospy.Time.now()
            self.vehicleinpub.publish(self.vehicle_in)

    def trajstar_callback(self, msg):
        self.trajstar = msg

if __name__ == '__main__':
    vm = CtrlInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
