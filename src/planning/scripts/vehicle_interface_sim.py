#!/usr/bin/env python

import numpy as np
import rospy
from common.msg import VehicleIn
from common.msg import VehicleOut

def __init__(self):
    # init node subs pubs
    rospy.init_node('VehicleInterface', anonymous=True)
    self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
    self.dynamicparamsub = rospy.Subscriber("dynamic_vehicle_params", DynamicVehicleParams, self.dynamicparams_callback)
    self.pathlocalsub = rospy.Subscriber("pathlocal", PathLocal, self.pathlocal_callback)
    self.statepub = rospy.Publisher('state', State, queue_size=10)
    self.dt = 0.1
    self.rate = rospy.Rate(1.0/self.dt)
    #self.rate = rospy.Rate(2)

if __name__ == '__main__':
    vi = VehicleInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

