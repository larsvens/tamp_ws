#!/usr/bin/env python
import rospy
from common.msg import StaticVehicleParams

class ParamPublisher:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('liveplotter', anonymous=True)
        self.static_param_pub = rospy.Publisher('static_vehicle_params', StaticVehicleParams, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        # define static params, todo load from file
        self.static_params = StaticVehicleParams()   
        self.static_params.l = 4
        self.static_params.w = 1.8400
        self.static_params.m = 1500
        self.static_params.Iz = 2250
        self.static_params.lf = 1.0400
        self.static_params.lr = 1.4200
        self.static_params.Cf = 160000
        self.static_params.Cr = 180000
        self.static_params.Da = 0.3600
        self.static_params.deltamin = -0.3491
        self.static_params.deltamax = 0.3491
        self.static_params.g = 9.8200

        # Main loop
        while not rospy.is_shutdown():
            
            # publish static params
            self.static_params.header.stamp = rospy.Time.now()
            self.static_param_pub.publish(self.static_params)
            
            # end of main loop
            self.rate.sleep()            

if __name__ == '__main__':
    pp = ParamPublisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
