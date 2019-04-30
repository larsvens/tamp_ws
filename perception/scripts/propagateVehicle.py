#!/usr/bin/env python

import rospy
from common.msg import State
from common.msg import StaticVehicleParams
from common.msg import DynamicVehicleParams

class VehicleModel:
    def __init__(self):
        # init node subs pubs
        rospy.init_node('vehiclemodel', anonymous=True)
        self.statesub = rospy.Subscriber("state", State, self.state_callback)
        self.staticparamsub = rospy.Subscriber("static_vehicle_params", StaticVehicleParams, self.staticparams_callback)
        self.dynamicparamsub = rospy.Subscriber("dynamic_vehicle_params", DynamicVehicleParams, self.dynamicparams_callback)
        self.dt = 0.1
        self.rate = rospy.Rate(1.0/self.dt) 
        
        # init internal vars
        self.state = State()
        self.sp = StaticVehicleParams()
        self.dp = DynamicVehicleParams()
        
        # main loop
        while not rospy.is_shutdown():
            print("in main loop")
            
            
            # end of main loop
            self.rate.sleep()
            
            
    def state_callback(self, msg):
        #print("in state callback")
        self.state = msg

    def staticparams_callback(self, msg):
        #print("in static params callback")
        self.sp = msg
        
    def dynamicparams_callback(self, msg):
        #print("in static params callback")
        self.dp = msg

if __name__ == '__main__':
    vm = VehicleModel()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    