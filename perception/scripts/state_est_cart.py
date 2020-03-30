#!/usr/bin/env python

# Descrition: Publishes state in cartesian coordinates 
# selects input topic based on sensor_setup param 
# sensor_setup = "rhino_real": /OpenDLV/SensorMsgGPS & /OpenDLV/SensorMsgCAN
# sensor_setup = "fssim": /fssim/base_pose_ground_truth

#import numpy as np
import rospy
from fssim_common.msg import State as fssimState
from common.msg import State
from common.msg import OriginPoseUTM
from opendlv_ros.msg import SensorMsgGPS
from opendlv_ros.msg import SensorMsgCAN 
import utm

class StateEstCart:
    # constructor
    def __init__(self):
        # init node
        rospy.init_node('state_est_cart', anonymous=True)
        self.dt = 0.01
        self.rate = rospy.Rate(1/self.dt)      
        
        # load rosparams
        self.robot_name = rospy.get_param('/robot_name')
        self.sensor_setup = rospy.get_param('/sensor_setup')

        # init local vars
        self.state_out = State()
       
        # init subs pubs
        if (self.sensor_setup == "rhino_real"):
            self.odlv_gps_sub = rospy.Subscriber("/OpenDLV/SensorMsgGPS", SensorMsgGPS, self.odlv_gps_callback)
            self.odlv_gps_msg = SensorMsgGPS()
            self.received_odlv_gps = False
            self.odlv_can_sub = rospy.Subscriber("/OpenDLV/SensorMsgCAN", SensorMsgCAN, self.odlv_can_callback)
            self.odlv_can_msg = SensorMsgCAN()
            self.received_odlv_can = False
            self.origin_pose_utm_sub = rospy.Subscriber("/origin_pose_utm", OriginPoseUTM, self.origin_pose_utm_callback)
            self.origin_pose_utm = OriginPoseUTM()
            self.received_origin_pose_utm = False
        else:
            self.fssim_state_sub = rospy.Subscriber("/fssim/base_pose_ground_truth", fssimState, self.fssim_state_callback)
            self.received_fssim_state = False
    
        self.statepub = rospy.Publisher('state_cart', State, queue_size=1)

        # wait for messages before entering main loop
        if (self.sensor_setup == "rhino_real"):
            while((not self.received_odlv_gps) or (not self.received_odlv_can)):
                rospy.loginfo_throttle(1, "state_est_cart: waiting opendlv messages")
                self.rate.sleep()
            while(self.received_origin_pose_utm):
                rospy.loginfo_throttle(1, "state_est_cart: waiting origin pose utm")
                self.rate.sleep()
        else:
            while(not self.received_fssim_state):
                rospy.loginfo_throttle(1, "state_est_cart: waiting fssim state message")
                self.rate.sleep()

        rospy.logwarn("state_est_cart: started with sensor setup " + self.sensor_setup)

        # Main loop
        while not rospy.is_shutdown():
            if (self.sensor_setup == "rhino_real"):
                self.update_rhino_state()
            
            self.rate.sleep() 

    def update_rhino_state(self):
        X, Y, utm_nr, utm_letter = utm.from_latlon(self.odlv_gps_msg.lat, self.odlv_gps_msg.long)
        
        # check utm zone
        if(utm_nr != self.origin_pose_utm.utm_nr):
            rospy.logerr("UTM zone mismatch: GPS measurement utm_nr =     " + str(utm_nr) + ", origin_pose utm_nr =     " + str(self.origin_pose_utm.utm_nr))
            rospy.logerr("UTM zone mismatch: GPS measurement utm_letter = " + utm_letter + ", origin_pose utm_letter = " + str(chr(self.origin_pose_utm.utm_letter)))


    def fssim_state_callback(self, msg):
        self.received_fssim_state = True
        self.state_out.X = msg.x
        self.state_out.Y = msg.y
        self.state_out.psi = msg.yaw
        self.state_out.psidot = msg.r
        self.state_out.vx = msg.vx
        self.state_out.vy = msg.vy
        self.state_out.header.stamp = rospy.Time.now()
        self.statepub.publish(self.state_out) # publish in callback to minimize delay

    def odlv_gps_callback(self, msg):
        self.odlv_gps_msg = msg
        self.received_odlv_gps = True
        
    def odlv_can_callback(self, msg):
        self.odlv_can_msg = msg
        self.received_odlv_can = True

    def origin_pose_utm_callback(self, msg):
        self.origin_pose_utm = msg
        self.received_origin_pose_utm = True

if __name__ == '__main__':
    sec = StateEstCart()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   