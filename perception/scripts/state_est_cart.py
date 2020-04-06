#!/usr/bin/env python

# Descrition: Publishes state in cartesian coordinatesand broadcasts tf tamp_map -> base_link
# selects input topic based on system_setup param 
# system_setup = "rhino_real": /OpenDLV/SensorMsgGPS & /OpenDLV/SensorMsgCAN
# system_setup = "rhino_fssim": /fssim/base_pose_ground_truth


import numpy as np
import time
import utm
import yaml
import rospy
import tf
import rospkg
from fssim_common.msg import State as fssimState
from common.msg import State
from common.msg import OriginPoseUTM
from opendlv_ros.msg import SensorMsgGPS
from opendlv_ros.msg import SensorMsgCAN 

class StateEstCart:
    # constructor
    def __init__(self):
        # init node
        rospy.init_node('state_est_cart', anonymous=True)
        self.dt = rospy.get_param('/dt_state_est_cart')
        self.rate = rospy.Rate(1/self.dt)      
        
        # load rosparams
        self.robot_name = rospy.get_param('/robot_name')
        self.system_setup = rospy.get_param('/system_setup')
        self.lr = rospy.get_param('/car/kinematics/b_R')

        # init local vars
        self.state_out = State()

        # load vehicle dimensions 
        dimsyaml = rospkg.RosPack().get_path('common') + '/config/vehicles/' + self.robot_name + '/config/distances.yaml'
        with open(dimsyaml, 'r') as f:
            self.dims = yaml.load(f,Loader=yaml.SafeLoader)   
       
        # init subs pubs
        if (self.system_setup == "rhino_real"):
            self.odlv_gps_sub = rospy.Subscriber("/OpenDLV/SensorMsgGPS", SensorMsgGPS, self.odlv_gps_callback)
            self.odlv_gps_msg = SensorMsgGPS()
            self.received_odlv_gps = False
            self.odlv_can_sub = rospy.Subscriber("/OpenDLV/SensorMsgCAN", SensorMsgCAN, self.odlv_can_callback)
            self.odlv_can_msg = SensorMsgCAN()
            self.received_odlv_can = False
            self.origin_pose_utm_sub = rospy.Subscriber("/origin_pose_utm", OriginPoseUTM, self.origin_pose_utm_callback)
            self.origin_pose_utm = OriginPoseUTM()
            self.received_origin_pose_utm = False
        elif(self.system_setup == "rhino_fssim"):
            self.fssim_state_sub = rospy.Subscriber("/fssim/base_pose_ground_truth", fssimState, self.fssim_state_callback)
            self.received_fssim_state = False
        else: 
            rospy.logerr("state_est_cart: invalid value of system_setup param, system_setup = " + self.system_setup)
        self.statepub = rospy.Publisher('state_cart', State, queue_size=1)
        self.tfbr = tf.TransformBroadcaster()
        
        # wait for messages before entering main loop
        if (self.system_setup == "rhino_real"):
            while((not self.received_odlv_gps) or (not self.received_odlv_can)):
                rospy.loginfo_throttle(1, "state_est_cart: waiting opendlv messages")
                self.rate.sleep()
            while(not self.received_origin_pose_utm):
                rospy.loginfo_throttle(1, "state_est_cart: waiting origin pose utm")
                self.rate.sleep()
        elif(self.system_setup == "rhino_fssim"):
            while(not self.received_fssim_state):
                rospy.loginfo_throttle(1, "state_est_cart: waiting fssim state message")
                self.rate.sleep()

        rospy.logwarn("state_est_cart: started with sensor setup " + self.system_setup)

        # Main loop
        while not rospy.is_shutdown():
            
            # timing
            start = time.time()
            
            # state estimation
            if (self.system_setup == "rhino_real"):
                self.update_rhino_state()
                self.statepub.publish(self.state_out)
                
            # broadcast tf
            start_tfbc = time.time()
            self.broadcast_dyn_tfs()
            self.broadcast_static_tfs()
            end_tfbc = time.time()
            comptime_tfbc = end_tfbc-start_tfbc


            # timing: check wrt dt
            end = time.time()
            comptime = end-start
            if (comptime > self.dt):
                rospy.logwarn("state_est_cart: compute time exceeding dt!")
                rospy.logwarn("state_est_cart: total comptime =        " + str(comptime))
                rospy.logwarn("state_est_cart: comptime tf broadcast = " + str(comptime_tfbc))
            
            self.rate.sleep()




    def update_rhino_state(self):
        X_utm, Y_utm, utm_nr, utm_letter = utm.from_latlon(self.odlv_gps_msg.lat, self.odlv_gps_msg.long)
        
        # check utm zone
        if(utm_nr != self.origin_pose_utm.utm_nr):
            rospy.logerr("UTM zone mismatch: GPS measurement utm_nr =     " + str(utm_nr) + ", origin_pose utm_nr =     " + str(self.origin_pose_utm.utm_nr))
            rospy.logerr("UTM zone mismatch: GPS measurement utm_letter = " + utm_letter + ", origin_pose utm_letter = " + str(chr(self.origin_pose_utm.utm_letter)))
        
        # set pose
        psi_offset = -27*(np.pi/180)
        self.state_out.X = X_utm - self.origin_pose_utm.X0_utm
        self.state_out.Y = Y_utm - self.origin_pose_utm.Y0_utm
        self.state_out.psi = -self.odlv_gps_msg.yawangle + psi_offset
        
        # set velocities
        self.state_out.psidot = self.odlv_can_msg.yawrate
        self.state_out.vx = np.sqrt(self.odlv_gps_msg.vx**2 + self.odlv_gps_msg.vy**2)
        self.state_out.vy = self.state_out.psidot*self.lr
        
        #self.state_out.vx = self.odlv_gps_msg.vx*np.cos(-self.odlv_gps_msg.yawangle) - self.odlv_gps_msg.vy*np.sin(-self.odlv_gps_msg.yawangle)
        #self.state_out.vy = self.odlv_gps_msg.vx*np.sin(-self.odlv_gps_msg.yawangle) + self.odlv_gps_msg.vy*np.cos(-self.odlv_gps_msg.yawangle)
        
        

    def broadcast_dyn_tfs(self):
        self.tfbr.sendTransform((self.state_out.X, self.state_out.Y, 0),
                                tf.transformations.quaternion_from_euler(0, 0, self.state_out.psi),
                                rospy.Time.now(),
                                "base_link",
                                "tamp_map")
        
        # todo - steering angle here
#        self.tfbr.sendTransform((self.dims["left_steering_hinge"]["left_front_wheel"]["x"], self.dims["left_steering_hinge"]["left_front_wheel"]["y"], self.dims["left_steering_hinge"]["left_front_wheel"]["z"]),
#                                tf.transformations.quaternion_from_euler(0, 0, 1.0),
#                                rospy.Time.now(),
#                                "left_front_wheel",
#                                "left_steering_hinge") 
#
#        self.tfbr.sendTransform((self.dims["right_steering_hinge"]["right_front_wheel"]["x"], self.dims["right_steering_hinge"]["right_front_wheel"]["y"], self.dims["right_steering_hinge"]["right_front_wheel"]["z"]),
#                                tf.transformations.quaternion_from_euler(0, 0, 1.0),
#                                rospy.Time.now(),
#                                "right_front_wheel",
#                                "right_steering_hinge") 

    def broadcast_static_tfs(self):
        self.tfbr.sendTransform((self.dims["base_link"]["cog"]["x"], self.dims["base_link"]["cog"]["y"], self.dims["base_link"]["cog"]["z"]),
                                (0, 0, 0, 1), 
                                rospy.Time.now(),
                                "cog",
                                "base_link")         

        self.tfbr.sendTransform((self.dims["cog"]["chassis"]["x"], self.dims["cog"]["chassis"]["y"], self.dims["cog"]["chassis"]["z"]),
                                (0, 0, 0, 1), 
                                rospy.Time.now(),
                                "chassis",
                                "cog")  

        self.tfbr.sendTransform((self.dims["chassis"]["left_rear_wheel_joint"]["x"], self.dims["chassis"]["left_rear_wheel_joint"]["y"], self.dims["chassis"]["left_rear_wheel_joint"]["z"]),
                                (0, 0, 0, 1), 
                                rospy.Time.now(),
                                "left_rear_wheel_joint",
                                "chassis") 

        self.tfbr.sendTransform((self.dims["chassis"]["right_rear_wheel_joint"]["x"], self.dims["chassis"]["right_rear_wheel_joint"]["y"], self.dims["chassis"]["right_rear_wheel_joint"]["z"]),
                                (0, 0, 0, 1), 
                                rospy.Time.now(),
                                "right_rear_wheel_joint",
                                "chassis") 

        self.tfbr.sendTransform((self.dims["chassis"]["left_steering_hinge_joint"]["x"], self.dims["chassis"]["left_steering_hinge_joint"]["y"], self.dims["chassis"]["left_steering_hinge_joint"]["z"]),
                                (0, 0, 0, 1), 
                                rospy.Time.now(),
                                "left_steering_hinge",
                                "chassis") 

        self.tfbr.sendTransform((self.dims["chassis"]["right_steering_hinge"]["x"], self.dims["chassis"]["right_steering_hinge"]["y"], self.dims["chassis"]["right_steering_hinge"]["z"]),
                                (0, 0, 0, 1), 
                                rospy.Time.now(),
                                "right_steering_hinge",
                                "chassis") 


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
        
        # TMP FOR TESTING 

if __name__ == '__main__':
    sec = StateEstCart()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   