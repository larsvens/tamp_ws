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
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from util import angleToInterval
from util import angleToContinous
from opendlv_ros.msg import SensorMsgGPS
from opendlv_ros.msg import SensorMsgCAN 

class pos2DKalmanFilter:
    # constructor
    def __init__(self,dt,Qscale):
        self.x = np.array([[0.], # X
                           [0.], # Xdot
                           [0.], # Y
                           [0.]]) # Ydot  
        self.F = np.array([[1., dt, 0., 0.],
                              [0., 1., 0., 0.],
                              [0., 0., 1., dt],
                              [0., 0., 0., 1.]])
    
        self.H = np.array([[1.,0.,0.,0.],   # measurement function
                           [0.,0.,1.,0.]])         
        self.Q = Qscale*np.array([[1.0,    0.,    0.,    0.], 
                                  [0.,    1.0,    0.,    0.],
                                  [0.,    0.,    1.0,    0.],
                                  [0.,    0.,    0.,    1.0]])
        self.P = np.copy(self.Q) # init covariance matrix same as Q
        self.R = np.array([[0.1, 0.], # measurement noise
                           [0., 0.1]])

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(self.F, self.P).dot(self.F.T) + self.Q

    def update(self,z):
        S = np.dot(self.H, self.P).dot(self.H.T) + self.R
        K = np.dot(self.P, self.H.T).dot(np.linalg.pinv(S))
        y = z - np.dot(self.H, self.x)
        self.x += np.dot(K, y)
        self.P = self.P - np.dot(K, self.H).dot(self.P)   
    
class StateEstCart:
    # constructor
    def __init__(self):
        # init node
        rospy.init_node('state_est_cart', anonymous=True)
        self.dt = rospy.get_param('/dt_state_est_cart')
        self.rate = rospy.Rate(1./self.dt)      
        
        # load rosparams
        self.robot_name = rospy.get_param('/robot_name')
        self.system_setup = rospy.get_param('/system_setup')
        self.lf = rospy.get_param('/car/kinematics/b_F')
        self.lr = rospy.get_param('/car/kinematics/b_R')
        self.h_cg = rospy.get_param('/car/kinematics/h_cg')
        self.m = rospy.get_param('/car/inertia/m')
        self.g = rospy.get_param('/car/inertia/g')

        # init local vars
        self.state_out = State()
        self.live = False # todo incorporate in "system_setup"
        self.ts_latest_pos_update = rospy.Time.now()

        # init kf 
        Qscale = 0.01 
        self.kf = pos2DKalmanFilter(self.dt,Qscale)
    
        # load vehicle dimensions 
        dimsyaml = rospkg.RosPack().get_path('common') + '/config/vehicles/' + self.robot_name + '/config/distances.yaml'
        with open(dimsyaml, 'r') as f:
            self.dims = yaml.load(f,Loader=yaml.SafeLoader)   
       
        # heading filter
        self.psi_buffer_size = 20
        self.psi_buffer = np.zeros(self.psi_buffer_size)
        
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
        self.poserawpub = rospy.Publisher('/pose_raw_vis', Marker, queue_size=1)
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
        # HANDLE INCOMING DATA
        
        # get message delay times
        delta_t_gps = rospy.Time.now() - self.ts_latest_pos_update
        delta_t_can = rospy.Time.now() - self.odlv_can_msg.header.stamp

        # check message age
        msg_time_mgn = 0.1
        if(delta_t_gps.to_sec() > msg_time_mgn):
            rospy.logwarn("state_est_cart: Old GPS measurement, delta_t_gps.to_sec() = " + str(delta_t_gps.to_sec()))
#        else:
#            rospy.logwarn_throttle(1,"state_est_cart: GPS measurement age: = " + str(delta_t_gps.to_sec()))

        if(delta_t_can.to_sec() > msg_time_mgn):
            rospy.logwarn("state_est_cart: Old CAN measurement, delta_t_can.to_sec() = " + str(delta_t_can.to_sec()))        

        # incoming pos
        X_utm, Y_utm, utm_nr, utm_letter = utm.from_latlon(self.odlv_gps_msg.lat, self.odlv_gps_msg.long)
        X_raw = X_utm - self.origin_pose_utm.X0_utm
        Y_raw = Y_utm - self.origin_pose_utm.Y0_utm
        
        # check utm zone
        if(utm_nr != self.origin_pose_utm.utm_nr or utm_letter != chr(self.origin_pose_utm.utm_letter)):
            rospy.logerr("UTM zone mismatch: GPS measurement utm_nr =     " + str(utm_nr) + ", origin_pose utm_nr =     " + str(self.origin_pose_utm.utm_nr))
            rospy.logerr("UTM zone mismatch: GPS measurement utm_letter = " + utm_letter + ", origin_pose utm_letter = " + str(chr(self.origin_pose_utm.utm_letter)))

        # incoming yawangle field is heading (degrees 0 to 360, 0 North, increasing clockwise)
        # converting to psi (radians -pi to pi, 0 East, increasing counterclockwise )
        heading_raw = self.odlv_gps_msg.yawangle
        psi_raw = (np.pi/180)*(90-heading_raw) 
        psi_raw = angleToInterval(np.array([psi_raw]))[0]
        
        # filter psi
        #psi_raw_cont = angleToContinous(psi_raw)
        self.psi_buffer = np.roll(self.psi_buffer,1)
        #self.psi_buffer[0] = psi_raw_cont
        self.psi_buffer[0] = psi_raw
        #psi_raw_cont_filtered = np.median(self.psi_buffer)
        psi_filtered = np.median(self.psi_buffer)
        #psi_filtered = angleToInterval(psi_raw_cont_filtered)
        
        
        # convert heading-rate to yawrate
        psidot_raw = -self.odlv_gps_msg.yawrate*(np.pi/180)
        
        # velocities 
        vx_raw = self.odlv_can_msg.vx
        vy_raw = -self.odlv_gps_msg.vy # flipped sign convention oxgs

        # accelerations
        ax_raw = self.odlv_gps_msg.ax
        ay_raw = -self.odlv_gps_msg.ay # flipped sign convention oxgs
        
        # normal forces
        Fzf_raw = self.odlv_can_msg.load_axle_1*9.82
        Fzr_raw = self.odlv_can_msg.load_axle_2*9.82
        
        # publish raw pose marker
        m = self.get_pose_marker(X_raw,Y_raw,psi_raw)
        self.poserawpub.publish(m)
        
        # STATE EST
        
        # set velocities, acc and heading directly
        self.state_out.psidot = psidot_raw
        self.state_out.vx = vx_raw
        self.state_out.vy = vy_raw 
        self.state_out.ax = ax_raw
        self.state_out.ay = ay_raw 
        self.state_out.psi = psi_raw 
        self.state_out.Fzf = Fzf_raw
        self.state_out.Fzr = Fzr_raw

        # set position from KF
        z = np.array([[X_raw],
                      [Y_raw]])
        self.kf.predict()
        self.kf.update(z)      
        self.state_out.X = self.kf.x[0][0]
        self.state_out.Y = self.kf.x[2][0]

        # print errors if faulty state estimates
        if(self.state_out.psi < -np.pi or self.state_out.psi > np.pi):
            rospy.logerr("state_est_cart: psi outside interval, psi = " + str(self.state_out.psi))

    def get_pose_marker(self,X,Y,psi):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.pose.position.x = X;
        m.pose.position.y = Y;
        m.pose.position.z = 1.0;
        q = quaternion_from_euler(0, 0, psi)
        m.pose.orientation.x = q[0]
        m.pose.orientation.y = q[1]
        m.pose.orientation.z = q[2]
        m.pose.orientation.w = q[3]
        m.type = m.ARROW;
        m.scale.x = 2.0;
        m.scale.y = 0.6;
        m.scale.z = 0.6;
        m.color.a = 1.0; 
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        return m

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
        q = tf.transformations.quaternion_from_euler(-np.pi/2, 0., -np.pi/2.)
        self.tfbr.sendTransform((2.0, 0.0, 1.45),
                                (q[0], q[1], q[2], q[3]), 
                                rospy.Time.now(),
                                "cam",
                                "cog")  
        
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

    def get_normal_forces_from_motion(self,ax,theta):
        Fzf = (1.0/(self.lf+self.lr))*(self.m*ax*self.h_cg - self.m*self.g*self.h_cg*np.sin(theta) + self.m*self.g*self.lr*np.cos(theta));
        Fzr = (1.0/(self.lf+self.lr))*(-self.m*ax*self.h_cg + self.m*self.g*self.h_cg*np.sin(theta) + self.m*self.g*self.lf*np.cos(theta));
        return Fzf,Fzr

    def fssim_state_callback(self, msg):
        self.received_fssim_state = True
        self.state_out.X = msg.x
        self.state_out.Y = msg.y
        self.state_out.psi = angleToInterval(np.array([msg.yaw]))[0]
        self.state_out.psidot = msg.r
        self.state_out.vx = msg.vx
        self.state_out.vy = msg.vy
        self.state_out.ax = 0. # todo get ax from sim
        self.state_out.ay = 0.
        Fzf, Fzr = self.get_normal_forces_from_motion(self.state_out.ax,0.)
        self.state_out.Fzf = Fzf 
        self.state_out.Fzr = Fzr       
        self.state_out.header.stamp = rospy.Time.now()
        self.statepub.publish(self.state_out) # publish in callback to minimize delay

    def odlv_gps_callback(self, msg):
        # print errors if faulty measurements
        if(msg.yawangle < 0.0 or msg.yawangle > 360):
            rospy.logerr("state_est_cart: incoming heading outside interval, hdng = " + str(msg.yawangle))
        
        # store timestamp of latest incoming position
        if (self.odlv_gps_msg.vx >= 1.0 and not (self.odlv_gps_msg.lat == msg.lat and self.odlv_gps_msg.long == msg.long)):
            self.ts_latest_pos_update = rospy.Time.now()
        elif (self.odlv_gps_msg.vx < 1.0):
            self.ts_latest_pos_update = rospy.Time.now()
            
        # receive new message
        self.odlv_gps_msg = msg
        self.received_odlv_gps = True
                
        # restamp incoming msg if not live
        if(not self.live):
            self.odlv_gps_msg.header.stamp = rospy.Time.now()
        
    def odlv_can_callback(self, msg):
        self.odlv_can_msg = msg
        self.received_odlv_can = True
        # restamp incoming msg if not live        
        if(not self.live):
            self.odlv_can_msg.header.stamp = rospy.Time.now()

    def origin_pose_utm_callback(self, msg):
        self.origin_pose_utm = msg
        self.received_origin_pose_utm = True

if __name__ == '__main__':
    sec = StateEstCart()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   