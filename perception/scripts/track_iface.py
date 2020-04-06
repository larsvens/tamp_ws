#!/usr/bin/env python

# Descrition: 
# load and publish global path 
# publish utm origin
# broadcast static tf tamp_map -> map

import numpy as np
import rospy
import rospkg
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path as navPath
from geometry_msgs.msg import PoseStamped
from common.msg import Path
from common.msg import OriginPoseUTM
from common.msg import MuSegments
from coordinate_transforms import ptsFrenetToCartesian
import yaml

class TrackInterface:
    def __init__(self):
        rospy.init_node('track_interface', anonymous=True)
        self.pathglobalpub = rospy.Publisher('pathglobal', Path, queue_size=5)
        self.pathglobalvispub = rospy.Publisher('pathglobal_vis', navPath, queue_size=1)
        self.originposeutmpub = rospy.Publisher('origin_pose_utm', OriginPoseUTM, queue_size=1)
        self.dubvispub = rospy.Publisher('dubglobal_vis', navPath, queue_size=1)
        self.dlbvispub = rospy.Publisher('dlbglobal_vis', navPath, queue_size=1)
        self.musegs_sub = rospy.Subscriber("/mu_segments", MuSegments, self.musegs_callback)
        self.pathglobal = Path()
        self.originposeUTM = OriginPoseUTM()
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # set params
        self.track_name = rospy.get_param('/track_name')
        self.dt = rospy.get_param('/dt_track_iface')
        self.rate = rospy.Rate(1.0/self.dt)
        self.N_delay_pub_pathglobal = rospy.get_param('/N_delay_pub_pathglobal')
        self.N_delay_repub_pathglobal = rospy.get_param('/N_delay_repub_pathglobal')
        
        # get track from yaml
        trackyaml = rospkg.RosPack().get_path('common') + '/config/tracks/' + self.track_name + '/' + self.track_name + '.yaml'
        with open(trackyaml, 'r') as f:
            track_ = yaml.load(f,Loader=yaml.SafeLoader)
        fcl_X = np.array(track_["centerline"])[:,0]
        fcl_Y = np.array(track_["centerline"])[:,1]
        psic_out = np.array(track_["tangent"])
        kappac_out = np.array(track_["curvature"])
        s      = np.array(track_["curvilinear_abscissa"])
        dub = np.array(track_["cones_left_normal_dist"])
        dlb = np.array(track_["cones_right_normal_dist"])
        N = s.size
        kappacprime_out = np.zeros(N)  # todo remove entirely
        self.pathglobal.X = fcl_X
        self.pathglobal.Y = fcl_Y
        self.pathglobal.s = s
        self.pathglobal.psi_c = psic_out
        self.pathglobal.kappa_c = kappac_out
        self.pathglobal.kappaprime_c = kappacprime_out
        self.pathglobal.theta_c = np.zeros(N) # grade/bank implement later
        self.pathglobal.dub = dub
        self.pathglobal.dlb = dlb

        # set initial mu (init to 1, set to whatever in callback)
        self.s_begin_mu_segments = [0]
        self.mu_segment_values = [1.0]
        self.N_mu_segments = len(self.s_begin_mu_segments)
        mu = self.get_mu_from_segments(self.pathglobal.s,self.s_begin_mu_segments,self.mu_segment_values,self.N_mu_segments)

        # wait for receiving nodes to launch
        count = self.N_delay_pub_pathglobal
        while(count > 0):
            self.rate.sleep()
            count = count - 1

        # put mu in message and publish        
        self.pathglobal.mu = mu        
        rospy.logwarn("track_iface: publishing pathglobal")
        self.pathglobalpub.publish(self.pathglobal)

        # build utm origin message
        self.originposeUTM.X0_utm = track_["origin_pose_utm"]["X0_utm"]
        self.originposeUTM.Y0_utm = track_["origin_pose_utm"]["Y0_utm"]
        self.originposeUTM.psi0_utm = track_["origin_pose_utm"]["psi0_utm"]
        self.originposeUTM.utm_nr = track_["origin_pose_utm"]["utm_nr"]
        self.originposeUTM.utm_letter = ord(track_["origin_pose_utm"]["utm_letter"])

        rospy.logwarn("track_iface: publishing origin pose UTM")
        self.originposeutmpub.publish(self.originposeUTM)

        # set map transform in TF
        rospy.logwarn("track_iface: setting initial pose in TF")
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "tamp_map"
        t.child_frame_id = 'map'
        t.transform.translation.z = 0.0
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


        # wait for rviz to launch
        count = 3
        while(count > 0):
            #print "publish pathglobal in " + str(count)
            self.rate.sleep()
            count = count - 1
        
        # publish paths for rviz
        pathglobalvis = navPath()
        for i in range(N):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = fcl_X[i]
            pose.pose.position.y = fcl_Y[i]            
            quaternion = tf.transformations.quaternion_from_euler(0, 0, psic_out[i])
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            pathglobalvis.poses.append(pose)
        pathglobalvis.header.stamp = rospy.Time.now()
        pathglobalvis.header.frame_id = "map"
        #rospy.logwarn("track_iface: publishing pathglobal visualization")
        self.pathglobalvispub.publish(pathglobalvis)

        # test correctness of dub and dlb in rviz
        Xleft,Yleft = ptsFrenetToCartesian(s,dub,fcl_X,fcl_Y,psic_out,s)
        pathleft = navPath()
        pathleft.header.stamp = rospy.Time.now()
        pathleft.header.frame_id = "map"
        for i in range(N):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = Xleft[i]
            pose.pose.position.y = Yleft[i]            
            pathleft.poses.append(pose)
        self.dubvispub.publish(pathleft)
        
        Xright,Yright = ptsFrenetToCartesian(s,dlb,fcl_X,fcl_Y,psic_out,s)
        pathright = navPath()        
        pathright.header.stamp = rospy.Time.now()
        pathright.header.frame_id = "map"
        for i in range(N):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = Xright[i]
            pose.pose.position.y = Yright[i]            
            pathright.poses.append(pose)
        self.dlbvispub.publish(pathright)    

        while not rospy.is_shutdown():
            # republish pathglobal every 5 seconds
            for i in range(self.N_delay_repub_pathglobal): 
                self.rate.sleep()
            self.pathglobalpub.publish(self.pathglobal)

    def get_mu_from_segments(self,s,s_begin_mu_segments,mu_segment_values,N_mu_segments):
        Ns = s.size
        mu = []
        for i in range(Ns): 
            mu_ele = self.mu_segment_values[-1]
            for j in range(self.N_mu_segments-1):
                if(self.s_begin_mu_segments[j]-0.01 <= s[i] <= self.s_begin_mu_segments[j+1]):
                    mu_ele = self.mu_segment_values[j]
                    break                    
            mu.append(mu_ele)
        mu = np.array(mu)        
        return mu

    def musegs_callback(self, msg):
        self.s_begin_mu_segments = msg.s_begin_mu_segments
        self.mu_segment_values = msg.mu_segment_values
        self.N_mu_segments = len(self.s_begin_mu_segments)
        mu = self.get_mu_from_segments(self.pathglobal.s,self.s_begin_mu_segments,self.mu_segment_values,self.N_mu_segments)
        
        # put mu in message and publish        
        self.pathglobal.mu = mu        
        rospy.logwarn("track_iface: publishing pathglobal")
        self.pathglobalpub.publish(self.pathglobal)
    
if __name__ == '__main__':
    ti = TrackInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
        
        