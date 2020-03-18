#!/usr/bin/env python

# Descrition: 
# Subscribes to fssim topics
# Publishes state, local path and dynamic params

import numpy as np
import rospy
import rospkg
import tf
from nav_msgs.msg import Path as navPath
from geometry_msgs.msg import PoseStamped
from fssim_common.msg import Track
from common.msg import Path
from coordinate_transforms import ptsFrenetToCartesian
import yaml

class TrackInterface:
    def __init__(self):
        rospy.init_node('track_interface', anonymous=True)
        self.pathglobalpub = rospy.Publisher('pathglobal', Path, queue_size=10)
        self.pathglobalvispub = rospy.Publisher('pathglobal_vis', navPath, queue_size=10)
        self.dubvispub = rospy.Publisher('dubglobal_vis', navPath, queue_size=10)
        self.dlbvispub = rospy.Publisher('dlbglobal_vis', navPath, queue_size=10)
        self.track_sub = rospy.Subscriber("/fssim/track", Track, self.track_callback)
        self.track = Track()
        self.pathglobal = Path()
        self.received_track = False
        self.rate = rospy.Rate(1)
        
        # set params
        self.track_name = rospy.get_param('/track_name')
        self.s_begin_mu_segments = rospy.get_param('/s_begin_mu_segments')
        self.mu_segment_values = rospy.get_param('/mu_segment_values')
        self.N_mu_segments = len(self.s_begin_mu_segments)
        
        # wait for track
        while(not self.received_track):
            print "waiting for track"
            self.rate.sleep()      

        trackyaml = rospkg.RosPack().get_path('common') + '/config/tracks/' + self.track_name + '/' + self.track_name + '.yaml'
        #trackyaml = "/home/larsvens/ros/tamp__ws/src/saarti/common/config/tracks/frihamnen/frihamnen.yaml"
        with open(trackyaml, 'r') as f:
            track_ = yaml.load(f,Loader=yaml.SafeLoader)
        fcl_X = np.array(track_["centerline"])[:,0]
        fcl_Y = np.array(track_["centerline"])[:,1]
        #fll_X  = np.array(track_["cones_left"])[:,0]
        #fll_Y  = np.array(track_["cones_left"])[:,1]
        #frl_X  = np.array(track_["cones_right"])[:,0]
        #frl_Y  = np.array(track_["cones_right"])[:,1]
        psic_out = np.array(track_["tangent"])
        kappac_out = np.array(track_["curvature"])
        s      = np.array(track_["curvilinear_abscissa"])
        dub = np.array(track_["cones_left_normal_dist"])
        dlb = np.array(track_["cones_right_normal_dist"])
        N = s.size
        kappacprime_out = np.zeros(N)  # todo remove entirely
            
        # set mu 
        mu = []
        for i in range(N): 
            mu_ele = self.mu_segment_values[-1]
            for j in range(self.N_mu_segments-1):
                if(self.s_begin_mu_segments[j]-0.01 <= s[i] <= self.s_begin_mu_segments[j+1]):
                    mu_ele = self.mu_segment_values[j]
                    break                    
            mu.append(mu_ele)
        mu = np.array(mu)

        # put all in message and publish
        self.pathglobal.X = fcl_X
        self.pathglobal.Y = fcl_Y
        self.pathglobal.s = s
        self.pathglobal.psi_c = psic_out
        self.pathglobal.kappa_c = kappac_out
        self.pathglobal.kappaprime_c = kappacprime_out
        self.pathglobal.theta_c = np.zeros(N) # grade/bank implement later
        self.pathglobal.mu = mu
        self.pathglobal.dub = dub
        self.pathglobal.dlb = dlb
        
        print "publishing pathglobal"
        self.pathglobalpub.publish(self.pathglobal)

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
        print "publishing pathglobal visualization"
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

    def track_callback(self, msg):
        self.track = msg
        self.received_track = True

if __name__ == '__main__':
    ti = TrackInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
        
        