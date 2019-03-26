#!/usr/bin/env python

import scipy.io as sio # for loading .mat file

# ros
import rospy
from std_msgs.msg import String
from common.msg import Trajectory


def trajhat_publisher():
    pub = rospy.Publisher('trajhat', Trajectory, queue_size=10)
    rospy.init_node('trajhat_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # load .mat file
    mat = sio.loadmat('/home/larsvens/ros/saasqp_ws/src/saasqp/data/trajhat_example.mat')
    
    while not rospy.is_shutdown():
        trajhat = Trajectory()
        
        trajhat.header.stamp = rospy.Time.now()
        
        pub.publish(trajhat)
        rate.sleep()

if __name__ == '__main__':
    try:
        trajhat_publisher()
    except rospy.ROSInterruptException:
        pass
