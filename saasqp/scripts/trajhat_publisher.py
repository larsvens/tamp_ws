#!/usr/bin/env python
import numpy as np
import scipy.io as sio # for loading .mat file
import rospy
from std_msgs.msg import String
from common.msg import Trajectory

def trajhat_publisher():
    pub = rospy.Publisher('trajhat', Trajectory, queue_size=10)
    rospy.init_node('trajhat_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # load .mat file
    filename = '/home/larsvens/ros/saasqp_ws/src/saasqp/data/trajhat_example.mat'
    mat = sio.loadmat(filename,struct_as_record=False, squeeze_me=True)
    
    while not rospy.is_shutdown():
        trajhat = Trajectory()
        trajhat.t = mat['traj_hat'].t
        trajhat.kappac = mat['traj_hat'].kappa_c
        trajhat.s  = mat['traj_hat'].s
        trajhat.d = mat['traj_hat'].d
        trajhat.X = mat['traj_hat'].X
        trajhat.Y = mat['traj_hat'].Y
        trajhat.deltapsi = mat['traj_hat'].deltapsi
        trajhat.psi = mat['traj_hat'].psi
        trajhat.psidot = mat['traj_hat'].psidot
        trajhat.vx = mat['traj_hat'].vx
        trajhat.vy = mat['traj_hat'].vy
        trajhat.ax = mat['traj_hat'].ax
        trajhat.ay = mat['traj_hat'].ay
        trajhat.Fyf = mat['traj_hat'].Fyf
        trajhat.Fxf = mat['traj_hat'].Fxf
        trajhat.Fyr = mat['traj_hat'].Fyr
        trajhat.Fxr = mat['traj_hat'].Fxr
        trajhat.Fx = mat['traj_hat'].Fx
        trajhat.alphaf = mat['traj_hat'].alphaf
        trajhat.alphar = mat['traj_hat'].alphar
        trajhat.Crtilde = mat['traj_hat'].Crtilde
        trajhat.delta = mat['traj_hat'].delta
        trajhat.valid = mat['traj_hat'].valid
        trajhat.coll_cost = mat['traj_hat'].coll_cost
        trajhat.cost = mat['traj_hat'].cost
        trajhat.header.stamp = rospy.Time.now()
        pub.publish(trajhat)
        rate.sleep()

if __name__ == '__main__':
    try:
        trajhat_publisher()
    except rospy.ROSInterruptException:
        pass
