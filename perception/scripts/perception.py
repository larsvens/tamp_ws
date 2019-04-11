#!/usr/bin/env python

# Descrition: Publishes local path and object list

# todo publish object list

import scipy.io as sio # for loading .mat file
import rospy
from common.msg import PathLocal

class PerceptionSimple:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('perception', anonymous=True)
        self.pathlocalpub = rospy.Publisher('pathlocal', PathLocal, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        
        # init internal vars
        self.pathlocal = PathLocal()
        
        # load .mat file
        self.loadPathLocalFromFile();
        
        # Main loop
        while not rospy.is_shutdown():
            print("in main loop")
            self.pathlocal.header.stamp = rospy.Time.now()
            self.pathlocalpub.publish(self.pathlocal)
            self.rate.sleep()          

    def loadPathLocalFromFile(self):
        filename = '/home/larsvens/ros/saasqp_ws/src/perception/data/path_local_example.mat'
        mat = sio.loadmat(filename,struct_as_record=False, squeeze_me=True)
        self.pathlocal.X =              mat['path_local'].X
        self.pathlocal.Y =              mat['path_local'].Y
        self.pathlocal.s =              mat['path_local'].s
        self.pathlocal.psi_c =          mat['path_local'].psi_c
        self.pathlocal.theta_c =        mat['path_local'].theta_c
        self.pathlocal.kappa_c =        mat['path_local'].kappa_c
        self.pathlocal.kappa_c_prime =  mat['path_local'].kappa_c_prime
        self.pathlocal.dub =            mat['path_local'].dub
        self.pathlocal.dlb =            mat['path_local'].dlb

if __name__ == '__main__':
    ps = PerceptionSimple()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    