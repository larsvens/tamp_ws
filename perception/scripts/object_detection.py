#!/usr/bin/env python

# Descrition: Publishes object list

import rospy
from common.msg import Obstacles

class ObjectDetection:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('obj_det', anonymous=True)
        self.obstaclespub = rospy.Publisher('obstacles', Obstacles, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        # init obstacle list
        self.obstacles = Obstacles()
        self.obstacles.s = [540]
        self.obstacles.d = [-1.5]
        self.obstacles.R = [0.5]
        self.obstacles.Rmgn = [3.0] # 2.5 
        
        # Main loop
        while not rospy.is_shutdown():
            #print("in main loop")         
            self.obstacles.header.stamp = rospy.Time.now()
            self.obstaclespub.publish(self.obstacles)            
            self.rate.sleep()          

if __name__ == '__main__':
    od = ObjectDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    