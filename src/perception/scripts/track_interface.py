#!/usr/bin/env python

# Descrition: 
# Subscribes to fssim topics
# Publishes state, local path and dynamic params

import numpy as np
import matplotlib.pyplot as plt
import rospy
from fssim_common.msg import Track
from common.msg import Path

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 10, 10

class TrackInterface:
    def __init__(self):
        rospy.init_node('track_interface', anonymous=True)
        self.pathglobalpub = rospy.Publisher('pathglobal', Path, queue_size=10)
        self.track_sub = rospy.Subscriber("/fssim/track", Track, self.track_callback)
        self.track = Track()
        self.pathglobal = Path()
        self.received_track = False
        self.rate = rospy.Rate(1)
        plot_track = True
        
        # wait for track
        while(not self.received_track):
            print "waiting for track"
            self.rate.sleep()      

        # convert
        print len(self.track.cones_left)
        cl_x = []
        cl_y = []
        for i in range(len(self.track.cones_left)):
            cl_x.append(self.track.cones_left[i].x)
            cl_y.append(self.track.cones_left[i].y)            
 
        print len(self.track.cones_right)
        cr_x = []
        cr_y = []
        for i in range(len(self.track.cones_right)):    
            cr_x.append(self.track.cones_right[i].x)
            cr_y.append(self.track.cones_right[i].y) 
            
        # plot to see what we're doing
        if plot_track:
            fig, ax = plt.subplots()
            ax.axis("equal")
            ax.plot(cl_x,cl_y, '.k')
            ax.plot(cr_x,cr_y, '.k')
            
            plt.show()
        
        self.pathglobalpub.publish(self.pathglobal)


    def track_callback(self, msg):
        self.track = msg
        self.received_track = True

if __name__ == '__main__':
    ti = TrackInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
        
        