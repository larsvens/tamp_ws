#!/usr/bin/env python


import rospy
from common.msg import State
from std_srvs.srv import Empty

s_pause = rospy.get_param('/s_sim_pause')
pausecounter = 0

#unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)


def state_callback(msg):
    global s_pause
    global pausecounter
    s = msg.s
    if (pausecounter < len(s_pause)):
        if (s > s_pause[pausecounter]):
            pause()
            pausecounter +=1
            
            # wait for user to reactivate
            raw_input("Simulation paused. Press Enter to unpause")
            unpause()
    
if __name__ == '__main__':

    rospy.init_node('tamp_gazebo_pauser')
    rospy.Subscriber("/state", State, state_callback)
    
    rospy.wait_for_service('gazebo/pause_physics')
    try:
        pause = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        unpause = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")  