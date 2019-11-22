#!/usr/bin/env python

'''
Description: This node
    - Publishes clock, controlling simulation time
    - controls pop-up obstacles
    - controls friction conditions
    - all other nodes shut down when this shuts down
'''

import time
import rospy
from rosgraph_msgs.msg import Clock
from common.msg import Path
from common.msg import Obstacles
from common.msg import State
from fssim_common.msg import TireParams

class ExperimentManager:
    # constructor
    def __init__(self):
        
        # timing params
        self.dt = 0.01 # timestep of the simulation
        self.t_final = rospy.get_param('/t_final')
        
        # track params
        self.track_name = rospy.get_param('/track_name')
        self.N_mu_segments = rospy.get_param('/N_mu_segments')
        self.s_begin_mu_segments = rospy.get_param('/s_begin_mu_segments')
        self.mu_segment_values = rospy.get_param('/mu_segment_values')
        
        # obstacle params
        self.s_ego_at_popup = rospy.get_param('/s_ego_at_popup')
        self.s_diff_at_popup = rospy.get_param('/s_diff_at_popup')
        
        # init node subs pubs
        rospy.init_node('experiment_manager', anonymous=True)
        #self.clockpub = rospy.Publisher('/clock', Clock, queue_size=10)   
        self.pathglobalsub = rospy.Subscriber("pathglobal", Path, self.pathglobal_callback)
        self.statesub = rospy.Subscriber("state", State, self.state_callback)
        self.tireparampub = rospy.Publisher('/tire_params', TireParams, queue_size=1)
                     
        # init internal variables
        self.pathglobal = Path()
        self.received_pathglobal = False
        self.state = State()
        self.received_state = False
    
        self.tireparams = TireParams()
        self.obstacles = Obstacles()
    
    
        # Main loop
        print 'running experiment: '
        print 'track: ', self.track_name
        
        self.t = 0 
        while (not rospy.is_shutdown()) and self.t<self.t_final :
            
            print 'simtime t =', self.t
            
            # if position w.r.t global path (params use also in track iface )
            # pass friction values
            if self.state.s > 20.0: 
                self.tireparams.tire_coefficient = 1.0 
                self.tireparams.B = 10
                self.tireparams.C = 1.9
                self.tireparams.D = -0.3 # only vary D for now
                self.tireparams.E = 1.0
            else:
                self.tireparams.tire_coefficient = 1.0 
                self.tireparams.B = 10
                self.tireparams.C = 1.9
                self.tireparams.D = -1.5
                self.tireparams.E = 1.0              
                # todo function setting these from mu (from param)
            self.tireparams.header.stamp = rospy.Time.now()
            self.tireparampub.publish(self.tireparams)
            
            
            # EXP1 
            
            # if position w.r.t global path
            #      publish obstacle

            
            # handle simtime
            self.t += self.dt
            msg = Clock()
            t_rostime = rospy.Time(self.t)
            msg.clock = t_rostime
            #self.clockpub.publish(msg)
                                     
            
            time.sleep(self.dt)

        print 'simulation finished'
    
        # send shutdown signal
        message = 'run finished, shutting down'
        print message
        rospy.signal_shutdown(message)


    def pathglobal_callback(self, msg):
        self.pathglobal = msg
        self.received_pathglobal = True
        
    def state_callback(self, msg):
        self.state = msg
        self.received_state = True

    
    
if __name__ == '__main__':
    em = ExperimentManager()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
    
    



    
    
