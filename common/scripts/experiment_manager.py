#!/usr/bin/env python

'''
Description: This node
    - Publishes clock, controlling simulation time
    - controls pop-up obstacles
    - controls friction conditions
    - all other nodes shut down when this shuts down
'''

import time
import numpy as np
import rospy
from rosgraph_msgs.msg import Clock
from common.msg import Path
from common.msg import Obstacles
from common.msg import State
from fssim_common.msg import TireParams
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
from coordinate_transforms import ptsFrenetToCartesian

class ExperimentManager:
    # constructor
    def __init__(self):
        
        # timing params
        self.dt = 0.01 # timestep of the simulation
        self.t_activate = rospy.get_param('/t_activate')
        self.t_final = rospy.get_param('/t_final')
        
        # pop-up scenario params
        self.s_ego_at_popup = rospy.get_param('/s_ego_at_popup')
        self.s_obs_at_popup = rospy.get_param('/s_obs_at_popup')
        self.d_obs_at_popup = rospy.get_param('/d_obs_at_popup')
        
        # track params
        self.track_name = rospy.get_param('/track_name')
        self.s_begin_mu_segments = rospy.get_param('/s_begin_mu_segments')
        self.mu_segment_values = rospy.get_param('/mu_segment_values')
        self.N_mu_segments = len(self.s_begin_mu_segments)
        self.mu_segment_idx = 0
        
        # init node subs pubs
        rospy.init_node('experiment_manager', anonymous=True)
        #self.clockpub = rospy.Publisher('/clock', Clock, queue_size=10)   
        self.pathglobalsub = rospy.Subscriber("pathglobal", Path, self.pathglobal_callback)
        self.statesub = rospy.Subscriber("state", State, self.state_callback)
        self.obspub = rospy.Publisher('/obs', Obstacles, queue_size=1)
        self.obsvispub = rospy.Publisher('/obs_vis', Marker, queue_size=1)
        self.tireparampub = rospy.Publisher('/tire_params', TireParams, queue_size=1)
        self.ctrl_mode_pub = rospy.Publisher('/ctrl_mode', Int16, queue_size=1)
                     
        # init internal variables
        self.pathglobal = Path()
        self.received_pathglobal = False
        self.state = State()
        self.received_state = False

        while(not self.received_pathglobal):
            print("waiting for pathglobal")
            time.sleep(self.dt)

        # init experiment variables
        self.scenario_id = rospy.get_param('/scenario_id')
        self.tireparams = TireParams()
        self.obs = Obstacles()
        self.obs.s = [self.s_obs_at_popup]
        self.obs.d = [self.d_obs_at_popup]
        self.obs.R = [0.5]
        self.obs.Rmgn = [1.5]
        Xobs, Yobs = ptsFrenetToCartesian(np.array(self.obs.s), \
                                          np.array(self.obs.d), \
                                          np.array(self.pathglobal.X), \
                                          np.array(self.pathglobal.Y), \
                                          np.array(self.pathglobal.psi_c), \
                                          np.array(self.pathglobal.s))
        self.ctrl_mode = 0 # initial ctrl_mode: STOP

        # Main loop
        print 'running experiment: '
        print 'track: ', self.track_name
        self.t = 0 
        while (not rospy.is_shutdown()) and self.t<self.t_final :
            
            #print 'simtime t =', self.t
            if (self.t >= self.t_activate):
                
                
                # HANDLE TRACTION IN SIMULATION                
                s_ego = self.state.s % self.s_lap                
                for i in range(self.N_mu_segments-1):
                    if(self.s_begin_mu_segments[i] <= s_ego <= self.s_begin_mu_segments[i+1]):
                        self.mu_segment_idx = i
                        break
                if(s_ego >= self.s_begin_mu_segments[-1]):
                    self.mu_segment_idx = self.N_mu_segments-1
                      
                print "s_ego =              ", s_ego
                #print "state.s =            ", self.state.s
                #print "self.N_mu_segments = ", self.N_mu_segments
                print "mu_segment_idx =     ", self.mu_segment_idx
                print "mu in this section = ", self.mu_segment_values[self.mu_segment_idx] 
                
                # only vary D for now
                self.tireparams.tire_coefficient = 1.0 
                self.tireparams.B = 10
                self.tireparams.C = 1.9
                self.tireparams.D = -self.mu_segment_values[self.mu_segment_idx] 
                self.tireparams.E = 1.0            
                self.tireparams.header.stamp = rospy.Time.now()
                self.tireparampub.publish(self.tireparams)
                
                # POPUP SCENARIO
                if (self.scenario_id == 1):
                    self.ctrl_mode = 1 # cruise control
                    m = self.getobstaclemarker(Xobs,Yobs,self.obs.R[0])
                    m.color.a = 0.3 # transparent before detect
                    if (self.state.s >= self.s_ego_at_popup):
                        self.obspub.publish(self.obs)
                        m.color.a = 1.0 # non-transparent after detect
                        self.ctrl_mode = 2 # tamp
                    self.obsvispub.publish(m)
                
                # REDUCED MU TURN
                elif(self.scenario_id == 2):
                    self.ctrl_mode = 1 # cruise control
                    
                # RACING
                else:
                    self.ctrl_mode = 2 # tamp
                    

            # publish 
            self.ctrl_mode_pub.publish(self.ctrl_mode)
            
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

    def getobstaclemarker(self,X,Y,R):
        m = Marker()
        height =1.75
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.pose.position.x = X;
        m.pose.position.y = Y;
        m.pose.position.z = height/2.0;
        m.type = m.CYLINDER;
        m.scale.x = R;
        m.scale.y = R;
        m.scale.z = height;
        m.color.a = 1.0; 
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        return m

    def pathglobal_callback(self, msg):
        self.pathglobal = msg
        
        # get s of one lap
        stot_global = self.pathglobal.s[-1]
        dist_sf = np.sqrt( (self.pathglobal.X[0]-self.pathglobal.X[-1])**2 + (self.pathglobal.Y[0]-self.pathglobal.Y[-1])**2)
        self.s_lap = stot_global + dist_sf  
        
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
    
    
    
    



    
    
