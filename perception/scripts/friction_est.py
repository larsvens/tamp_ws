#!/usr/bin/env python

# Descrition: Friction estimation

# subscribes:
# state (incl s and rho)
# planned traj (incl s and mu)

# publishes: 
# friction estimate (todo determine format and how to incorporate in pathlocal/global)

import time
import numpy as np

#import matplotlib
#matplotlib.use('TkAgg')
#import matplotlib.pyplot as plt

import matplotlib.pyplot as plt

import rospy
from common.msg import State
from common.msg import Trajectory
from common.msg import MuEst
from std_msgs.msg import Float32MultiArray
class FrictionEstimation:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('friction_estimation', anonymous=True)
        self.rate = rospy.Rate(10)  # approx 10hz update rate of live plot
        
        self.state_sub = rospy.Subscriber("/state", State, self.state_callback)
        self.state = State()
        self.received_state = False
        
        self.trajstar_sub = rospy.Subscriber("/trajstar", Trajectory, self.trajstar_callback)
        self.trajstar = Trajectory()
        self.received_trajstar = False        
        
        self.mu_est_pub = rospy.Publisher("/mu_est", MuEst, queue_size=1)
        self.mu_est = MuEst()
        
        # test
        self.test_pub = rospy.Publisher("/test_est", Float32MultiArray, queue_size=1)
        
        # params
        self.N_buffer = rospy.get_param('/N_buffer_fe')
        self.idx_lh = rospy.get_param('/idx_lh')
        
        # init node vars
        self.s_perceived_buffer = np.zeros(self.N_buffer)
        self.rhof_perceived_buffer = np.zeros(self.N_buffer)
        self.rhor_perceived_buffer = np.zeros(self.N_buffer)

        self.s_planned_buffer = np.zeros(self.N_buffer*2) # 
        self.rhof_planned_buffer = np.zeros(self.N_buffer*2)
        self.rhor_planned_buffer = np.zeros(self.N_buffer*2)
        
        # init live plot
        self.do_live_plot = True
        if self.do_live_plot:
            fig = plt.figure(num=None, figsize=(24, 12), dpi=80, facecolor='w', edgecolor='k')
            ax = fig.add_subplot(111)
            rho_perceived_ln, = ax.plot(np.zeros(self.N_buffer),'b.',markersize=20)
#            rho_planned_ln    = ax.plot(np.zeros(self.N_buffer),'r.',markersize=20)
            ax.set_ylim([0,1])
            plt.ion()
            plt.show()  

        
        while(not self.received_state):
            rospy.loginfo_throttle(1, "friction_est: waiting for state")
            rospy.sleep(0.1)

        while(not self.received_trajstar):
            rospy.loginfo_throttle(1, "friction_est: waiting for trajectory")
            rospy.sleep(0.1)

        rospy.loginfo("friction_est: started") # no main loop - everything happens in state callback
        


        # live plot
        t = time.time()
        while not rospy.is_shutdown():
            
            if self.do_live_plot:
                rho_perceived_ln.set_ydata(self.rhof_perceived_buffer)
                rho_perceived_ln.set_xdata(np.arange(len(self.rhof_perceived_buffer)))
#                rho_planned_ln.set_ydata(self.self.rhof_planned)
#                rho_planned_ln.set_xdata(np.arange(len(self.rhof_planned)))
                
                plt.pause(0.0001)
                rospy.loginfo("friction_est: looptime plot update: " + str(time.time() - t))
                t = time.time()
                
            self.rate.sleep()
    
    def state_callback(self, msg):
        self.state = msg
        self.received_state = True
        
        if(self.state.vx > 1.0): # avoid issues at low speed (also we wont have useful data at very low speeds)
        
            # update buffers
            self.s_perceived_buffer = np.roll(self.s_perceived_buffer,1)
            self.s_perceived_buffer[0] = self.state.s
            self.rhof_perceived_buffer = np.roll(self.rhof_perceived_buffer,1)
            self.rhof_perceived_buffer[0] = self.state.rhof
            self.rhor_perceived_buffer = np.roll(self.rhor_perceived_buffer,1)
            self.rhor_perceived_buffer[0] = self.state.rhor
            
            s_lh = self.trajstar.s[self.idx_lh]
            rhof_lh = np.sqrt(self.trajstar.Fxf[self.idx_lh]**2 + self.trajstar.Fyf[self.idx_lh]**2)/self.trajstar.Fzf[self.idx_lh]
            rhor_lh = np.sqrt(self.trajstar.Fxr[self.idx_lh]**2 + self.trajstar.Fyr[self.idx_lh]**2)/self.trajstar.Fzr[self.idx_lh]
            self.s_planned_buffer = np.roll(self.s_planned_buffer,1)
            self.s_planned_buffer[0] = s_lh
            self.rhof_planned_buffer = np.roll(self.rhof_planned_buffer,1)
            self.rhof_planned_buffer[0] = rhof_lh
            self.rhor_planned_buffer = np.roll(self.rhor_planned_buffer,1)
            self.rhor_planned_buffer[0] = rhor_lh
            
            # if buffers are filled
            if (np.count_nonzero(self.s_perceived_buffer) == self.s_perceived_buffer.size and
                np.count_nonzero(self.s_planned_buffer) == self.s_planned_buffer.size):
                # interpolate planned 
                self.rhof_planned = np.interp(self.s_perceived_buffer, self.s_planned_buffer, self.rhof_planned_buffer)
                #rhor_planned = np.interp(self.s_perceived_buffer, self.s_planned_buffer, self.rhor_planned_buffer)
            
                
                # do mu est (tmp)
                mgn = 0.1
                self.mu_est.s = self.s_perceived_buffer
                self.mu_est.mu_est_mean = np.ones(self.N_buffer) # TMP
                for i in range(self.N_buffer):
                    if(self.rhof_perceived_buffer[i] < self.rhof_planned[i] - mgn ):
                        self.mu_est.mu_est_mean[i] = self.rhof_perceived_buffer[i]
                    
                self.mu_est.mu_est_sigma = np.zeros(self.N_buffer)
                self.mu_est.header.stamp = rospy.Time.now()
                self.mu_est_pub.publish(self.mu_est)
               
                testmsg = Float32MultiArray()
                testmsg.data = np.random.rand(10)
                self.test_pub.publish(testmsg)


                
                
    def trajstar_callback(self, msg):
        self.trajstar = msg
        self.received_trajstar = True


if __name__ == '__main__':
    fe = FrictionEstimation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
