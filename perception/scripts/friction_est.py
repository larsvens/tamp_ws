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
from std_msgs.msg import Float32
class FrictionEstimation:
    # constructor
    def __init__(self):
        # init node subs pubs
        rospy.init_node('friction_estimation', anonymous=True)
        self.rate = rospy.Rate(10)  # approx 10hz update rate of live plot

        # params
        self.N_buffer = rospy.get_param('/N_buffer_fe')
        self.N_est = 100
        self.idx_lh = rospy.get_param('/idx_lh')
        self.mu_mean_nominal = rospy.get_param('/mu_nominal')
        self.mu_sigma_nominal = rospy.get_param('/mu_sigma_nominal')
        self.Ff_util = rospy.get_param('/Ff_util')
        self.Fr_util = rospy.get_param('/Fr_util')

        # subs & pubs        
        self.state_sub = rospy.Subscriber("/state", State, self.state_callback)
        self.state = State()
        self.received_state = False
        
        self.trajstar_sub = rospy.Subscriber("/trajstar", Trajectory, self.trajstar_callback)
        self.trajstar = Trajectory()
        self.received_trajstar = False        
        
        self.mu_est_pub = rospy.Publisher("/mu_est", MuEst, queue_size=1)
        self.mu_est = MuEst()
        self.mu_est.s = np.zeros(self.N_est)
        self.mu_est.mu_est_mean = np.zeros(self.N_est)
        self.mu_est.mu_est_sigma = np.zeros(self.N_est)
                
        self.rho_diff_pub = rospy.Publisher("/rho_diff", Float32, queue_size=1)
        
        
        # init node vars
        self.s_perceived_buffer = np.zeros(self.N_buffer)
        self.rhof_perceived_buffer = np.zeros(self.N_buffer)
        self.rhor_perceived_buffer = np.zeros(self.N_buffer)

        self.s_planned_buffer = np.zeros(self.N_buffer*2) # 
        self.rhof_planned_buffer = np.zeros(self.N_buffer*2)
        self.rhor_planned_buffer = np.zeros(self.N_buffer*2)
        self.rhof_planned = np.zeros(self.N_buffer)
        
        # init live plot
        self.do_live_plot = True
        if self.do_live_plot:
            fig = plt.figure(num=None, figsize=(24, 12), dpi=80, facecolor='w', edgecolor='k')
            ax = fig.add_subplot(111)
            line_objs = ax.plot(np.arange(self.N_buffer),np.zeros(self.N_buffer),'b.',
                                 np.arange(self.N_buffer),np.zeros(self.N_buffer),'r.',
                                 np.arange(self.N_est),np.zeros(self.N_est),'k-', 
                                 np.arange(self.N_est),np.zeros(self.N_est),'k--', 
                                 np.arange(self.N_est),np.zeros(self.N_est),'k--',ms = 20,lw = 5)

            rho_perceived_ln = line_objs[0]
            rho_planned_ln = line_objs[1]
            mu_est_mean_ln = line_objs[2]
            conf_intv_ub_ln = line_objs[3]
            conf_intv_lb_ln = line_objs[4]
            
            ax.legend(['perceived','planned', 'mu est mean', 'conf_itv'])
            ax.set_xlabel('s (m)')
            ax.set_ylabel('mu')
            
            ax.set_ylim([0,1])
            plt.ion()
            plt.show()  

        
        while(not self.received_state):
            rospy.loginfo_throttle(1, "friction_est: waiting for state")
            rospy.sleep(0.1)

        while(not self.received_trajstar):
            rospy.loginfo_throttle(1, "friction_est: waiting for trajectory")
            rospy.sleep(0.1)

        rospy.loginfo("friction_est: started") 
        
        # main loop - only live plot
        t = time.time()
        while not rospy.is_shutdown():
                        
            # set the prior (TODO SHIFT FWD INSTEAD OF RESETTING)
            #mu_est_mean_prior = self.mu_mean_nominal*np.ones(self.N_buffer) 
            #mu_est_sigma_prior = self.mu_sigma_nominal*np.ones(self.N_buffer)
            self.mu_est.s = np.linspace(self.s_perceived_buffer[0],self.s_perceived_buffer[-1] + (self.s_perceived_buffer.max()-self.s_perceived_buffer.min()),self.N_est) 
            
            self.mu_est.mu_est_mean = 0.6*np.ones(self.N_est) 
            self.mu_est.mu_est_sigma = 0.2*np.ones(self.N_est)
                        
            # extract measurements from buffers
            mgn = 0.1
            s_train = []
            mu_train = []
            for i in range(self.N_buffer):
                if(self.rhof_perceived_buffer[i] < self.rhof_planned[i]*self.Ff_util - mgn ):
                    #self.mu_est.mu_est_mean[i] = self.rhof_perceived_buffer[i]
                    s_train.append(self.s_perceived_buffer[i])
                    mu_train.append(self.rhof_perceived_buffer[i])
            
            # GP regression
            if (len(s_train) > 0):
                s_train = np.reshape(s_train,(len(s_train),1))
                mu_train = np.reshape(mu_train,(len(mu_train),1))
                
                s_est = np.reshape(self.mu_est.s,(len(self.mu_est.s),1))
                mean, sigma, K = self.posterior(s_train,mu_train,s_est,5.0,0.01,0.25)
                
                self.mu_est.mu_est_mean = mean.flatten()
                self.mu_est.mu_est_sigma = sigma.flatten()
                
                self.mu_est.header.stamp = rospy.Time.now()
                self.mu_est_pub.publish(self.mu_est)
            
            
            
            if self.do_live_plot:
                x_axis_buf = self.s_perceived_buffer-self.state.s
                x_axis_est = self.mu_est.s-self.state.s
                
                rho_perceived_ln.set_xdata(x_axis_buf)
                rho_perceived_ln.set_ydata(self.rhof_perceived_buffer)
                
                rho_planned_ln.set_xdata(x_axis_buf) 
                rho_planned_ln.set_ydata(self.rhof_planned) 
                
                #rospy.loginfo("friction_est: len x: " + str(len(x_axis)))
                #y_axis = np.array(self.mu_est.mu_est_mean) + 1.96*np.array(self.mu_est.mu_est_sigma)
                #rospy.loginfo("friction_est: max y: " + str(np.max(y_axis)))
                
                mu_est_mean_ln.set_xdata(x_axis_est)
                mu_est_mean_ln.set_ydata(self.mu_est.mu_est_mean )
                
                conf_intv_ub_ln.set_xdata(x_axis_est)
                conf_intv_ub_ln.set_ydata(self.mu_est.mu_est_mean + 1.96*np.array(self.mu_est.mu_est_sigma))
                
                conf_intv_lb_ln.set_xdata(x_axis_est)
                conf_intv_lb_ln.set_ydata(self.mu_est.mu_est_mean - 1.96*np.array(self.mu_est.mu_est_sigma))
                
                plt.pause(0.0001)
                ax.set_xlim([x_axis_buf[0], -x_axis_buf[0]])
                
                rospy.loginfo("friction_est: looptime plot update: " + str(time.time() - t))
                t = time.time()
                
            self.rate.sleep()
    
    def state_callback(self, msg):
        self.state = msg
        self.received_state = True
        
        if(self.state.vx > 1.0): # avoid issues at low speed (also we wont have useful data at very low speeds)
            
            # update buffers
            self.s_perceived_buffer = np.roll(self.s_perceived_buffer,-1)
            self.s_perceived_buffer[-1] = self.state.s
            self.rhof_perceived_buffer = np.roll(self.rhof_perceived_buffer,-1)
            self.rhof_perceived_buffer[-1] = self.state.rhof
#            self.rhor_perceived_buffer = np.roll(self.rhor_perceived_buffer,1)
#            self.rhor_perceived_buffer[0] = self.state.rhor
            
            s_lh = self.trajstar.s[self.idx_lh]
            rhof_lh = np.sqrt(self.trajstar.Fxf[self.idx_lh]**2 + self.trajstar.Fyf[self.idx_lh]**2)/self.trajstar.Fzf[self.idx_lh]
            #rhor_lh = np.sqrt(self.trajstar.Fxr[self.idx_lh]**2 + self.trajstar.Fyr[self.idx_lh]**2)/self.trajstar.Fzr[self.idx_lh]
            self.s_planned_buffer = np.roll(self.s_planned_buffer,-1)
            self.s_planned_buffer[-1] = s_lh
            self.rhof_planned_buffer = np.roll(self.rhof_planned_buffer,-1)
            self.rhof_planned_buffer[-1] = rhof_lh
            #self.rhor_planned_buffer = np.roll(self.rhor_planned_buffer,1)
            #self.rhor_planned_buffer[0] = rhor_lh
            
            # interp if buffers are filled
            if (np.count_nonzero(self.s_perceived_buffer) == self.s_perceived_buffer.size and
                np.count_nonzero(self.s_planned_buffer) == self.s_planned_buffer.size):
                
                # error if interpolation intervals do not overlap
                mgn = 0.5   
                if (self.s_perceived_buffer[0] < self.s_planned_buffer[0]-mgn):
                    rospy.logerr("friction est: improper overlap: s_perceived_buffer[0] = " + str(self.s_perceived_buffer[0]) + ", s_planned_buffer[0] = " + str(self.s_planned_buffer[0]))
                if (self.s_perceived_buffer[-1] > self.s_planned_buffer[-1]+mgn):
                    rospy.logerr("friction est: improper overlap: s_perceived_buffer[-1] = " + str(self.s_perceived_buffer[-1]) + ", s_planned_buffer[-1] = " + str(self.s_planned_buffer[-1]))
                                
                
                # interpolate planned 
                self.rhof_planned = np.interp(self.s_perceived_buffer, self.s_planned_buffer, self.rhof_planned_buffer)
                #rhor_planned = np.interp(self.s_perceived_buffer, self.s_planned_buffer, self.rhor_planned_buffer)

                # pub rho diff 
                msg = Float32()
                msg.data = self.rhof_planned[-1] - self.rhof_perceived_buffer[-1]
                self.rho_diff_pub.publish(msg)
           
                

               


    def posterior(self, Xtrain, ytrain, Xtest, l2=0.1, noise_var_train=1e-6,sigma_f=1.0):
        ytrain_zeromean = ytrain-np.mean(ytrain)
        
        # compute the mean at our test points.
        Ntrain = len(Xtrain)
        K = self.kernel(Xtrain, Xtrain, l2,sigma_f)
        L = np.linalg.cholesky(K + noise_var_train*np.eye(Ntrain))
        Lk = np.linalg.solve(L, self.kernel(Xtrain, Xtest, l2,sigma_f))
        mean = np.dot(Lk.T, np.linalg.solve(L, ytrain_zeromean)) + np.mean(ytrain)
        # compute the variance at our test points.
        K_ = self.kernel(Xtest, Xtest, l2,sigma_f)
        sd = np.sqrt(np.diag(K_) - np.sum(Lk**2, axis=0))
        return (mean, sd, K)

    def kernel(self,x, y, l2=0.1, sigma_f=1.0):
        sqdist = np.sum(x**2,1).reshape(-1,1) + \
                 np.sum(y**2,1) - 2*np.dot(x, y.T)
        return sigma_f**2 * np.exp(-.5 * (1/l2) * sqdist)                
                
    def trajstar_callback(self, msg):
        self.trajstar = msg
        self.received_trajstar = True


if __name__ == '__main__':
    fe = FrictionEstimation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
