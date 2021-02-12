#!/usr/bin/env python

# Descrition: Publishes local path along with estimated friction along it

# subscribes:
# state from stateestimation node (topic /state)
# global path from track interface (topic /pathglobal)

# publishes: 
# local path (topic /pathlocal)
# visualization of local path (/pathlocal_vis)

import numpy as np
import rospy
from common.msg import State
from common.msg import Path

from util import angleToInterval
from util import angleToContinous
from coordinate_transforms import ptsFrenetToCartesian
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as navPath
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
import time

import GPy
import matplotlib.pyplot as plt

class RoadPerception:
    # constructor
    def __init__(self):

        # init node
        rospy.init_node('road_perception', anonymous=True)
        self.dt = rospy.get_param('/dt_road_perception')
        self.rate = rospy.Rate(1/self.dt)
        self.system_setup = rospy.get_param('/system_setup')
        
        # init node subs pubs
        self.pathglobalsub = rospy.Subscriber("pathglobal", Path, self.pathglobal_callback)
        self.pathglobal = Path()
        self.received_pathglobal = False
        
        self.state_sub = rospy.Subscriber("state", State, self.state_callback)
        self.state = State()
        self.received_state = False
        
        self.pathlocalpub = rospy.Publisher('pathlocal', Path, queue_size=10)
        self.pathlocal = Path()
        
        self.pathlocalvispub = rospy.Publisher('pathlocal_vis', PolygonArray, queue_size=1)
        
        if(self.system_setup == "rhino_fssim"):
            from fssim_common.msg import CarInfo
            self.carinfosub = rospy.Subscriber("/fssim/car_info", CarInfo, self.carinfo_callback)
            self.fssim_carinfo = CarInfo()
            self.received_fssim_carinfo = False
            
        # params of local path
        self.N = rospy.get_param('/N_pathlocal')
        self.ds = rospy.get_param('/ds_pathlocal')
        self.s_rel_start_pathlocal = rospy.get_param('/s_rel_start_pathlocal')
        
        # params of friction est emulation
        # 0 pred_gt: ground truth (same as camera only GT)
        # 1 local_gt: local only (GT)
        # 2 local_wce: local only (with noise)
        # 3 pred_wce: camera only (with noise)
        # 4 gpmrg_wce: merged local + camera (GP)-->
        self.mu_est_mode = rospy.get_param('/mu_est_mode')
        self.cons_level = rospy.get_param('/conservativeness_level')
        self.do_live_plot = rospy.get_param('/do_live_plot_mu_est')
        self.wce_pred = rospy.get_param('/wce_pred')
        self.wce_local = rospy.get_param('/wce_local')
        self.availability_th_local = rospy.get_param('/availability_th_local')
        
        # set static vehicle params
        self.m = rospy.get_param('/car/inertia/m')
        self.g = rospy.get_param('/car/inertia/g')
        self.lf = rospy.get_param('/car/kinematics/b_F')
        self.lr = rospy.get_param('/car/kinematics/b_R')
            
        # local vars
        self.pathrolling = Path() # used to run several laps
        self.rbf_ls = 15
        self.kern = GPy.kern.RBF(input_dim=1, variance=1.0, lengthscale=self.rbf_ls)
        #self.kern = GPy.kern.MLP(1)
        self.N_pl_50 = int(50.0/self.ds)
        mu_nominal = rospy.get_param('/mu_nominal')
        self.mu_est_local = mu_nominal
        self.Ff_util_buffer = np.zeros(3)
        N_histbuffer = 10
        # wait for state
        while (not self.received_state):
            rospy.logwarn_throttle(1,"road_perception: waiting for /state")
        self.s_mu_est_histbuffer = self.state.s -0.5*np.flip(np.arange(N_histbuffer))
        self.mu_est_histbuffer = np.ones(N_histbuffer)*mu_nominal
        self.mu_est_error_histbuffer = np.ones(N_histbuffer)*self.wce_pred
        
        # init live plot mu est
        if self.do_live_plot:

            fig = plt.figure(num=None, figsize=(24, 12), dpi=80, facecolor='w', edgecolor='k')
            self.ax = fig.add_subplot(111)
            line_objs = self.ax.plot(np.arange(self.N_pl_50),np.zeros(self.N_pl_50),'b.',
                                     np.arange(self.N_pl_50),np.zeros(self.N_pl_50),'r.',
                                     np.arange(self.N_pl_50),np.zeros(self.N_pl_50),'k-', 
                                     np.arange(self.N_pl_50),np.zeros(self.N_pl_50),'k--', 
                                     np.arange(self.N_pl_50),np.zeros(self.N_pl_50),'k--',
                                     np.arange(self.N_pl_50),np.zeros(self.N_pl_50),'g',ms = 20,lw = 5)

            self.mu_est_local_ln = line_objs[0]
            self.mu_est_pred_ln = line_objs[1]
            self.mu_est_gp_mean_ln = line_objs[2]
            self.mu_est_gp_ub_ln = line_objs[3]
            self.mu_est_gp_lb_ln = line_objs[4]
            self.mu_gt_ln = line_objs[5]
            
            # Hignlight active estimate
            if(self.cons_level == -1):
                self.mu_est_gp_lb_ln.set_color('m')
            elif(self.cons_level == 0):
                self.mu_est_gp_mean_ln.set_color('m')
            elif(self.cons_level == 1):
                self.mu_est_gp_ub_ln.set_color('m')
            
            self.ax.legend(['local','Ytrain', 'gp-mean', 'gp-itv'])
            self.ax.set_xlabel('s (m)')
            self.ax.set_ylabel('mu')
            
            self.ax.set_ylim([-0.1,1.1])
            plt.ion()
            plt.show()  


        # wait for messages before entering main loop
        while(not self.received_pathglobal):
            print "perception: waiting for pathglobal"
            self.rate.sleep()
        self.maxmu = np.max(self.pathglobal.mu)
        
        
        while(not self.received_state):
            print "perception: waiting for state"
            self.rate.sleep()

        print "road perception: running main with: "
        print "road perception: lap length: ", self.s_lap
        print "road perception: length of local path: ", self.N*self.ds
        
        # Main loop
        while not rospy.is_shutdown():
            
            # check timing wrt dt
            start = time.time()
 
            # update pathrolling to handle multiple laps
            if (self.state.s > self.pathrolling.s[0]+self.s_lap + 25): # if we're on the second lap of pathrolling
                self.pathrolling.s = self.pathrolling.s + self.s_lap
           
            # update local path 
            self.updateLocalPath()
            print self.pathlocal.s
            self.pathlocalpub.publish(self.pathlocal)
            
            # visualize local path in rviz
            pathlocal_vis = navPath()
            pathlocal_vis.header.stamp = rospy.Time.now()
            pathlocal_vis.header.frame_id = "map"
            for i in range(self.N):
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = self.pathlocal.X[i]
                pose.pose.position.y = self.pathlocal.Y[i]       
                pathlocal_vis.poses.append(pose)

            pa = self.pathLocalToPolArr()
            self.pathlocalvispub.publish(pa)

            end = time.time()
            comptime = end-start
            if (comptime > self.dt):
                rospy.logwarn("road_perception: compute time exceeding dt!") # TODO HOW MUCH?

            self.rate.sleep()   
            

    def updateLocalPath(self):
       
        stot_local = self.N*self.ds
        #self.smin_local = max(self.state.s-1, 0.0)
        self.smin_local = self.state.s-self.s_rel_start_pathlocal
        # at the start - make sure smin_local > 0
        if(self.smin_local < 0):
            self.smin_local = 0 

        smax_local = self.smin_local+stot_local
        s = np.linspace(self.smin_local,smax_local,self.N)
        
        if (s[0] < self.pathrolling.s[0]):
            rospy.logerr("perception: pathlocal.s not on pathrolling interval! (pathlocal is behind)")
            rospy.logerr("smin_pathlocal = " + str(s[0]) + ", pathrolling.s[0] = " + str(self.pathrolling.s[0]))
        if (s[-1] > self.pathrolling.s[-1]):
            rospy.logerr("perception: pathlocal.s not on pathrolling interval! (pathlocal is ahead)")
            rospy.logerr("smax_pathlocal = " + str(s[-1]) + ", pathrolling.s[-1] = " + str(self.pathrolling.s[-1]))
        
        # interpolate on global path            
        self.pathlocal.header.stamp = rospy.Time.now()
        self.pathlocal.header.frame_id = "map"
        self.pathlocal.X =              np.interp(s,self.pathrolling.s,self.pathrolling.X)
        self.pathlocal.Y =              np.interp(s,self.pathrolling.s,self.pathrolling.Y)
        self.pathlocal.s =              s
        self.pathlocal.psi_c =          np.interp(s,self.pathrolling.s,angleToContinous(self.pathrolling.psi_c)) # interpolating on continous psic
        self.pathlocal.psi_c =          angleToInterval(self.pathlocal.psi_c)
        self.pathlocal.theta_c =        np.interp(s,self.pathrolling.s,self.pathrolling.theta_c)
        self.pathlocal.kappa_c =        np.interp(s,self.pathrolling.s,self.pathrolling.kappa_c)
        self.pathlocal.kappaprime_c =   np.interp(s,self.pathrolling.s,self.pathrolling.kappaprime_c)
        
        self.pathlocal.dub =            np.interp(s,self.pathrolling.s,self.pathrolling.dub)
        self.pathlocal.dlb =            np.interp(s,self.pathrolling.s,self.pathrolling.dlb)

        # set mu        
        mu_gt = np.interp(s,self.pathrolling.s,self.pathrolling.mu)
        self.pathlocal.mu = self.emulateFrictionEst(self.pathlocal.s,
                                                    mu_gt,
                                                    self.mu_est_mode,
                                                    self.cons_level,
                                                    self.wce_local)   


    def emulateFrictionEst(self,s_pl,mu_gt,mu_est_mode,cons_level,wce_local):
        
        sf = self.state.s+self.lf
        mu_gt_local = self.nearest_interp(np.array([sf]),s_pl,mu_gt)
        loc_av = self.local_available(mu_gt_local)
        if(mu_est_mode == "pred_gt"): # 0 GT  
            mu_est = mu_gt
        elif(mu_est_mode == "local_gt"): # 1 local only (GT)
            if(self.local_available(mu_gt_local)):
                self.mu_est_local = mu_gt_local
            mu_est = self.mu_est_local*np.ones_like(mu_gt) # if not available return persistent var without update
        elif(mu_est_mode == "local_wce"): # 2 local with error
            if(self.local_available(mu_gt_local)):
                self.mu_est_local = mu_gt_local + wce_local
            
            if (self.cons_level == -1):
                mu_est = self.mu_est_local*np.ones_like(mu_gt)-abs(wce_local)
            else:
                mu_est = self.mu_est_local*np.ones_like(mu_gt)
            
            
        elif(mu_est_mode == "pred_wce"): # 3 predictive with error
            mu_est_pred, mu_est_pred_cons = self.get_closest_mu_class(mu_gt)
            if (self.cons_level == -1):
                mu_est = mu_est_pred_cons
            else:
                mu_est = mu_est_pred
        elif(mu_est_mode == "gpmrg_wce"): # 4 GP merge
            
            # emulate local and pred mu estimates
            if(loc_av):
                self.mu_est_local = mu_gt_local + wce_local
            mu_est_pred, mu_est_pred_cons = self.get_closest_mu_class(mu_gt)

            # cut out for camera range
            idx_ego = np.argmin(np.abs(s_pl-self.state.s))
            idx_cam_min_range = idx_ego
            s_pl_50 = s_pl[idx_cam_min_range:idx_cam_min_range+self.N_pl_50]
            mu_est_pred_50 = mu_est_pred[idx_cam_min_range:idx_cam_min_range+self.N_pl_50]
            
            # merge and specify uncertainty in individual measurements 
            mu_est_merged = mu_est_pred_50
            abs_mu_errors = (self.wce_pred)*np.ones_like(s_pl_50)
            if(loc_av):
                mu_est_merged[0] = self.mu_est_local
                abs_mu_errors[0] = self.wce_local

            # interpolate history onto pathlocal (use first value of hist buffer if pl is farther behind than buffer )
            s_pl_behind = s_pl[0:idx_ego]
            mu_est_hist_pl = self.nearest_interp(s_pl_behind, self.s_mu_est_histbuffer, self.mu_est_histbuffer)
            mu_est_error_hist_pl = self.nearest_interp(s_pl_behind,self.s_mu_est_histbuffer,self.mu_est_error_histbuffer)
                   
            # prepend history
            Xtrain = np.concatenate([s_pl_behind,s_pl_50])
            Ytrain = np.concatenate([mu_est_hist_pl,mu_est_merged])
            abs_errors_Y = np.concatenate([mu_est_error_hist_pl**2,abs_mu_errors**2])
            
            # gp regression
            offset = 0.5 # s.t. Ytrain is cenered around mean
            gp_posterior_mean, gp_posterior_std_dev = self.het_gp_regression(self.kern, Xtrain, Ytrain-offset, abs_errors_Y)
            gp_posterior_mean += offset
            if(self.cons_level == -1):
                gp_est = gp_posterior_mean - gp_posterior_std_dev
            elif(self.cons_level == 0):
                gp_est = gp_posterior_mean
            elif(self.cons_level == 1):
                gp_est = gp_posterior_mean + gp_posterior_std_dev
            
            # put estimate on pl shape
            mu_est = np.zeros_like(mu_gt)
            mu_est[0:idx_ego+self.N_pl_50] = gp_est.reshape(1, -1)
            mu_est[idx_ego+self.N_pl_50:] = gp_est[-1]  # set remainder of pathlocal same as furthest est
            
            
            # roll history buffer
            self.s_mu_est_histbuffer = np.roll(self.s_mu_est_histbuffer,-1)
            self.s_mu_est_histbuffer[-1] = self.state.s
            self.mu_est_histbuffer = np.roll(self.mu_est_histbuffer,-1)
            self.mu_est_histbuffer[-1] = mu_est_merged[0] # already set to local if available
            self.mu_est_error_histbuffer = np.roll(self.mu_est_error_histbuffer,-1)
            if(loc_av):                
                self.mu_est_error_histbuffer[-1] = self.wce_local
            else:
                self.mu_est_error_histbuffer[-1] = self.wce_pred
        
        else:
            rospy.logerr_throttle(1,"road perception: ERROR, faulty mu est mode")
        
        # update live plot
        if self.do_live_plot:
            
            if(loc_av):
                self.mu_est_local_ln.set_xdata(np.array([self.state.s]))
                self.mu_est_local_ln.set_ydata(np.array([self.mu_est_local]))
            else:
                self.mu_est_local_ln.set_xdata(np.array([self.state.s]))
                self.mu_est_local_ln.set_ydata(np.array([0.0])) # just to show state.s in plot 

            if(mu_est_mode == "pred_wce"): # 3 camera only with noise
                self.mu_est_pred_ln.set_xdata(s_pl) 
                self.mu_est_pred_ln.set_ydata(mu_est) 
            
            if(mu_est_mode == "gpmrg_wce"): # 4 GP merge
                self.mu_est_pred_ln.set_xdata(Xtrain) 
                self.mu_est_pred_ln.set_ydata(Ytrain) 
                
                self.mu_est_gp_mean_ln.set_xdata(Xtrain)
                self.mu_est_gp_mean_ln.set_ydata(gp_posterior_mean)
                
                self.mu_est_gp_ub_ln.set_xdata(Xtrain)
                self.mu_est_gp_ub_ln.set_ydata(gp_posterior_mean + 1.96*gp_posterior_std_dev)
                
                self.mu_est_gp_lb_ln.set_xdata(Xtrain)
                self.mu_est_gp_lb_ln.set_ydata(gp_posterior_mean - 1.96*gp_posterior_std_dev)
            
            self.mu_gt_ln.set_xdata(s_pl)
            self.mu_gt_ln.set_ydata(mu_gt)
            
            plt.pause(0.0001)
            self.ax.set_xlim([np.min(s_pl)-5, np.max(s_pl)])
            
        # return estimate
        return mu_est 

    def get_closest_mu_class(self,mu_gt):
        # 3 classes
        # snow/ice:    mu in [0.0 0.3) 
        # wet asphalt: mu in [0.3 0.6) 
        # dry asphalt: mu in [0.6 0.9) 
        mu_est_pred = 0.15 + np.round((mu_gt-0.15)/0.3)*0.3
        if any(mu_est_pred) not in [0.15,0.45,0.75]:
            print "WARNING: faulty mu class"
            
        mu_est_pred_cons = mu_est_pred-0.15
        return mu_est_pred, mu_est_pred_cons

    def local_available(self,mu_gt):
        # comp Ff_util
        Fxf = self.fssim_carinfo.Fx * (self.state.Fzf / (self.state.Fzf + self.state.Fzr))
        Ff = np.sqrt(Fxf**2+(2*self.fssim_carinfo.Fy_f)**2) # F_yf in msg is mean of F_yf_l and F_yf_r, hence the 2
        Ff_util = Ff/(self.state.Fzf*mu_gt)
        # roll buffer
        self.Ff_util_buffer = np.roll(self.Ff_util_buffer,-1)
        self.Ff_util_buffer[-1] = Ff_util
        # condition
        if any(self.Ff_util_buffer > self.availability_th_local):
            return True
        else:
            return False    


    def nearest_interp(self, xi, x, y):
        idx = np.abs(x - xi[:,None])
        return y[idx.argmin(axis=1)]

    def fast_nearest_interp(self, xi, x, y):
        """Assumes that x is monotonically increasing!!."""
        # Shift x points to centers
        spacing = np.diff(x) / 2
        x = x + np.hstack([spacing, spacing[-1]])
        # Append the last point in y twice for ease of use
        y = np.hstack([y, y[-1]])
        return y[np.searchsorted(x, xi)]

    def het_gp_regression(self, kern, X_train, Y_train, abs_errors_Y):
        m = GPy.models.GPHeteroscedasticRegression(X_train[:,None],Y_train[:,None],kern)
        m['.*het_Gauss.variance'] = abs_errors_Y[:,None] #Set the noise parameters to the error in Y
        mu, var = m._raw_predict(m.X)
        
        return mu, np.sqrt(var)

    def pathLocalToPolArr(self):
        pa = PolygonArray()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = "map"
        for i in range(self.N-1):

            spoly = np.array([self.pathlocal.s[i], self.pathlocal.s[i+1],self.pathlocal.s[i+1],self.pathlocal.s[i]])
            dpoly = np.array([self.pathlocal.dub[i], self.pathlocal.dub[i+1],self.pathlocal.dlb[i+1],self.pathlocal.dlb[i]])
            Xpoly, Ypoly = ptsFrenetToCartesian(spoly,dpoly,self.pathlocal.X,self.pathlocal.Y,self.pathlocal.psi_c,self.pathlocal.s)

            p = PolygonStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "map"
            p.polygon.points = [Point32(x=Xpoly[0], y=Ypoly[0], z=0.0),
                                Point32(x=Xpoly[1], y=Ypoly[1], z=0.0),
                                Point32(x=Xpoly[2], y=Ypoly[2], z=0.0),
                                Point32(x=Xpoly[3], y=Ypoly[3], z=0.0)]
            pa.polygons.append(p)    
        
        # color for mu
        pa.likelihood = self.pathlocal.mu*(0.2/self.maxmu) # discarding final value

        return pa
 
              
    def pathglobal_callback(self, msg):
        self.pathglobal = msg
        
        # get s of one lap
        stot_global = self.pathglobal.s[-1]
        dist_sf = np.sqrt( (self.pathglobal.X[0]-self.pathglobal.X[-1])**2 + (self.pathglobal.Y[0]-self.pathglobal.Y[-1])**2)
        self.s_lap = stot_global + dist_sf  
        
        # start pathrolling as two first laps
        if(not self.received_pathglobal):
            
            self.pathrolling.X = np.concatenate((np.array(self.pathglobal.X),np.array(self.pathglobal.X)),axis=0)
            self.pathrolling.Y = np.concatenate((np.array(self.pathglobal.Y),np.array(self.pathglobal.Y)),axis=0)
            self.pathrolling.s = np.concatenate((np.array(self.pathglobal.s),np.array(self.pathglobal.s) + self.s_lap ),axis=0)
            self.pathrolling.psi_c = np.concatenate((np.array(self.pathglobal.psi_c),np.array(self.pathglobal.psi_c)),axis=0)
            self.pathrolling.theta_c = np.concatenate((np.array(self.pathglobal.theta_c),np.array(self.pathglobal.theta_c)),axis=0)
            self.pathrolling.kappa_c = np.concatenate((np.array(self.pathglobal.kappa_c),np.array(self.pathglobal.kappa_c)),axis=0)
            self.pathrolling.kappaprime_c = np.concatenate((np.array(self.pathglobal.kappaprime_c),np.array(self.pathglobal.kappaprime_c)),axis=0)
            self.pathrolling.dub = np.concatenate((np.array(self.pathglobal.dub),np.array(self.pathglobal.dub)),axis=0)
            self.pathrolling.dlb = np.concatenate((np.array(self.pathglobal.dlb),np.array(self.pathglobal.dlb)),axis=0)
        
        # mu can be updated later by exp manager
        self.pathrolling.mu = np.concatenate((np.array(self.pathglobal.mu),np.array(self.pathglobal.mu)),axis=0)
        
        self.received_pathglobal = True
    
    def state_callback(self, msg):
        self.state = msg
        self.received_state = True

    def carinfo_callback(self, msg):
        self.fssim_carinfo = msg
        self.received_fssim_carinfo = True

if __name__ == '__main__':
    rp = RoadPerception()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
