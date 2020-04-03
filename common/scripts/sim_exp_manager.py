#!/usr/bin/env python

'''
Description: This node
    - Publishes clock, controlling simulation time
    - controls pop-up obstacles
    - controls friction conditions
    - save or plot run data 
    - all other nodes shut down when this shuts down
'''
import os
import time
import copy 
import numpy as np
import rospy
from rosgraph_msgs.msg import Clock
from common.msg import Path
from common.msg import Obstacles
from common.msg import State
from common.msg import Trajectory
from common.msg import SaartiStatus
from fssim_common.msg import TireParams
from fssim_common.msg import CarInfo
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
from coordinate_transforms import ptsFrenetToCartesian
from std_srvs.srv import Empty

class ExperimentManager:
    # constructor
    def __init__(self):
        
        # init node
        rospy.init_node('experiment_manager')
        
        # timing params
        self.dt_sim = 0.01 # timestep of the simulation
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
        
        # vehicle params
        self.robot_name = rospy.get_param('/robot_name') 
        self.vehicle_width = rospy.get_param('/car/kinematics/l_width')    
        
        # algo params (from acado)
        N = 40 
        dt_algo = 0.1
        
        # set up pausing of gazebo
        rospy.wait_for_service('gazebo/pause_physics')
        pause_gazebo = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        rospy.wait_for_service('gazebo/unpause_physics')
        unpause_gazebo = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.s_pause_gazebo = rospy.get_param('/s_pause_gazebo')
        n_pauses = len(self.s_pause_gazebo) 
        i_pauses = 0
        
        # init subs pubs
        #self.clockpub = rospy.Publisher('/clock', Clock, queue_size=10)   
        self.pathglobalsub = rospy.Subscriber("pathglobal", Path, self.pathglobal_callback)
        self.statesub = rospy.Subscriber("state", State, self.state_callback)
        self.carinfosub = rospy.Subscriber("/fssim/car_info", CarInfo, self.fssim_carinfo_callback)
        self.trajstarsub = rospy.Subscriber("trajstar", Trajectory, self.trajstar_callback)
        self.statussub = rospy.Subscriber("saarti_status", SaartiStatus, self.status_callback)
        self.obspub = rospy.Publisher('/obs', Obstacles, queue_size=1)
        self.obsvispub = rospy.Publisher('/obs_vis', Marker, queue_size=1)
        self.tireparampub = rospy.Publisher('/tire_params', TireParams, queue_size=1)
        self.ctrl_mode_pub = rospy.Publisher('/ctrl_mode', Int16, queue_size=1)
        self.statetextmarkerpub = rospy.Publisher('/state_text_marker', Marker, queue_size=1)        

        # init logging vars
        self.s_begin_log = rospy.get_param('/s_begin_log')
        self.N_iters_to_save = 60
        self.explog_iterationcounter = 0
        self.explog_activated = False
        self.stored_pathglobal = False
        self.stored_trajstar = False
        self.stored_trajcl = False
        self.stored_saartistatus = False
        self.explog_saved = False
        
        # init misc internal variables
        self.pathglobal = Path()
        self.received_pathglobal = False
        self.state = State()
        self.received_state = False
        self.saarti_status = SaartiStatus()
        self.received_saartistatus = False
        self.trajstar = Trajectory()
        self.received_trajstar = False
        self.fssim_carinfo = CarInfo()
        
        while(not self.received_pathglobal):
            print("waiting for pathglobal")
            time.sleep(self.dt_sim*100)

        # init experiment variables
        self.scenario_id = rospy.get_param('/scenario_id')
        self.traction_adaptive  = rospy.get_param('/traction_adaptive')
        
        self.tireparams = TireParams()
        self.obs = Obstacles()
        self.obs.s = [self.s_obs_at_popup]
        self.obs.d = [self.d_obs_at_popup]
        self.obs.R = [0.5]
        wiggleroom = 1.0 # todo param
        self.obs.Rmgn = [0.5*self.obs.R[0] + 0.5*self.vehicle_width + wiggleroom]
        Xobs, Yobs = ptsFrenetToCartesian(np.array(self.obs.s), \
                                          np.array(self.obs.d), \
                                          np.array(self.pathglobal.X), \
                                          np.array(self.pathglobal.Y), \
                                          np.array(self.pathglobal.psi_c), \
                                          np.array(self.pathglobal.s))
        self.ctrl_mode = 0 # # 0: stop, 1: cruise_ctrl, 2: tamp 
        
        # Main loop
        self.exptime = 0 
        while (not rospy.is_shutdown()) and self.exptime<self.t_final :
            if (self.exptime >= self.t_activate):        
                rospy.loginfo_throttle(1, "Running experiment, ctrl mode = %i"%self.ctrl_mode)
                
                # HANDLE TRACTION IN SIMULATION                
                s_ego = self.state.s % self.s_lap                
                for i in range(self.N_mu_segments-1):
                    if(self.s_begin_mu_segments[i] <= s_ego <= self.s_begin_mu_segments[i+1]):
                        self.mu_segment_idx = i
                        break
                if(s_ego >= self.s_begin_mu_segments[-1]):
                    self.mu_segment_idx = self.N_mu_segments-1
                mu = self.mu_segment_values[self.mu_segment_idx] 
                
                # set tire params of sim vehicle
                self.tireparams.B, self.tireparams.C, self.tireparams.D, self.tireparams.E = self.get_tire_params(mu)
                self.tireparams.D = - self.tireparams.D # fssim sign convention             
                self.tireparams.tire_coefficient = 1.0        
                self.tireparams.header.stamp = rospy.Time.now()
                self.tireparampub.publish(self.tireparams)
                
                # POPUP SCENARIO
                if (self.scenario_id in [1,4] ):
                    self.ctrl_mode = 1 # cruise control
                    if(self.state.vx > 5):
                        self.ctrl_mode = 2 # tamp
                    m_obs = self.getobstaclemarker(Xobs,Yobs,self.obs.R[0])
                    m_obs.color.a = 0.3 # transparent before detect
                    if (self.state.s >= self.s_ego_at_popup):
                        self.obspub.publish(self.obs)
                        m_obs.color.a = 1.0 # non-transparent after detect
                        
                    self.obsvispub.publish(m_obs)
                
                # REDUCED MU TURN
                elif(self.scenario_id == 2):
                    self.ctrl_mode = 1 # pp cruise control from standstill
                    if(self.state.vx > 5): # todo
                        self.ctrl_mode = 2 # tamp cruise control when we get up to speed
                
                # PURE PURSUIT TEST
                elif(self.scenario_id == 5):
                    self.ctrl_mode = 1

                # TAMP TEST
                elif(self.scenario_id == 6):
                    self.ctrl_mode = 2
                                       
                # RACING
                else:
                    self.ctrl_mode = 2 # tamp
                
                # SEND STOP IF EXIT TRACK
                dlb = np.interp(self.state.s,self.pathglobal.s,self.pathglobal.dlb)
                dub = np.interp(self.state.s,self.pathglobal.s,self.pathglobal.dub)
                if (self.state.d < dlb-1.0 or self.state.d > dub+1.0): # todo get from param
                    self.ctrl_mode = 0 # stop
                self.ctrl_mode_pub.publish(self.ctrl_mode)
                
                # publish text marker (state info)
                if(self.traction_adaptive):
                    traction_adaptive_str = "on"
                else:
                    traction_adaptive_str = "off"
                    
                state_text = "traction_adapt: " + traction_adaptive_str + "\n"  \
                             "vx: " + "%.3f" % self.state.vx + "\n"  \
                             "mu: " + "%.3f" % mu            
                self.statetextmarkerpub.publish(self.gettextmarker(state_text))    
                
                # handle pausing of gazebo
                if (i_pauses < n_pauses):
                    if (self.state.s >= self.s_pause_gazebo[i_pauses]):
                        pause_gazebo()
                        raw_input("Press Enter to continue...")
                        unpause_gazebo()
                        i_pauses += 1
                
                # save data for plot
                if (self.state.s >= self.s_begin_log and not self.explog_activated):
                    print "STARTED EXPLOG"
                    t_start_explog = copy.deepcopy(self.exptime) 
                    self.explog_activated = True
                    # store planned traj
                    self.trajstar_dict = {
                      "X": np.array(self.trajstar.X),
                      "Y": np.array(self.trajstar.Y),
                      "psi": np.array(self.trajstar.psi),
                      "s": np.array(self.trajstar.s),
                      "d": np.array(self.trajstar.d),
                      "deltapsi": np.array(self.trajstar.deltapsi),
                      "psidot": np.array(self.trajstar.psidot),
                      "vx": np.array(self.trajstar.vx),
                      "vy": np.array(self.trajstar.vy),
                      "ax": np.array(self.trajstar.ax),
                      "ay": np.array(self.trajstar.ay),
                      "Fyf": np.array(self.trajstar.Fyf),
                      "Fxf": np.array(self.trajstar.Fxf),
                      "Fyr": np.array(self.trajstar.Fyr),
                      "Fxr": np.array(self.trajstar.Fxr),
                      "Fzf": np.array(self.trajstar.Fzf),
                      "Fzr": np.array(self.trajstar.Fzr),
                      "kappac": np.array(self.trajstar.kappac),
                      "Cr": np.array(self.trajstar.Cr),
                      "t": np.arange(0,(N+1)*dt_algo,dt_algo),
                    }
                    self.stored_trajstar = True
                    
                    # initialize vars for storing CL traj
                    self.X_cl = []
                    self.Y_cl = []
                    self.psi_cl = []
                    self.s_cl = []
                    self.d_cl = []
                    self.deltapsi_cl = []
                    self.psidot_cl = []
                    self.vx_cl = []
                    self.vy_cl = []
                    self.ax_cl = []
                    self.ay_cl = []
                    self.t_cl = []
                    self.Fyf_cl = []
                    self.Fyr_cl = []
                    self.Fx_cl = []
                    
                    # and saarti status
                    self.rollout_selected = []
                    self.n_coll_free = []
                    self.total_planning_time = []
                    self.rollout_time = []
                    self.optimization_time = []
                    
                if (self.state.s >= self.s_begin_log and self.explog_activated and self.exptime >= t_start_explog + self.explog_iterationcounter*dt_algo):
                    # build CL traj       
                    self.X_cl.append(self.state.X)
                    self.Y_cl.append(self.state.Y)
                    self.psi_cl.append(self.state.psi)
                    self.s_cl.append(self.state.s)
                    self.d_cl.append(self.state.d)
                    self.deltapsi_cl.append(self.state.deltapsi)
                    self.psidot_cl.append(self.state.psidot)
                    self.vx_cl.append(self.state.vx)
                    self.vy_cl.append(self.state.vy)
                    self.ax_cl.append(self.state.ax)
                    self.ay_cl.append(self.state.ay)
                    self.t_cl.append(self.exptime)
                    self.Fyf_cl.append(self.fssim_carinfo.Fy_f_l+self.fssim_carinfo.Fy_f_r)
                    self.Fyr_cl.append(self.fssim_carinfo.Fy_r)
                    self.Fx_cl.append(self.fssim_carinfo.Fx)

                    # and saarti status
                    self.rollout_selected.append(self.saarti_status.rollout_selected)
                    self.n_coll_free.append(self.saarti_status.nr_of_collision_free_candidates)
                    self.total_planning_time.append(self.saarti_status.total_planning_time)
                    self.rollout_time.append(self.saarti_status.rollout_time)
                    self.optimization_time.append(self.saarti_status.optimization_time)
                    
                    # store CL traj as dict
                    if (self.explog_iterationcounter == N): # store N+1 values, same as trajstar
                        self.trajcl_dict = {
                            "X": np.array(self.X_cl),
                            "Y": np.array(self.Y_cl),
                            "psi": np.array(self.psi_cl),
                            "s": np.array(self.s_cl),
                            "d": np.array(self.d_cl),
                            "deltapsi": np.array(self.deltapsi_cl),
                            "psidot": np.array(self.psidot_cl),
                            "vx": np.array(self.vx_cl),
                            "vy": np.array(self.vy_cl),
                            "ax": np.array(self.ax_cl),
                            "ay": np.array(self.ay_cl),
                            "t": np.array(self.t_cl),
                            "Fyf": np.array(self.Fyf_cl),
                            "Fyr": np.array(self.Fyr_cl),
                            "Fx": np.array(self.Fx_cl),
                        }
                        self.stored_trajcl = True
                        
                        self.sarti_status_dict = {
                            "rollout_selected": np.array(self.rollout_selected),
                            "n_coll_free": np.array(self.n_coll_free),
                            "total_planning_time": np.array(self.total_planning_time),
                            "rollout_time": np.array(self.rollout_time),
                            "optimization_time": np.array(self.optimization_time),
                        }
                        self.stored_saartistatus = True
                    
                    self.explog_iterationcounter +=1
                
                # save explog               
                if (self.stored_pathglobal and self.stored_trajstar and self.stored_trajcl and self.stored_saartistatus and not self.explog_saved):  
                    explog = {
                      "pathglobal": self.pathglobal_dict,
                      "trajstar": self.trajstar_dict,
                      "trajcl": self.trajcl_dict,
                      "saartistatus": self.sarti_status_dict,
                    }
                    
                    filepath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../../../', 'tamp_logs/logs/data_latest'))
                    filename = "/explog"
                    if(self.scenario_id == 1):
                        filename = filename + "_popup"
                        if(np.min(self.mu_segment_values)>0.8):
                            filename = filename + "_dry"
                        else:
                            filename = filename + "_wet"
                    elif(self.scenario_id == 2):
                        filename = filename + "_reducedmuturn"
                    elif(self.scenario_id == 3):
                        filename = filename + "_racing"
                    elif(self.scenario_id == 4):
                        filename = filename + "_localmin"
                    # separate adaptive (SA) and nonadaptive (RTI)
                    if(self.traction_adaptive):
                        filename = filename + "_adaptive"
                    else:
                        filename = filename + "_nonadaptive"
                    filename = filename + ".npy"
                    
                    np.save(filepath+filename,explog)
                    self.explog_saved = True
                    print("SAVED EXPLOG to " + filepath + filename)
            
            else: # not reached activation time
                rospy.loginfo_throttle(1, "Experiment starting in %i seconds"%(self.t_activate-self.exptime))
            
            # handle exptime
            self.exptime += self.dt_sim
            msg = Clock()
            t_rostime = rospy.Time(self.exptime)
            msg.clock = t_rostime
            #self.clockpub.publish(msg)
                                     
            
            time.sleep(self.dt_sim)

        print 'simulation finished'
    
        # send shutdown signal
        message = 'run finished, shutting down'
        print message
        rospy.signal_shutdown(message)

    def get_tire_params(self,mu):
        if (0.0 <= mu <0.3): # ice
            B = 4.0
            C = 2.0
            D = mu
            E = 1.0
        elif (0.3 <= mu < 0.5): # snow
            B = 5.0
            C = 2.0
            D = mu
            E = 1.0
        elif (0.5 <= mu < 0.9): # wet
            B = 12.0
            C = 2.3
            D = mu
            E = 1.0
        elif (0.9 <= mu < 1.5): # dry
            B = 10.0
            C = 1.9
            D = mu
            E = 0.97
        elif (1.5 <= mu < 2.5): # dry + racing tires (gotthard default)
            B = 12.56;
            C = 1.38; 
            D = mu;
            E = 1.0               
        else: 
            rospy.logerr("Faulty mu value in exp manager")
        
        return B,C,D,E

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

    def gettextmarker(self,text):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "base_link"
        m.pose.position.x = 0;
        m.pose.position.y = 7.5;
        m.pose.position.z = 5.0;
        m.type = m.TEXT_VIEW_FACING;
        m.text = text
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        m.color.a = 1.0; 
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        return m

    def pathglobal_callback(self, msg):
        self.pathglobal = msg
        
        # get s of one lap
        stot_global = self.pathglobal.s[-1]
        dist_sf = np.sqrt( (self.pathglobal.X[0]-self.pathglobal.X[-1])**2 + (self.pathglobal.Y[0]-self.pathglobal.Y[-1])**2)
        self.s_lap = stot_global + dist_sf  
  
        # put on dictionary format for explog
        self.pathglobal_dict = {
          "X": self.pathglobal.X,
          "Y": self.pathglobal.Y,
          "s": self.pathglobal.s,
          "psi_c": self.pathglobal.psi_c,
          "theta_c": self.pathglobal.theta_c,
          "kappa_c": self.pathglobal.kappa_c,
          "kappaprime_c": self.pathglobal.kappaprime_c,
          "mu": self.pathglobal.mu,
          "dub": self.pathglobal.dub,
          "dlb": self.pathglobal.dlb,
          "s_lap": self.s_lap,
        }
        self.stored_pathglobal = True
        self.received_pathglobal = True
        
    def state_callback(self, msg):
        self.state = msg
        self.received_state = True
    
    def status_callback(self, msg):
        self.saarti_status = msg
        self.received_saartistatus = True

    def fssim_carinfo_callback(self,msg):
        self.fssim_carinfo = msg

    def trajstar_callback(self, msg):
        self.trajstar = msg
        self.received_trajstar = True
    
    
if __name__ == '__main__':
    em = ExperimentManager()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
    
    
