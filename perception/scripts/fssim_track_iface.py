#!/usr/bin/env python

# Descrition: 
# Subscribes to fssim topics
# Publishes state, local path and dynamic params

import numpy as np
from scipy import interpolate
import rospy
import tf
from nav_msgs.msg import Path as navPath
from geometry_msgs.msg import PoseStamped
from fssim_common.msg import Track
from common.msg import Path
from coordinate_transforms import ptsFrenetToCartesian

import matplotlib.pyplot as plt
# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 10, 10

class TrackInterface:
    def __init__(self):
        rospy.init_node('track_interface', anonymous=True)
        self.pathglobalpub = rospy.Publisher('pathglobal', Path, queue_size=10)
        self.pathglobalvispub = rospy.Publisher('pathglobal_vis', navPath, queue_size=10)
        self.dubvispub = rospy.Publisher('dubglobal_vis', navPath, queue_size=10)
        self.dlbvispub = rospy.Publisher('dlbglobal_vis', navPath, queue_size=10)
        self.track_sub = rospy.Subscriber("/fssim/track", Track, self.track_callback)
        self.track = Track()
        self.pathglobal = Path()
        self.received_track = False
        self.rate = rospy.Rate(1)
        
        # set params
        ds = 0.5 # step size in s
        plot_track = False
        plot_orientation = False
        
        # wait for track
        while(not self.received_track):
            print "waiting for track"
            self.rate.sleep()      

        # get cone positions left and right as numpy arrays
        cl_X = []
        cl_Y = []
        for i in range(len(self.track.cones_left)):
            cl_X.append(self.track.cones_left[i].x)
            cl_Y.append(self.track.cones_left[i].y)            
        cl_X = np.array(cl_X)
        cl_Y = np.array(cl_Y)
        print "nr of cones left: ", cl_X.size
        
        cr_X = []
        cr_Y = []
        for i in range(len(self.track.cones_right)):    
            cr_X.append(self.track.cones_right[i].x)
            cr_Y.append(self.track.cones_right[i].y) 
        cr_X = np.array(cr_X)
        cr_Y = np.array(cr_Y)
        print "nr of cones right: ", cr_X.size

        # compute rough centerline
        ccl_X = []
        ccl_Y = []
        for i in range(cl_X.size):
            pt_left = {"X": cl_X[i], "Y": cl_Y[i]}
            # find closest right cone
            dist = np.sqrt((cr_X-pt_left["X"])**2 + (cr_Y-pt_left["Y"])**2)
            idx = np.argmin(dist)
            pt_right = {"X": cr_X[idx], "Y": cr_Y[idx]}
            pt_mid = {"X": 0.5*(pt_left["X"]+pt_right["X"]), "Y": 0.5*(pt_left["Y"]+pt_right["Y"])}
            ccl_X.append(pt_mid["X"])
            ccl_Y.append(pt_mid["Y"])
        ccl_X = np.array(ccl_X)
        ccl_Y = np.array(ccl_Y)
        
        # get approximate length of track
        stot = 0
        for i in range(ccl_X.size-1):
            stot += np.sqrt((ccl_X[i+1]-ccl_X[i])**2 + (ccl_Y[i+1]-ccl_Y[i])**2)
        stot = (stot//ds)*ds
        print "length of track: stot = ", str(stot)
        
        # set s
        s = np.arange(0, stot, ds)
        
        # parametric spline interpolation
        print "spline interpolation of centerline"
        N = int(stot/ds)
        unew = np.arange(0, 1.0, 1.0/N) # N equidistant pts
        # center
        tck, u = interpolate.splprep([ccl_X, ccl_Y], s=0)
        out = interpolate.splev(unew, tck)
        fcl_X = out[0]
        fcl_Y = out[1]
        # left
        tck, u = interpolate.splprep([cl_X, cl_Y], s=0)
        out = interpolate.splev(unew, tck)
        fll_X = out[0]
        fll_Y = out[1]        
        # right
        tck, u = interpolate.splprep([cr_X, cr_Y], s=0)
        out = interpolate.splev(unew, tck)
        frl_X = out[0]
        frl_Y = out[1]

        # set psic
        print "computing psic"        
        dX = np.diff(fcl_X)
        dY = np.diff(fcl_Y)

        psic = np.arctan2(dY,dX)
        psic_final = np.arctan2(fcl_Y[0]-fcl_Y[-1],fcl_X[0]-fcl_X[-1])  
        psic = np.append(psic,psic_final) # assuming closed track

        # separate in pieces (for 2pi flips)
        idx_low = 0
        idx_high = 0
        psic_piecewise = []
        s_piecewise = []
        for i in range(psic.size-1):
            if(np.abs(psic[i+1] - psic[i]) > np.pi ): 
                # if single pt, remove
                if(np.abs(psic[i+2] - psic[i]) < np.pi ):
                    print("removing single flip")
                    psic[i+1] = psic[i]
                # otherwise make a piece
                else:
                    print("making pieces (psic flips)")
                    idx_high = i+1
                    psic_piece = psic[idx_low:idx_high]
                    psic_piecewise.append(psic_piece)
                    s_piece = s[idx_low:idx_high]
                    s_piecewise.append(s_piece)
                    idx_low = i+1
                                  
        # add final piece
        psic_piece = psic[idx_low:psic.size]
        psic_piecewise.append(psic_piece)
        s_piece = s[idx_low:psic.size]
        s_piecewise.append(s_piece)
        
        # shift pieces to make continous psi_c
        for j in range(len(psic_piecewise)-1):
            while(psic_piecewise[j][-1] - psic_piecewise[j+1][0] > np.pi):
                psic_piecewise[j+1] = psic_piecewise[j+1] + 2*np.pi
                
            while(psic_piecewise[j][-1] - psic_piecewise[j+1][0] < -np.pi):
                psic_piecewise[j+1] = psic_piecewise[j+1] - 2*np.pi
        
        # concatenate pieces
        psic_cont = psic_piecewise[0]
        for j in range(len(psic_piecewise)-1):
            psic_cont = np.concatenate((psic_cont,psic_piecewise[j+1]))        
        
        # downsample for smoother curve
        step = 11
        print "interpolating downsampled psic with step size ", str(step)
        s_ds = s[0::step]
        psic_ds = psic_cont[0::step]
        
        # interpolate 
        t, c, k = interpolate.splrep(s_ds, psic_ds, s=0, k=4)
        psic_spl = interpolate.BSpline(t, c, k, extrapolate=True)
        
        # compute derrivatives (compare with naive numerical) 
        print "computing derivatives of psic"
        kappac_spl = psic_spl.derivative(nu=1)
        kappacprime_spl = psic_spl.derivative(nu=2)
        
        # put psi_c back on interval [-pi,pi]
        psic_out = psic_spl(s)
        for i in range(psic_out.size):
            while(psic_out[i] > np.pi):
                psic_out[i] = psic_out[i] -2*np.pi
            while(psic_out[i] < -np.pi):
                psic_out[i] = psic_out[i] +2*np.pi

        # set kappac        
        kappac_out = kappac_spl(s)
        kappacprime_out = kappacprime_spl(s)

        # set dlb and dub
        print "setting dlb and dub"
        dub = []
        dlb = []
        for i in range(N):
            X = fcl_X[i]
            Y = fcl_Y[i]
            # left
            dub.append(np.amin(np.sqrt((fll_X-X)**2 + (fll_Y-Y)**2)))
            # right
            dlb.append(-np.amin(np.sqrt((frl_X-X)**2 + (frl_Y-Y)**2)))
        dub = np.array(dub)
        dlb = np.array(dlb)      
        
        # plot to see what we're doing  
        if plot_orientation:   
            fig, axs = plt.subplots(2,1)
            axs[0].plot(s,psic,'k*')
            #for j in range(len(psic_piecewise)):
            #    ax.plot(s_piecewise[j],psic_piecewise[j],'.')
            #ax.plot(s,psic_cont,'r')        
            axs[0].plot(s,psic_out,'m.')
            axs[1].plot(s,kappac_out,'.m')
            plt.show()
        
        if plot_track:
            fig, ax = plt.subplots()
            ax.axis("equal")
            ax.plot(fcl_X,fcl_Y, '.b') # fine centerline
            ax.plot(fll_X,fll_Y, '.b') # fine left line
            ax.plot(frl_X,frl_Y, '.b') # fine right line
            ax.plot(cl_X,cl_Y, '*k') # cones left
            ax.plot(cr_X,cr_Y, '*k') # cones right 
            ax.plot(ccl_X,ccl_Y, '*r') # coarse centerline
            plt.show()
        
        # put all in message and publish
        self.pathglobal.X = fcl_X
        self.pathglobal.Y = fcl_Y
        self.pathglobal.s = s
        self.pathglobal.psi_c = psic_out
        self.pathglobal.kappa_c = kappac_out
        self.pathglobal.kappaprime_c = kappacprime_out
        self.pathglobal.theta_c = np.zeros(N) # grade/bank implement later
        self.pathglobal.dub = dub
        self.pathglobal.dlb = dlb
        print "publishing pathglobal"
        self.pathglobalpub.publish(self.pathglobal)

        # publish paths for rviz
        pathglobalvis = navPath()
        for i in range(N):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = fcl_X[i]
            pose.pose.position.y = fcl_Y[i]            
            quaternion = tf.transformations.quaternion_from_euler(0, 0, psic[i])
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            pathglobalvis.poses.append(pose)
        pathglobalvis.header.stamp = rospy.Time.now()
        pathglobalvis.header.frame_id = "map"
        print "publishing pathglobal visualization"
        self.pathglobalvispub.publish(pathglobalvis)

        # test correctness of dub and dlb in rviz
        Xleft,Yleft = ptsFrenetToCartesian(s,dub,fcl_X,fcl_Y,psic,s)
        pathleft = navPath()
        pathleft.header.stamp = rospy.Time.now()
        pathleft.header.frame_id = "map"
        for i in range(N):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = Xleft[i]
            pose.pose.position.y = Yleft[i]            
            pathleft.poses.append(pose)
        self.dubvispub.publish(pathleft)
        
        Xright,Yright = ptsFrenetToCartesian(s,dlb,fcl_X,fcl_Y,psic,s)
        pathright = navPath()        
        pathright.header.stamp = rospy.Time.now()
        pathright.header.frame_id = "map"
        for i in range(N):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = Xright[i]
            pose.pose.position.y = Yright[i]            
            pathright.poses.append(pose)
        self.dlbvispub.publish(pathright)    

    def track_callback(self, msg):
        self.track = msg
        self.received_track = True

if __name__ == '__main__':
    ti = TrackInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")   
        
        