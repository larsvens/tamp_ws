#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Todo 
- put in notebook with picture for documentation (and paper)
- overlay on asta track import
- export gps coordinates
- export cone positions in asta coordinate system
@author: larsvens
"""

import os 
import numpy as np
import matplotlib.pyplot as plt
from coordinate_transforms import ptsFrenetToCartesian

from lxml import etree # fos .sdf generation
import yaml

# adjust for high dpi screen
plt.rcParams['figure.dpi'] = 100 # default 100
plt.rcParams['figure.figsize'] = 15, 5

# params
la = 200 # back of L shape
lc = 70  # base of L shape
Rb  = 22  # radius of curve at experiment turn
R = 30   # radius at other turns 
lanewidth = 3.5

# additional track info
cones_orange_X = np.array([])
cones_orange_Y = np.array([])    
cones_orange_big_X = np.array([4.7, 4.7, 7.3, 7.3])
cones_orange_big_Y = np.array([4.0, -4.0, 4.0, -4.0])
tk_device_X = np.array([6.0, 6.0])
tk_device_Y = np.array([4.5, -4.5])
starting_pose_front_wing = np.array([0.01, 0.0, 0.0]) # small adjust to avoid negative s

# section a 
Na = la # default 1 node per meter
Xa = np.linspace(0,la,Na)
Ya = np.zeros(Na)

# section b
Nb = np.ceil(2*Rb*np.pi/4.0)
t = np.linspace(np.pi/2,0,Nb)
Xb = Rb*np.cos(t) + Xa[-1]
Yb = Rb*np.sin(t) - Rb + Ya[-1]

# section c
Nc = lc
Xc = np.zeros(Nc) + Xb[-1]
Yc = np.linspace(Yb[-1],Yb[-1]-lc,Nc)

# section d
Nd = np.ceil(R*np.pi) 
t = np.linspace(0,-np.pi,Nd)
Xd = R*np.cos(t) + Xc[-1] - R
Yd = R*np.sin(t) + Yc[-1]

# section e
le = lc + Rb -3*R
Ne = le
Xe = np.zeros(Ne) + Xd[-1]
Ye = np.linspace(Yd[-1], Yd[-1]+le,Ne)

# section f
Nf = np.ceil(2*R*np.pi/4.0)
t = np.linspace(0,np.pi/2.0,Nf)
Xf = R*np.cos(t) + Xe[-1] - R
Yf = R*np.sin(t) + Ye[-1]

# section g
lg = la+Rb-3*R
Ng = lg
Xg = np.linspace(Xf[-1],Xf[-1]-lg,Ng)
Yg = np.zeros(Ng)+Yf[-1]

# section h
Nh = np.ceil(R*np.pi) 
t = np.linspace(-np.pi/2,-3*np.pi/2,Nh)
Xh = R*np.cos(t) + Xg[-1]
Yh = R*np.sin(t) + Yg[-1]+R

# concatenate vectors
X_cl_tmp = np.concatenate((Xa, Xb, Xc, Xd, Xe, Xf, Xg, Xh), axis=0)
Y_cl_tmp = np.concatenate((Ya, Yb, Yc, Yd, Ye, Yf, Yg, Yh), axis=0)

# remove duplicate points
threshold_dist = 0.1
X_cl = []
Y_cl = []
for i in range(X_cl_tmp.size-1):
    dist = np.sqrt((X_cl_tmp[i+1]-X_cl_tmp[i])**2 + (Y_cl_tmp[i+1]-Y_cl_tmp[i])**2)
    if (dist > threshold_dist):
        X_cl.append(X_cl_tmp[i])
        Y_cl.append(Y_cl_tmp[i])        
X_cl = np.array(X_cl)
Y_cl = np.array(Y_cl)

# compute s
dX = np.diff(X_cl)
dY = np.diff(Y_cl)
ds = np.sqrt(dX**2+dY**2)
ds_final = np.sqrt((X_cl[0]-X_cl[-1])**2 + (Y_cl[0]-Y_cl[-1])**2)
ds = np.append(ds,ds_final)
s_tmp = np.cumsum(ds)
s_tmp = s_tmp - s_tmp[0]

# resample with equidistant points
ds = 1.0 # step size 
s = np.arange(0,s_tmp[-1],)
X_cl = np.interp(s,s_tmp,X_cl)
Y_cl = np.interp(s,s_tmp,Y_cl)

# compute psic
dX = np.diff(X_cl)
dY = np.diff(Y_cl)
psic = np.arctan2(dY,dX)
psic_final = np.arctan2(Y_cl[0]-Y_cl[-1],X_cl[0]-X_cl[-1])  # assuming closed track
psic = np.append(psic,psic_final) 

## get left and right boundaries
#dlb = (-lanewidth/2.0)*np.ones(s.size)
#dub = (3.0*lanewidth/2.0)*np.ones(s.size)

dlb = -lanewidth*np.ones(s.size)
dub =  lanewidth*np.ones(s.size)

X_ll,Y_ll = ptsFrenetToCartesian(s,dub,X_cl,Y_cl,psic,s) 
X_rl,Y_rl = ptsFrenetToCartesian(s,dlb,X_cl,Y_cl,psic,s)

# downsample to get cone positions
threshold_dist_cones = 4
cones_left_X = [X_ll[0]]
cones_left_Y = [Y_ll[0]]
for i in range(X_ll.size-1):
    dist = np.sqrt((X_ll[i]-cones_left_X[-1])**2 + (Y_ll[i]-cones_left_Y[-1])**2)
    if(dist > threshold_dist_cones):
        cones_left_X.append(X_ll[i])
        cones_left_Y.append(Y_ll[i])
cones_left_X = np.array(cones_left_X)
cones_left_Y = np.array(cones_left_Y)

cones_right_X = [X_rl[0]]
cones_right_Y = [Y_rl[0]]
for i in range(X_rl.size-1):
    dist = np.sqrt((X_rl[i]-cones_right_X[-1])**2 + (Y_rl[i]-cones_right_Y[-1])**2)
    if(dist > threshold_dist_cones):
        cones_right_X.append(X_rl[i])
        cones_right_Y.append(Y_rl[i])
cones_right_X = np.array(cones_right_X)
cones_right_Y = np.array(cones_right_Y)


# plot things
#plt.plot(X_cl_tmp,Y_cl_tmp,'.')

plt.plot(X_cl,Y_cl,'.')

# lane limits
plt.plot(X_ll,Y_ll,'-k')
plt.plot(X_rl,Y_rl,'-k')

# cone positions
plt.plot(cones_left_X,cones_left_Y,'*r')
plt.plot(cones_right_X,cones_right_Y,'*r')

# start
plt.plot(tk_device_X,tk_device_Y,'b*')
plt.plot(cones_orange_big_X,cones_orange_big_Y,'y*')

plt.gca().set_aspect('equal', adjustable='box')
plt.show()

#print "cones_left:"
#for i in range(cones_left_X.size):
#    print "- - ", cones_left_X[i]
#    print "  - ", cones_left_Y[i]
#
#print "cones_right:"
#for i in range(cones_right_X.size):
#    print "- - ", cones_right_X[i]
#    print "  - ", cones_right_Y[i]






### EXPORT TRACK
name = "asta_zero"
#export_path = os.path.dirname(os.path.realpath(__file__))
export_path = "/home/larsvens/ros/tamp__ws/src/fssim/fssim_gazebo/models/track"

#### EXPORT AS YAML 
cones_left = np.column_stack((cones_left_X,cones_left_Y))
cones_right = np.column_stack((cones_right_X,cones_right_Y))
cones_orange = np.column_stack((cones_orange_X,cones_orange_Y))
cones_orange_big = np.column_stack((cones_orange_big_X,cones_orange_big_Y))
tk_device = np.column_stack((tk_device_X,tk_device_Y))

dict_track = {"cones_left": np.array(cones_left).tolist(),
              "cones_right": np.array(cones_right).tolist(),
              "cones_orange": np.array(cones_orange).tolist(),
              "cones_orange_big": np.array(cones_orange_big).tolist(),
              "tk_device": np.array(tk_device).tolist(),
#              "middle_points":np.array(self.middle).tolist(),
              "starting_pose_front_wing": starting_pose_front_wing.tolist()}


file_path = export_path + '/tracks_yaml/' + name + ".yaml"
with open(file_path, 'w') as outfile:
    yaml.dump(dict_track, outfile, default_flow_style = False)
print "[INFO] Saving track to: ",file_path


### EXPORT AS SDF

#def export_model(self, path, name):
#root = etree.Element("model")
#etree.SubElement(root, "name").text = "track"
#etree.SubElement(root, "version").text = "1.0"
#etree.SubElement(root, "sdf", version="1.4").text = name + ".sdf"
#etree.SubElement(root, "description").text = "random track"
#tree = etree.ElementTree(root)
#tree.write(self.model_path + "/track/model.config", pretty_print=True, xml_declaration=True, encoding='UTF-8')

root = etree.Element("sdf", version="1.4")
model = etree.SubElement(root, "model", name="some track")

for i in range(0, cones_right_X.size):
    include = etree.SubElement(model, "include")
    etree.SubElement(include, "uri").text = "model://fssim_gazebo/models/cone_blue"
    etree.SubElement(include, "pose").text = str(cones_right_X[i]) + " " + str(cones_right_Y[i]) + " 0 0 0 0"
    etree.SubElement(include, "name").text = "cone_right"

for i in range(0, cones_left_X.size):
    include = etree.SubElement(model, "include")
    etree.SubElement(include, "uri").text = "model://fssim_gazebo/models/cone_yellow"
    etree.SubElement(include, "pose").text = str(cones_left_X[i]) + " " + str(cones_left_Y[i]) + " 0 0 0 0"
    etree.SubElement(include, "name").text = "cone_left"

for i in range(0, cones_orange_X.size):
    include = etree.SubElement(model, "include")
    etree.SubElement(include, "uri").text = "model://fssim_gazebo/models/cone_orange"
    etree.SubElement(include, "pose").text = ""
    etree.SubElement(include, "name").text = "cone_orange"

for i in range(0, cones_orange_big_X.size):
    include = etree.SubElement(model, "include")
    etree.SubElement(include, "uri").text = "model://fssim_gazebo/models/cone_orange_big"
    etree.SubElement(include, "pose").text = str(cones_orange_big_X[i]) + " " + str(cones_orange_big_Y[i]) + " 0 0 0 0"
    etree.SubElement(include, "name").text = "cone_orange_big"

for i in range(0, tk_device_X.size):
    include = etree.SubElement(model, "include")
    etree.SubElement(include, "uri").text = "model://fssim_gazebo/models/time_keeping"
    etree.SubElement(include, "pose").text = str(tk_device_X[i]) + " " + str(tk_device_Y[i]) + " 0 0 0 0"
    etree.SubElement(include, "name").text = "tk_device_" + str(i)

tree = etree.ElementTree(root)
#gazebo_models = self.model_path + "/track/" + name
#tree.write(gazebo_models + ".sdf", pretty_print=True, xml_declaration=True, encoding='UTF-8')

file_path = export_path + '/' + name + ".sdf"
tree.write(file_path, pretty_print=True, xml_declaration=True, encoding='UTF-8')
#self.track.export_to_yaml(self.model_path + "/track/tracks_yaml", name,create_dir=False)

print "[INFO] Saving track to: ",export_path + name + ".sdf"
#print "[INFO] Saving track to: ",gazebo_models + ".sdf"









