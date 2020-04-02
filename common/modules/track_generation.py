#!/usr/bin/env python
"""
Description: 
    Standalone script for generating evaluation tracks
    Saves track in 3 formats:
        .yaml
        .sdf - for gazebo simulation
        .kml - for vizualization in google earth
Usage:
    Specify params
        track_name
        utm start pose:  X0_utm, Y0_utm, psi0_utm
        track params: la, Rb etc... 
    Run and examine
"""
import os
from os import path
import utm
import simplekml
import numpy as np
import matplotlib.pyplot as plt
from lxml import etree 
import yaml
from coordinate_transforms import ptsFrenetToCartesian
from util import angleToContinous
#from util import angleToInterval
import re
from scipy import interpolate

def get_cl_from_kml(filepath):    
    tree = etree.parse(filepath)    
    lineStrings = tree.findall('.//{http://www.opengis.net/kml/2.2}LineString')    
    for attributes in lineStrings:
        for subAttribute in attributes:
            if subAttribute.tag == '{http://www.opengis.net/kml/2.2}coordinates':
                coords = subAttribute.text
    
    # clean and cast
    coords = re.split(',| ',coords)
    coords[0] = coords[0].translate(None, "\n\t\t\t\t")
    coords = coords[:-1]
    
    # split in lon lat and alt
    if(len(coords) % 3 != 0):
        print "Error: len(coords) not divisible by three"
    lat = []
    lon = []
    alt = []
    for i in range(len(coords)/3):
        lon.append(float(coords[3*i]))
        lat.append(float(coords[3*i+1]))
        alt.append(float(coords[3*i+2]))
    lat = np.array(lat)
    lon = np.array(lon)
    alt = np.array(alt)
    
    # convert to utm
    X_utm,Y_utm,utm_nr,utm_letter = utm.from_latlon(lat, lon)
    
    # interpolate 
    ds = 1.0 # want approx 1 m between pts
    # approximate stot
    stot = 0
    for i in range(X_utm.size-1):
        stot += np.sqrt((X_utm[i+1]-X_utm[i])**2 + (Y_utm[i+1]-Y_utm[i])**2)
    N = int(stot/ds)
    unew = np.arange(0, 1.0, 1.0/N) # N equidistant pts
    tck, u = interpolate.splprep([X_utm, Y_utm], s=0)
    out = interpolate.splev(unew, tck)
    X_utm_out = out[0]
    Y_utm_out = out[1]    

    return X_utm_out, Y_utm_out, utm_nr, utm_letter 

#def get_oval_cl(l_long,R_curve,lanewidth):
#    # section a 
#    la = l_long
#    Na = la # default 1 node per meter
#    Xa = np.linspace(0,la,Na)
#    Ya = np.zeros(Na)
#
#    # section b
#    Nb = np.ceil(R_curve*np.pi)
#    t = np.linspace(np.pi,0,Nb)
#    Xb = R_curve*np.cos(t) + Xa[-1]
#    Yb = R_curve*np.sin(t) - R_curve + Ya[-1]    
#
#    # section c
#    lextra = 5
#    lc = la + lextra
#    Nc = lc
#    Xc = np.linspace(Xb[-1],Xb[-1]-lc,Nc)
#    Yc = np.zeros(Nc)+Yb[-1]
#    
#    # section h
#    Nd = np.ceil(R_curve*np.pi) 
#    t = np.linspace(-np.pi/2,-3*np.pi/2,Nh)
#    Xh = R*np.cos(t) + Xg[-1]
#    Yh = R*np.sin(t) + Yg[-1]+R
#
#    
#    # section i
#    Ni = li
#    Xi = np.linspace(-li,0,Ni)
#    Yi = np.zeros(Ni)

# Centerline generation functions
def get_Lshaped_cl(l_vert,l_hor,R_curve):
    # params
    la = l_vert # back of L shape (200 short 300 long)
    lc = l_hor  # base of L shape
    Rb  = R_curve  # 22 radius of curve at experiment turn
    R = 20  # radius at other turns (30) 
    
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
    li = 5
    lg = la+Rb-3*R + li
    Ng = lg
    Xg = np.linspace(Xf[-1],Xf[-1]-lg,Ng)
    Yg = np.zeros(Ng)+Yf[-1]
    
    # section h
    Nh = np.ceil(R*np.pi) 
    t = np.linspace(-np.pi/2,-3*np.pi/2,Nh)
    Xh = R*np.cos(t) + Xg[-1]
    Yh = R*np.sin(t) + Yg[-1]+R
    
    # section i
    Ni = li
    Xi = np.linspace(-li,0,Ni)
    Yi = np.zeros(Ni)
    
    # concatenate vectors
    X_cl_tmp = np.concatenate((Xa, Xb, Xc, Xd, Xe, Xf, Xg, Xh, Xi), axis=0)
    Y_cl_tmp = np.concatenate((Ya, Yb, Yc, Yd, Ye, Yf, Yg, Yh, Yi), axis=0)
    
    # remove duplicate points
    threshold_dist = 0.1
    X_cl = []
    Y_cl = []
    for i in range(X_cl_tmp.size-1):
        dist = np.sqrt((X_cl_tmp[i+1]-X_cl_tmp[i])**2 + (Y_cl_tmp[i+1]-Y_cl_tmp[i])**2)
        if (dist > threshold_dist):
            X_cl.append(X_cl_tmp[i])
            Y_cl.append(Y_cl_tmp[i])        
    return np.array(X_cl), np.array(Y_cl)


def export_as_yaml(track_name, export_path, dict_track):
    file_path = export_path + '/' + track_name + ".yaml"
    with open(file_path, 'w') as outfile:
        yaml.dump(dict_track, outfile, default_flow_style = False)
    print "[INFO] Saving track to: ",file_path    

def export_as_sdf(track_name, export_path, dict_track):
    
    cones_right_X = np.array(dict_track["cones_right"])[:,0]
    cones_right_Y = np.array(dict_track["cones_right"])[:,1]
    cones_left_X  = np.array(dict_track["cones_left"])[:,0]
    cones_left_Y  = np.array(dict_track["cones_left"])[:,1] 
    cones_orange_big_X  = np.array(dict_track["cones_orange_big"])[:,0]
    cones_orange_big_Y  = np.array(dict_track["cones_orange_big"])[:,1]     
    tk_device_X = np.array(dict_track["tk_device"])[:,0]
    tk_device_Y = np.array(dict_track["tk_device"])[:,1]    
    
    root = etree.Element("sdf", version="1.4")
    model = etree.SubElement(root, "model", name=track_name)
    
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

    file_path = export_path + '/' + track_name + ".sdf"
    tree.write(file_path, pretty_print=True, xml_declaration=True, encoding='UTF-8')
    print "[INFO] Saving track to: ",file_path


def export_as_kml(track_name, export_path, X_cl,Y_cl,origin_pose_utm):
    
    # store gps coordinates of centerline
    X_cl_utm = origin_pose_utm["X0_utm"] + X_cl*np.cos(origin_pose_utm["psi0_utm"])-Y_cl*np.sin(origin_pose_utm["psi0_utm"])
    Y_cl_utm = origin_pose_utm["Y0_utm"] + Y_cl*np.cos(origin_pose_utm["psi0_utm"])+X_cl*np.sin(origin_pose_utm["psi0_utm"])
    
    lat_cl, lon_cl = utm.to_latlon(X_cl_utm, Y_cl_utm, origin_pose_utm["utm_nr"], origin_pose_utm["utm_letter"])
    kml = simplekml.Kml()
    ls = kml.newlinestring(name=track_name)
    #ls.coords = [(18.333868,-34.038274,10.0), (18.370618,-34.034421,10.0)]
    pointlist = []
    for i in range(X_cl.size):
        point_as_tuple = (lon_cl[i],lat_cl[i],0.5) # set fixed height above gnd
        pointlist.append(point_as_tuple)    
    ls.coords = pointlist
    ls.extrude = 1
    ls.altitudemode = simplekml.AltitudeMode.relativetoground
    ls.style.linestyle.width = 5
    ls.style.linestyle.color = simplekml.Color.blue
    
    file_path = export_path + '/' + track_name + ".kml"
    kml.save(file_path)
    print "[INFO] Saving track to: ",file_path
    
    
    
    

#
# Track Generation
#
plt.close('all')
track_name = "storaholm_gravel_south"

# export params
export_path_fssim = "/home/larsvens/ros/tamp__ws/src/fssim/fssim_gazebo/models/track"
export_path_saarti = "/home/larsvens/ros/tamp__ws/src/saarti/common/config/tracks/" + track_name
if not os.path.exists(export_path_saarti):
    os.makedirs(export_path_saarti)

if(track_name == "asta_zero_short"):
    # set origin pose in UTM to AstaZero HSA
    origin_pose_utm =	{
      "X0_utm": 367498,
      "Y0_utm": 6406767,
      "psi0_utm": -1.1949329177828036,
      "utm_nr": 33, 
      "utm_letter": 'V'
    }
    
    l_vert = 200
    l_hor = 70
    R_corner = 22
    lanewidth = 2.3
    X_cl, Y_cl = get_Lshaped_cl(l_vert,l_hor,R_corner)   
    
elif(track_name == "asta_zero_long"):
    # set origin pose in UTM to AstaZero HSA
    origin_pose_utm =	{
      "X0_utm": 367498,
      "Y0_utm": 6406767,
      "psi0_utm": -1.1949329177828036,
      "utm_nr": 33, 
      "utm_letter": 'V'
    }
    
    l_vert = 300
    l_hor = 70
    R_corner = 22
    lanewidth = 2.3
    X_cl, Y_cl = get_Lshaped_cl(l_vert,l_hor,R_corner)
    
elif(track_name == "asta_local_min"):
    # set origin pose in UTM to AstaZero HSA
    origin_pose_utm =	{
      "X0_utm": 367498,
      "Y0_utm": 6406767,
      "psi0_utm": -1.1949329177828036,
      "utm_nr": 33, 
      "utm_letter": 'V'
    }
    
    l_vert = 200
    l_hor = 80
    R_corner = 35
    lanewidth = 2.8
    X_cl, Y_cl = get_Lshaped_cl(l_vert,l_hor,R_corner)
    Y_cl = -Y_cl
    
if(track_name == "frihamnen"):
    filepath = path.join('../config/tracks/ge_exports/frihamnen_0.kml')
    lanewidth = 2.3
    X_cl, Y_cl, X0_utm, Y0_utm, psi0_utm, utm_nr, utm_letter = get_cl_from_kml(filepath)           

    origin_pose_utm =	{
      "X0_utm": X0_utm,
      "Y0_utm": Y0_utm,
      "psi0_utm": psi0_utm,
      "utm_nr": utm_nr, 
      "utm_letter": utm_letter
    }

if(track_name == "rural_test_route_1"):
    filepath = path.join('../config/tracks/ge_exports/rural_test_route_1.kml')
    lanewidth = 2.3
    X_utm, Y_utm, utm_nr, utm_letter = get_cl_from_kml(filepath)           

    # adjust offset for this route
    Xoffs = -90
    Yoffs = -150
    X_utm = X_utm+Xoffs
    Y_utm = Y_utm+Yoffs
 
    # set utm origin pose
    X0_utm = float(X_utm[0])
    Y0_utm = float(Y_utm[0])
    psi0_utm = float(np.arctan2(Y_utm[1]-Y_utm[0],X_utm[1]-X_utm[0]))
            
    origin_pose_utm =	{
      "X0_utm": X0_utm,
      "Y0_utm": Y0_utm,
      "psi0_utm": psi0_utm,
      "utm_nr": utm_nr, 
      "utm_letter": utm_letter
    }

    # set centerline in map coord sys
    X_cl = X_utm - X0_utm
    Y_cl = Y_utm - Y0_utm

if(track_name == "storaholm_gravel_south"):
    filepath = path.join('../config/tracks/ge_exports/storaholm_gravel_south.kml')
    lanewidth = 2.3
    X_utm, Y_utm, utm_nr, utm_letter = get_cl_from_kml(filepath)           

    # adjust offset for this route
    Xoffs = 0
    Yoffs = 0
    X_utm = X_utm+Xoffs
    Y_utm = Y_utm+Yoffs
 
    # set utm origin pose
    X0_utm = float(X_utm[0])
    Y0_utm = float(Y_utm[0])
    psi0_utm = float(np.arctan2(Y_utm[1]-Y_utm[0],X_utm[1]-X_utm[0]))
            
    origin_pose_utm =	{
      "X0_utm": X0_utm,
      "Y0_utm": Y0_utm,
      "psi0_utm": psi0_utm,
      "utm_nr": utm_nr, 
      "utm_letter": utm_letter
    }

    # set centerline in map coord sys
    X_cl = X_utm - X0_utm
    Y_cl = Y_utm - Y0_utm
    
# compute s
X_cl = np.append(X_cl, X_cl[0])
Y_cl = np.append(Y_cl, Y_cl[0])
dX = np.diff(X_cl)
dY = np.diff(Y_cl)
ds = np.sqrt(dX**2+dY**2)
s_tmp = np.cumsum(ds)
s_tmp = np.append(0, s_tmp)

# resample with equidistant points
s = np.arange(0,s_tmp[-1],1)
X_cl = np.interp(s,s_tmp,X_cl)
Y_cl = np.interp(s,s_tmp,Y_cl)
centerline = np.column_stack((np.array(X_cl),np.array(Y_cl)))

# compute psic
dX = np.diff(X_cl)
dY = np.diff(Y_cl)
psic = np.arctan2(dY,dX)
psic_final = np.arctan2(Y_cl[0]-Y_cl[-1],X_cl[0]-X_cl[-1])  # assuming closed track
psic = np.append(psic,psic_final) 

# compute curvature
psic_cont = angleToContinous(psic)
ds_factor = 0.8
s_ds = np.linspace(0,s[-1],int(ds_factor*s.size))
psic_ds = np.interp(s_ds,s,psic_cont)

t, c, k = interpolate.splrep(s_ds, psic_ds, s=0, k=4)
psic_spl = interpolate.BSpline(t, c, k, extrapolate=False)
kappac_spl = psic_spl.derivative(nu=1)
kappacprime_spl = psic_spl.derivative(nu=2)
kappac = kappac_spl(s)
kappacprime = kappacprime_spl(s)
if(np.isnan(np.sum(kappac))): 
    raise Exception('kappac has nans')

# set left and right boundaries
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
cones_left = np.column_stack((np.array(cones_left_X),np.array(cones_left_Y)))

cones_right_X = [X_rl[0]]
cones_right_Y = [Y_rl[0]]
for i in range(X_rl.size-1):
    dist = np.sqrt((X_rl[i]-cones_right_X[-1])**2 + (Y_rl[i]-cones_right_Y[-1])**2)
    if(dist > threshold_dist_cones):
        cones_right_X.append(X_rl[i])
        cones_right_Y.append(Y_rl[i])
cones_right = np.column_stack((np.array(cones_right_X),np.array(cones_right_Y)))

# add additional track info for gazebo sim 
cones_orange = np.column_stack((np.array([]),np.array([]))) 
cones_orange_big = np.column_stack((np.array([4.7, 4.7, 7.3, 7.3]),np.array([4.0, -4.0, 4.0, -4.0])))
tk_device = np.column_stack((np.array([6.0, 6.0]),np.array([4.5, -4.5])))
starting_pos_front_wing = np.array([0.1,0.0]) # small adjust to avoid negative s

# rotate
R =  np.array([[ np.cos(psic[0]), np.sin(psic[0])],
                [-np.sin(psic[0]), np.cos(psic[0])]])
cones_orange_big = np.dot(cones_orange_big,R)
tk_device = np.dot(tk_device,R)
starting_pos_front_wing = np.dot(starting_pos_front_wing,R)
starting_pose_front_wing = np.array([starting_pos_front_wing[0], starting_pos_front_wing[1], psic[0]]) 

# plot things
plt.rcParams['figure.dpi'] = 100 # default 100
plt.rcParams['figure.figsize'] = 15, 5

plt.plot(X_cl,Y_cl,'.')

# lane limits
plt.plot(X_ll,Y_ll,'-k')
plt.plot(X_rl,Y_rl,'-k')

# cone positions
plt.plot(cones_left[:,0],cones_left[:,1],'*r')
plt.plot(cones_right[:,0],cones_right[:,1],'*r')

# start line markers
plt.plot(tk_device[:,0],tk_device[:,1],'b*')
plt.plot(cones_orange_big[:,0],cones_orange_big[:,1],'y*')

# starting position
plt.arrow(starting_pose_front_wing[0], starting_pose_front_wing[1], \
          3*np.cos(starting_pose_front_wing[2]), 3*np.sin(starting_pose_front_wing[2]), \
          head_width=0.5, head_length=0.7)

plt.gca().set_aspect('equal', adjustable='box')
plt.show()

fig, ax = plt.subplots()
ax.plot(s,psic,'k*')
ax.plot(s,psic_cont,'r*')
ax.plot(s,psic_spl(s),'b.')
ax.set_title('psic')
plt.show()

fig, ax = plt.subplots()
ax.plot(s,kappac,'k*')
ax.set_title('kappac')
plt.show()

# export as .yaml .sdf and .kml
dict_track = {"centerline": np.array(centerline).tolist(),
              "tangent": psic.tolist(),
              "curvature": kappac.tolist(),
              "curvilinear_abscissa": s.tolist(),
              "cones_left": np.array(cones_left).tolist(),
              "cones_left_normal_dist": dub.tolist(),
              "cones_right": np.array(cones_right).tolist(),
              "cones_right_normal_dist": dlb.tolist(),
              "cones_orange": np.array(cones_orange).tolist(),
              "cones_orange_big": np.array(cones_orange_big).tolist(),
              "tk_device": np.array(tk_device).tolist(),
              "starting_pose_front_wing": starting_pose_front_wing.tolist(),
              "origin_pose_utm": origin_pose_utm
              }

export_as_yaml(track_name, export_path_saarti, dict_track)
export_as_sdf(track_name, export_path_fssim, dict_track)
export_as_kml(track_name, export_path_saarti, X_cl,Y_cl,origin_pose_utm)
print "[INFO] Track generation completed"



