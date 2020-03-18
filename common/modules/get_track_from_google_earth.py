#!/usr/bin/env python
from os import path
import re
import numpy as np
import utm
from scipy import interpolate

#import xml.etree.ElementTree as ET
from lxml import etree 
kml_filename = path.join('../config/tracks/frihamnen_ge/frihamnen_0.kml')
tree = etree.parse(kml_filename)

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
origin_pose_utm = {
  "X0_utm": X_utm[0],
  "Y0_utm": Y_utm[0],
  "psi0_utm": np.arctan2(Y_utm[1]-Y_utm[0],X_utm[1]-X_utm[0]),
  "utm_nr": utm_nr,
  "utm_letter": utm_letter
}
X = X_utm - origin_pose_utm["X0_utm"]
Y = Y_utm - origin_pose_utm["Y0_utm"]

# interpolate 
ds = 1.0 # want approx 1 m between pts
# approximate stot
stot = 0
for i in range(X.size-1):
    stot += np.sqrt((X[i+1]-X[i])**2 + (Y[i+1]-Y[i])**2)
N = int(stot/ds)
unew = np.arange(0, 1.0, 1.0/N) # N equidistant pts
tck, u = interpolate.splprep([X, Y], s=0)
out = interpolate.splev(unew, tck)
Xcl = out[0]
Ycl = out[1]





