#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 20 09:13:30 2019

@author: larsvens
"""

# specified params (http://segotn12827.rds.volvo.com/STPIFiles/Volvo/ModelRange/fh62pt6ha_gbr_eng.pdf)
m = 8350.0 
w_front = 5375
w_rear = 2960
wheelbase = 3.4
width = 2.6
lenght = 6.280

# estimated params
h = 1.0 # height of COM
r_wheel = 0.5
m_wheel = 75

# computed params
a = 0.8*wheelbase 
b = 0.8*width
Iz = (1/12.0)*m*(a**2+b**2) # approx as rectangle
w_ratio_front = w_front/m # percentage weight on front axle
w_ratio_rear = w_rear/m
lf = wheelbase*w_ratio_rear
lr = wheelbase*w_ratio_front
Iwheel = (1.0/2.0)*m_wheel*r_wheel**2 # approx as cylinder
