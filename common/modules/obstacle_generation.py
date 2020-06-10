#!/usr/bin/env python
"""
Description: 
    Standalone script for generating obstacle config files

Usage:
    Todo
"""

import yaml
import numpy as np

def export_as_yaml(filename, export_path, obs_dict):
    file_path = export_path + '/' + filename + ".yaml"
    with open(file_path, 'w') as outfile:
        yaml.dump(obs_dict, outfile, default_flow_style = False)
    print "[INFO] Saving obstacle config to: ",file_path 


files_to_generate = ["single_popup_1", "single_popup_sh", "double_popup_1","gauntlet_east_single_lap_gen","oval_east_single_lap_gen","multiple_popup_10_laps"]
export_path = "/home/larsvens/ros/tamp__ws/src/saarti/common/config/obstacles" # all files will be generated in this dir


if("single_popup_1" in files_to_generate):
    obs_dict = {
        's_ego_at_popup': [160.],
        's_obs_at_popup': [180.],
        'd_obs_at_popup': [0.8,]
    }
    export_as_yaml("single_popup_1", export_path, obs_dict)

if("single_popup_sh" in files_to_generate):
    obs_dict = {
        's_ego_at_popup': [250.],
        's_obs_at_popup': [265.],
        'd_obs_at_popup': [0.8,]
    }
    export_as_yaml("single_popup_sh", export_path, obs_dict)

if("double_popup_1" in files_to_generate):
    obs_dict = {
        's_ego_at_popup': [240.,260.],
        's_obs_at_popup': [260.,280.],
        'd_obs_at_popup': [0.6,-1.0]
    }
    export_as_yaml("double_popup_1", export_path, obs_dict)

if("multiple_popup_10_laps" in files_to_generate):
    N_laps = 10
    s_lap = 792.8049693850619 # get from track gen script 
    s_ego_first = 175
    delta_s_between_popups = s_lap/2.
    s_ego_at_popup = np.arange(s_ego_first,s_lap*N_laps,delta_s_between_popups)
    Nobs = s_ego_at_popup.size

    # generate obstacles at random positions ahead of vehicle
    # use fixed seed to get consistency between runs of this script
    rs = 43
    np.random.seed(rs) 
    delta_s_base = 16
    s_pm_range = 1.0
    d_pm_range = 1
    s_obs_at_popup = s_ego_at_popup + delta_s_base + s_pm_range*(np.random.rand(Nobs)*2-1)
    d_obs_at_popup = d_pm_range*(np.random.rand(Nobs)*2-1)
    
    obs_dict = {
        's_ego_at_popup': s_ego_at_popup.tolist(),
        's_obs_at_popup': s_obs_at_popup.tolist(),
        'd_obs_at_popup': d_obs_at_popup.tolist()
    }
    export_as_yaml("multiple_popup_10_laps_rs"+str(rs), export_path, obs_dict)

        
if("gauntlet_east_single_lap_gen" in files_to_generate):
    s_lap = 619 # get from track gen script 
    s_ego_first = 50
    delta_s_between_popups = 25
    s_ego_at_popup = np.arange(s_ego_first,s_lap,delta_s_between_popups)
    Nobs = s_ego_at_popup.size
    
    # generate obstacles at random positions ahead of vehicle
    # use fixed seed to get consistency between runs of this script
    np.random.seed(42) 
    delta_s_base = 20
    s_pm_range = 1.5
    d_pm_range = 1
    s_obs_at_popup = s_ego_at_popup + delta_s_base + s_pm_range*(np.random.rand(Nobs)*2-1)
    d_obs_at_popup = d_pm_range*(np.random.rand(Nobs)*2-1)
    
    obs_dict = {
        's_ego_at_popup': s_ego_at_popup.tolist(),
        's_obs_at_popup': s_obs_at_popup.tolist(),
        'd_obs_at_popup': d_obs_at_popup.tolist()
    }
    export_as_yaml("gauntlet_east_single_lap_gen", export_path, obs_dict)

if("oval_east_single_lap_gen" in files_to_generate):
    s_lap = 488 # get from track gen script
    s_ego_first = 50
    delta_s_between_popups = 50
    s_ego_at_popup = np.arange(s_ego_first,s_lap,delta_s_between_popups)
    Nobs = s_ego_at_popup.size
    
    # generate obstacles at random positions ahead of vehicle
    # use fixed seed to get consistency between runs of this script
    np.random.seed(42) 
    delta_s_base = 20
    s_pm_range = 1.5
    d_pm_range = 1
    s_obs_at_popup = s_ego_at_popup + delta_s_base + s_pm_range*(np.random.rand(Nobs)*2-1)
    d_obs_at_popup = d_pm_range*(np.random.rand(Nobs)*2-1)
    
    obs_dict = {
        's_ego_at_popup': s_ego_at_popup.tolist(),
        's_obs_at_popup': s_obs_at_popup.tolist(),
        'd_obs_at_popup': d_obs_at_popup.tolist()
    }
    export_as_yaml("oval_east_single_lap_gen", export_path, obs_dict)   
    
    
    
    
    