#!/usr/bin/env python
"""
Description: 
    Standalone script for generating obstacle config files

Usage:
    Todo
"""

import yaml

def export_as_yaml(filename, export_path, obs_dict):
    file_path = export_path + '/' + filename + ".yaml"
    with open(file_path, 'w') as outfile:
        yaml.dump(obs_dict, outfile, default_flow_style = False)
    print "[INFO] Saving obstacle config to: ",file_path 


files_to_generate = ["single_popup_1","double_popup_1"]
export_path = "/home/larsvens/ros/tamp__ws/src/saarti/common/config/obstacles" # all files will be generated in this dir




if("single_popup_1" in files_to_generate):
    obs_dict = {
        's_ego_at_popup': [240.],
        's_obs_at_popup': [260.],
        'd_obs_at_popup': [0.6,]
    }
    export_as_yaml("single_popup_1", export_path, obs_dict)

if("double_popup_1" in files_to_generate):
    obs_dict = {
        's_ego_at_popup': [240.,260.],
        's_obs_at_popup': [260.,280.],
        'd_obs_at_popup': [0.6,-1.0]
    }
    export_as_yaml("double_popup_1", export_path, obs_dict)
    
    
# todo:
# Random location in test area (one per lap)
# random in gauntlet 