# TAMP: Traction Adaptive Motion Planning

Main repo for developing traction adaptive motion planning using sampling augmented adaptive RTI. Under development!    
The TAMP algorithm uses sampling augmented adaptive RTI to allow dynamically setting the tire force constraints. This enables the motion planner to deal with locally varying traction conditions in critical maneuvers. Details on the algorithm are available here: https://arxiv.org/abs/1903.04240. Simulations are done using https://github.com/AMZ-Driverless/fssim and the RTI solver is exported by the acado toolkit https://github.com/acado/acado. 

## Example 1: Curve with reduced traction
<p align="center"> 
<img src="doc/static_vs_dynamic_constraints_reduced_mu_turn.gif" width="600" />
</p>

## Example 2: Autonomous Racing
<p align="center"> 
<img src="doc/tamp_racing_demo.gif" width="600" />
</p>


## Setup    
System configuration: ubuntu 16.04 LTS & ROS Kinetic   
http://wiki.ros.org/kinetic/Installation/Ubuntu   
Tested with Python version 2.7.12 and Scipy version 1.2.2

dependencies:   
`apt-get install ros-kinetic-jsk-rviz-plugins`   

clone this repo and fork of fssim to a new catkin workspace   
`git clone git@github.com:larsvens/tamp_ws.git`   
`git clone git@github.com:larsvens/fssim.git`   

clone my fork of the acado repo https://github.com/larsvens/acado_fork (outside the workspace), and follow the deploy instructions in the readme. 

build everything in the tamp workspace with `catkin build`   

To run the racing demo:   
`source devel/setup.bash`   
`roslaunch common bringup_gotthard_FSG.launch`   
`roslaunch common experiment.launch exp_config:="gotthard_racing_nonadapt_config.yaml"`   
`saarti saarti_node.launch`   
`roslaunch common ctrl_interface.launch`   

### Cite

If you find the code useful in your own research, please consider citing 

    @article{svensson2019adaptive,
      title={Adaptive trajectory planning and optimization at limits of handling},
      author={Svensson, Lars and Bujarbaruah, Monimoy and Kapania, Nitin and T{\"o}rngren, Martin},
      journal={arXiv preprint arXiv:1903.04240},
      year={2019}
    }
