# TAMP: Traction Adaptive Motion Planning

Main repo for developing traction adaptive motion planning using sampling augmented adaptive RTI. Keep in mind this is research code under development and is therefore subject to changes.   

The framework uses sampling augmented adaptive RTI to allow dynamically updating tire force constraints. This enables the motion planner to deal with locally varying traction conditions in aggressive maneuvers. Details on the algorithm are available here: https://arxiv.org/abs/2009.04180 and https://arxiv.org/abs/1903.04240. Simulations are done using https://github.com/AMZ-Driverless/fssim and the RTI solver is exported by the acado toolkit https://github.com/acado/acado.     

## Example 1: Curve with reduced traction
<p align="center"> 
<img src="doc/static_vs_dynamic_constraints_reduced_mu_turn.gif" width="600" />
</p>

## Example 2: Autonomous Racing
<p align="center"> 
<img src="doc/tamp_racing_demo.gif" width="600" />
</p>


## Setup with fssim     
Test system configuration: ubuntu 16.04 LTS, ROS Kinetic, Python 2.7.12 and Scipy version 1.2.2   

Additional dependencies:   
`apt-get install ros-kinetic-jsk-rviz-plugins`   

clone this repo and fork of fssim to a new catkin workspace   
`git clone git@github.com:larsvens/tamp_ws.git`   
`git clone git@github.com:larsvens/fssim.git`   

build with `catkin build`   

To run the racing demo:   
`source devel/setup.bash`   
`roslaunch common bringup_gotthard_FSG.launch`   
`roslaunch common experiment.launch exp_config:="gotthard_racing_nonadapt_config.yaml"`   
`saarti saarti_node.launch`   
`roslaunch common ctrl_interface.launch`   

## Setup standalone (or to integrate with existig ros framework)   
1. clone your fork of the tamp repo into the src directory of your catkin workspace. Example: `git clone --single-branch --branch racing_devel_2021 git@github.com:larsvens/tamp_ws.git saarti` This will download all the saarti ros packages in a main folder called saarti.   
2. Next, clone my fork of the acado tookit (racingdevel branch) to a convenient location outside your catkin workspace `git clone --single-branch --branch racing_devel git@github.com:larsvens/acado_fork.git` 
3. build the acado toolkit: cd to root dir of the repo (`acado_fork` by default), do `mkdir build`, `cd build`, `cmake ..` and `make` in sequence.
4. update the `DEPLOY_DIR` field of the file `acado_fork/deploy_rtisqp.bash` such that it points to the saarti folder in your catkin workspace. Save the edited file.
5. run the deploy script using the command `./deploy_rtisqp.bash`. Now you have exported a fast problem specific rtisqp solver as generated c-code into your saarti folder in the catkin workspace.  
6. now you can proceed to building the catkin workspace. cd to the root of the catkin workspace and run `catkin build`
7. to test the saarti node, run `roslaunch common tamp_racing_test.launch`

In order to change the auto-generated solver (for example parameters of the vehicle model) you need to go through these steps. 
1. edit the file `acado_fork/examples/code_generation/mpc_mhe/rtisqp.cpp`
2. build the acado toolkit by repeating step 3 above (if you have built before you can just cd to `build` and do `make`)
3. deploy, build and run as per 5,6,7 above


### Cite

If you find the code useful in your own research, please consider citing 

    @article{svensson2019adaptive,
      title={Adaptive trajectory planning and optimization at limits of handling},
      author={Svensson, Lars and Bujarbaruah, Monimoy and Kapania, Nitin and T{\"o}rngren, Martin},
      journal={arXiv preprint arXiv:1903.04240},
      year={2019}
    }
