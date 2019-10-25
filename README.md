## TAMP: Traction Adaptive Motion Planning

Main repo for developing traction adaptive motion planning using sampling augmented adaptive RTI. Under development! Details on the algorithm are available here: https://arxiv.org/abs/1903.04240   

Basic demo of the algorithm running with the fssim simulator https://github.com/AMZ-Driverless/fssim .   
Planning time < 20ms with a prediction horizon of 3s on a laptop CPU.

<p align="center"> 
<img src="doc/tamp_demo.gif" width="600" />
</p>


### Basic setup instructions  

System configuration: ubuntu 16.04 LTS & ROS Kinetic   

dependencies:   
`apt-get install ros-kinetic-jsk-rviz-plugins`   

clone this repo and fork of fssim to a new catkin workspace   
`git clone git@github.com:larsvens/tamp_ws.git`   
`git clone git@github.com:larsvens/fssim.git`   

build with `catkin build`   

source the workspace with `source devel/setup.bash` and run demo with `roslaunch common demo.launch`   

### Cite

If you find the code useful in your research, please consider citing 

    @article{svensson2019adaptive,
      title={Adaptive trajectory planning and optimization at limits of handling},
      author={Svensson, Lars and Bujarbaruah, Monimoy and Kapania, Nitin and T{\"o}rngren, Martin},
      journal={arXiv preprint arXiv:1903.04240},
      year={2019}
    }
