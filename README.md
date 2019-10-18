## TAMP: Traction Adaptive Motion Planning using sampling augmented adaptive RTI

<p align="center"> 
<img src="doc/tamp_demo.gif" width="600" />
</p>

### Basic setup instructions  

System configuration: ubuntu 16.04 LTS & ROS Kinetic   
https://www.ubuntu.com/download/alternative-downloads   
http://wiki.ros.org/kinetic/Installation   

dependencies:   
`apt-get install ros-kinetic-jsk-rviz-plugins`   

clone this repo and fork of fssim to a new catkin workspace   
`git clone git@github.com:larsvens/tamp_ws.git`   
`git clone https://github.com/larsvens/fssim`   

build with `catkin build`   

source the workspace with `source devel/setup.bash` and run demo with `roslaunch common demo.launch`   

