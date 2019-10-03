#ifndef SAARTI_NODE_H
#define SAARTI_NODE_H

// ros
#include "ros/ros.h"
#include "ros/time.h"
#include <tf2/LinearMath/Quaternion.h>

#include "std_msgs/String.h"
// visualization
#include "visualization_msgs/MarkerArray.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Path.h"

// saarti
#include "saarti/rtisqp_wrapper.h"
#include <common/Path.h>
#include <common/Obstacles.h>
#include <common/Trajectory.h>
#include <common/State.h>

// timing
#include <chrono>

// misc
#include <sstream>
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant" // supress warning at ros prints

// namespaces
using std::vector;
using std::cout;
using std::endl;

namespace saarti_node{
class SAARTI{
public:
    // constructor
    SAARTI(ros::NodeHandle nh);

private:
    // modes (todo get from launch)
    uint ctrl_mode_ = 2;    // 0: tracking(unused todo remove), 1: min s, 2: max s,
    uint algo_setting_ = 0; // 0: RTI, 1: SAA-RTI
    int Ntrajs_rollout_ = 24; // default 24

    // weights (todo get from launch)
    vector<float> Wx{10.0, 1.0, 1.0, 0.01, 0.01, 0.01};
    vector<float> Wu{0.1, 0.1};
    float Wslack = 10000000;

    // variables
    double dt;
    ros::NodeHandle nh_;
    ros::Subscriber pathlocal_sub_;
    ros::Subscriber obstacles_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher trajstar_pub_;
    ros::Publisher trajhat_pub_;
    ros::Publisher trajset_vis_pub_;
    ros::Publisher trajhat_vis_pub_;
    ros::Publisher trajstar_vis_pub_;
    ros::Publisher posconstr_vis_pub_;
    planning_util::statestruct state_;
    planning_util::pathstruct pathlocal_;
    vector<planning_util::trajstruct> trajset_;
    planning_util::obstastruct obst_;
    planning_util::refstruct refs_;
    RtisqpWrapper rtisqp_wrapper_;

    // functions
    void print_obj(planning_util::trajstruct traj);
    planning_util::refstruct setRefs(uint ctrlmode);
    void traj2cart(planning_util::trajstruct &traj);
    void trajset2cart();
    void sd_pts2cart(vector<float> &s, vector<float> &d, vector<float> &Xout, vector<float> &Yout);
    void angle_to_interval(vector<float> &psi);
    vector<float> angle_to_continous(vector<float> &psi);
    int trajset_eval_cost();
    common::Trajectory traj2msg(planning_util::trajstruct traj);
    nav_msgs::Path traj2navpath(planning_util::trajstruct traj);
    void trajset2ma();
    visualization_msgs::Marker trajset2cubelist();
    jsk_recognition_msgs::PolygonArray stateconstr2polarr(planning_util::posconstrstruct pc);
    void state_callback(const common::State::ConstPtr& msg);
    void pathlocal_callback(const common::Path::ConstPtr& msg);
    void obstacles_callback(const common::Obstacles::ConstPtr& msg);



};
} // end namespace saarti_node
#endif // SAARTI_NODE_H
