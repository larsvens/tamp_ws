#ifndef SAARTI_NODE_H
#define SAARTI_NODE_H

// ros
#include "ros/ros.h"
#include "ros/time.h"
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

// visualization
#include "visualization_msgs/MarkerArray.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "jsk_recognition_msgs/PlotData.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Path.h"

// saarti
#include "saarti/rtisqp_wrapper.h"
#include <common/Path.h>
#include <common/Obstacles.h>
#include <common/Trajectory.h>
#include <common/State.h>

// timing and threads
#include <chrono>
#include <thread>
#include <future>

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
    // general
    float dt_;
    float dt_integrator_ = 0.1; // todo get from param
    float cutoff_speed_;
    bool planner_activated_;

    // modes
    int ref_mode_;
    int sampling_augmentation_;
    int traction_adaptive_;

    // params
    float mu_nominal_; // only used for nonadaptive case
    float vxref_cc_; // only used for velocity keeping (refmode 1)
    float dref_cc_; // only used for velocity keeping (refmode 1)
    int Ntrajs_rollout_;
    vector<float> Wx_;
    vector<float> WNx_;
    vector<float> Wu_;
    float Wslack_;

    // internal variables
    ros::NodeHandle nh_;
    ros::Subscriber pathlocal_sub_;
    ros::Subscriber obstacles_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber ctrlmode_sub_;
    ros::Publisher trajstar_pub_;
    ros::Publisher trajhat_pub_;
    ros::Publisher trajset_vis_pub_;
    ros::Publisher trajhat_vis_pub_;
    ros::Publisher trajstar_vis_pub_;
    ros::Publisher trajstar_polarr_vis_pub_;
    ros::Publisher posconstr_vis_pub_;
    ros::Publisher vectordebug_pub_;
    planning_util::statestruct state_;
    int ctrlmode_;
    planning_util::pathstruct pathlocal_;
    vector<planning_util::trajstruct> trajset_;
    planning_util::obstastruct obst_;
    planning_util::refstruct refs_;
    planning_util::staticparamstruct sp_;
    RtisqpWrapper rtisqp_wrapper_;

    // functions
    void print_obj(planning_util::trajstruct traj);
    planning_util::refstruct setRefs(int ref_mode_, int traction_adaptive_, float mu_nominal_, float vxref_cc, float dref_cc, planning_util::staticparamstruct sp_, planning_util::pathstruct pathlocal_);
    void traj2cart(planning_util::trajstruct &traj);
    void get_additional_traj_variables(planning_util::trajstruct &traj, planning_util::pathstruct &pathlocal, planning_util::staticparamstruct sp);
    void trajset2cart();
    void sd_pts2cart(vector<float> &s, vector<float> &d, vector<float> &Xout, vector<float> &Yout);
    void angle_to_interval(vector<float> &psi);
    vector<float> angle_to_continous(vector<float> &psi);
    int trajset_eval_cost();
    common::Trajectory traj2msg(planning_util::trajstruct traj);
    nav_msgs::Path traj2navpath(planning_util::trajstruct traj);
    jsk_recognition_msgs::PolygonArray traj2polarr(planning_util::trajstruct traj, planning_util::staticparamstruct sp);
    Eigen::MatrixXf get_vehicle_corners(float X, float Y, float psi, float lf, float lr, float w);
    void trajset2ma();
    visualization_msgs::Marker trajset2cubelist();
    jsk_recognition_msgs::PolygonArray stateconstr2polarr(planning_util::posconstrstruct pc);
    void state_callback(const common::State::ConstPtr& msg);
    void ctrlmode_callback(const std_msgs::Int16::ConstPtr& msg);
    void pathlocal_callback(const common::Path::ConstPtr& msg);
    void obstacles_callback(const common::Obstacles::ConstPtr& msg);
    void get_rosparams();
    void run_optimization();

};
} // end namespace saarti_node
#endif // SAARTI_NODE_H
