// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <common/Trajectory.h>
#include <sstream>

// saasqp
#include "saasqp/rtisqp_wrapper.h"


class SAASQP
{
public:
    void traj_callback(const common::Trajectory::ConstPtr& msg){
        trajhat_ = *msg;
    }

    // constructor
    SAASQP(ros::NodeHandle nh){
        nh_ = nh;
        dt = 0.1;
        ros::Rate loop_rate(1/dt);

        // pubs & subs
        traj_sub_ = nh.subscribe("trajhat",1, &SAASQP::traj_callback,this);

        // init wrapper for rtisqp solver
        rtisqp_wrapper_ = RtisqpWrapper();

        // set initial guess
        rtisqp_wrapper_.setInitialGuess(trajhat_);

        // set references



        // complete one step and plot before putting anything in loop..
        while (ros::ok())
        {
            // print diagnostics
            //ROS_INFO_STREAM("trajectory stamp: " << trajhat_.header.stamp);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
private:
    double dt;
    ros::NodeHandle nh_;
    ros::Subscriber traj_sub_;
    ACADOvariables acadoVariables;
    ACADOworkspace acadoWorkspace;
    common::Trajectory trajhat_;
    RtisqpWrapper rtisqp_wrapper_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "saasqp");
    ros::NodeHandle nh;
    SAASQP saasqp(nh);
    return 0;
}
