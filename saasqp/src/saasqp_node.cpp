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

    void state_callback(const common::State::ConstPtr& msg){
        state_ = *msg;
    }

    // constructor
    SAASQP(ros::NodeHandle nh){
        nh_ = nh;
        dt = 0.1;
        ros::Rate loop_rate(1/dt);

        // pubs & subs        
        trajstar_pub_ = nh.advertise<common::Trajectory>("trajstar",1);
        trajhat_sub_  = nh.subscribe("trajhat", 1, &SAASQP::traj_callback,this);
        state_sub_ = nh.subscribe("state", 1,  &SAASQP::state_callback,this);

        // init wrapper for rtisqp solver
        rtisqp_wrapper_ = RtisqpWrapper();

        // main loop
        while (ros::ok())
        {
            // print diagnostics
            std::cout << std::endl;
            ROS_INFO_STREAM("main loop");

            // check for valid trajhat
            if((trajhat_.s.size() != 0) && (state_.s > 0) ){

                // update current state
                ROS_INFO_STREAM("setting state..");
                rtisqp_wrapper_.setInitialState(state_);

                // update adaptive constraints
                // todo

                // set initial guess
                ROS_INFO_STREAM("setting initial guess..");
                rtisqp_wrapper_.setInitialGuess(trajhat_);

                // set reference
                ROS_INFO_STREAM("setting reference..");
                rtisqp_wrapper_.setReference(trajhat_);

                // set state constraint
                // todo

                // do preparation step // todo: put timer
                ROS_INFO_STREAM("calling acado prep step..");
                rtisqp_wrapper_.doPreparationStep();

                // do feedback step // todo: put timer
                ROS_INFO_STREAM("calling acado feedback step..");
                int status = rtisqp_wrapper_.doFeedbackStep();
                if (status){
                    std::cout << "QP problem! QP status: " << status << std::endl;
                    break;
                }

                // extract state and control trajs from acado

                // get trajstar
                // todo

                // publish trajstar
                trajstar_ = rtisqp_wrapper_.getTrajectory();
                trajstar_.header.stamp = ros::Time::now();
                trajstar_pub_.publish(trajstar_);

                // shift X and U


            } else {
                ROS_INFO_STREAM("waiting for state and trajhat");
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
private:
    double dt;
    ros::NodeHandle nh_;
    ros::Subscriber trajhat_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher trajstar_pub_;
    //ACADOvariables acadoVariables;
    //ACADOworkspace acadoWorkspace;
    common::Trajectory trajstar_;
    common::Trajectory trajhat_;
    common::State state_;
    RtisqpWrapper rtisqp_wrapper_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "saasqp");
    ros::NodeHandle nh;
    SAASQP saasqp(nh);
    return 0;
}
