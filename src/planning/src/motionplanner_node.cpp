// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <common/PathLocal.h>
#include <common/Obstacles.h>
#include <common/Trajectory.h>
#include <common/State.h>
#include <common/interp.h>
#include <sstream>


#include "planning/rtisqp_wrapper.h"

class SAARTI
{
public:
    void tmp_trajhat_callback(const common::Trajectory::ConstPtr& msg){
        tmp_trajhat_ = *msg;
    }

    void state_callback(const common::State::ConstPtr& msg){
        state_ = *msg;
    }

    void pathlocal_callback(const common::PathLocal::ConstPtr& msg){
        pathlocal_ = *msg;
    }

    void obstacles_callback(const common::Obstacles::ConstPtr& msg){
        obstacles_ = *msg;
    }

    // constructor
    SAARTI(ros::NodeHandle nh){
        nh_ = nh;
        dt = 0.1;
        ros::Rate loop_rate(1/dt);

        // pubs & subs
        trajhat_pub_ = nh.advertise<common::Trajectory>("trajhat",1);
        trajstar_pub_ = nh.advertise<common::Trajectory>("trajstar",1);
        pathlocal_sub_ = nh.subscribe("pathlocal", 1, &SAARTI::pathlocal_callback,this);
        obstacles_sub_ = nh.subscribe("obstacles", 1, &SAARTI::obstacles_callback,this);
        tmp_trajhat_sub_  = nh.subscribe("tmp_trajhat", 1, &SAARTI::tmp_trajhat_callback,this);
        state_sub_ = nh.subscribe("state", 1,  &SAARTI::state_callback,this);

        // init wrapper for rtisqp solver
        rtisqp_wrapper_ = RtisqpWrapper();

        // set weights
        std::vector<double> Wx{10.0, 1.0, 1.0, 0.01, 0.01, 0.01};
        std::vector<double> Wu{0.1, 0.1};
        double Wslack = 10000000;
        rtisqp_wrapper_.setWeights(Wx,Wu,Wslack);

        // wait until tmp_trajhat, state and path_local is received
        while((tmp_trajhat_.s.size() == 0) || !(state_.s > 0) || pathlocal_.s.size() == 0 ){
            ROS_INFO_STREAM("waiting for state and tmp_trajhat");
            ros::spinOnce();
            loop_rate.sleep();
        }

        // initialize trajhat last (tmp: remove after sampling in place)
        common::Trajectory trajstar_last = tmp_trajhat_;

        // main loop
        while (ros::ok())
        {
            std::cout << std::endl;
            ROS_INFO_STREAM("main loop");
            ros::Time t_start = ros::Time::now();

            // update adaptive constraints
            rtisqp_wrapper_.setInputConstraints(1.0,1000);



            // selection step (tmp: always picks trajstar_last)
            common::Trajectory trajhat = trajstar_last; // create new every iteration to make sure it has all attributes
            //trajhat.s = trajstar_last.s;
            //trajhat.d = trajstar_last.d;



            // update current state
            ROS_INFO_STREAM("setting state..");
            rtisqp_wrapper_.setInitialState(state_);

            // check consistency between state and trajhat
            std::cout << "state.s - trajhat.s.at(0) = " << state_.s - trajhat.s.at(0) << std::endl;
            std::cout << "state.d - trajhat.d.at(0) = " << state_.d - trajhat.d.at(0) << std::endl;

            // set initial guess and shift fwd
            ROS_INFO_STREAM("setting trajstar as initial guess..");
            rtisqp_wrapper_.setInitialGuess(trajhat);
            //rtisqp_wrapper_.shiftStateAndControls();

            // set reference
            ROS_INFO_STREAM("setting reference..");
            int ctrlmode = 2; // 0: tracking, 1: min s, 2: max s,
            rtisqp_wrapper_.setReference(trajhat,ctrlmode);

            // set state constraint
            ROS_INFO_STREAM("setting state constraints..");
            std::vector<float> lld = cpp_utils::interp(trajhat.s,pathlocal_.s,pathlocal_.dub,false);
            std::vector<float> rld = cpp_utils::interp(trajhat.s,pathlocal_.s,pathlocal_.dlb,false);
            rtisqp_wrapper_.setStateConstraints(trajhat,obstacles_,lld,rld);

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
            Eigen::MatrixXd Xstarx = rtisqp_wrapper_.getStateTrajectory();
            Eigen::MatrixXd Xstaru = rtisqp_wrapper_.getControlTrajectory();
            //std::cout << "Xstarx is of size " << Xstarx.rows() << "x" << Xstarx.cols() << std::endl;
            //std::cout << "Xstaru is of size " << Xstaru.rows() << "x" << Xstaru.cols() << std::endl;

            // set trajstar
            common::Trajectory trajstar;
            std::vector<float> Xstar_s;
            for (uint k = 0; k < N+1; ++k){
                Xstar_s.push_back(float(Xstarx(0,k)));
            }
            std::vector<float> Xc = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.X,false);
            std::vector<float> Yc = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.Y,false);
            std::vector<float> psic = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.psi_c,false);
            trajstar.kappac = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.kappa_c,false);



            for (uint k = 0; k < N+1; ++k){
                // states
                trajstar.s.push_back(float(Xstarx(0,k)));
                trajstar.d.push_back(float(Xstarx(1,k)));
                trajstar.deltapsi.push_back(float(Xstarx(2,k)));
                trajstar.psidot.push_back(float(Xstarx(3,k)));
                trajstar.vx.push_back(float(Xstarx(4,k)));
                trajstar.vy.push_back(float(Xstarx(5,k)));

                //std::cout << "debug mark 1: k = " << k << std::endl;

                // cartesian pose
                trajstar.X.push_back(Xc.at(k) - trajstar.d.at(k)*std::sin(psic.at(k)));
                trajstar.Y.push_back(Yc.at(k) + trajstar.d.at(k)*std::cos(psic.at(k)));
                trajstar.psi.push_back(psic.at(k) + trajstar.deltapsi.at(k));


                //std::cout << "debug mark 2: k = " << k << std::endl;

                // forces (we have N+1 states but only N controls)
                if(k < N){
                    trajstar.Fyf.push_back(float(Xstaru(0,k)));
                    trajstar.Fx.push_back(float(Xstaru(1,k)));
                    trajstar.Fxf.push_back(0.5f*trajstar.Fx.at(k));
                    trajstar.Fxr.push_back(0.5f*trajstar.Fx.at(k));
                }

                //std::cout << "debug mark 3: k = " << k << std::endl;
            }

            // publish trajhat
            trajhat.header.stamp = ros::Time::now();
            trajhat_pub_.publish(trajhat);

            // publish trajstar
            trajstar.header.stamp = ros::Time::now();
            trajstar_pub_.publish(trajstar);

            // store fwd shifted trajstar for next iteration
            trajstar_last = trajstar;
            rtisqp_wrapper_.shiftTrajectoryFwdSimple(trajstar_last);

            // print loop time
            ros::Duration planningtime = ros::Time::now() - t_start;
            ROS_INFO_STREAM("planningtime = " << planningtime);



            ros::spinOnce();
            loop_rate.sleep();
        }
    }
private:
    double dt;
    ros::NodeHandle nh_;
    ros::Subscriber pathlocal_sub_;
    ros::Subscriber obstacles_sub_;
    ros::Subscriber tmp_trajhat_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher trajstar_pub_;
    ros::Publisher trajhat_pub_;
    common::PathLocal pathlocal_;
    common::Obstacles obstacles_;
    common::Trajectory tmp_trajhat_;
    common::State state_;
    RtisqpWrapper rtisqp_wrapper_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motionplanner");
    ros::NodeHandle nh;
    SAARTI saarti(nh);
    return 0;
}
