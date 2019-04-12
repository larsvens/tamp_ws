// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <common/PathLocal.h>
#include <common/Trajectory.h>
#include <common/State.h>
#include <common/interp.h>
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

    void pathlocal_callback(const common::PathLocal::ConstPtr& msg){
        pathlocal_ = *msg;
    }

    // constructor
    SAASQP(ros::NodeHandle nh){
        nh_ = nh;
        dt = 0.1;
        ros::Rate loop_rate(1/dt);

        // pubs & subs        
        trajstar_pub_ = nh.advertise<common::Trajectory>("trajstar",1);
        pathlocal_sub_ = nh.subscribe("pathlocal", 1, &SAASQP::pathlocal_callback,this);
        trajhat_sub_  = nh.subscribe("trajhat", 1, &SAASQP::traj_callback,this);
        state_sub_ = nh.subscribe("state", 1,  &SAASQP::state_callback,this);

        // init wrapper for rtisqp solver
        rtisqp_wrapper_ = RtisqpWrapper();

        // set weights
        std::vector<double> Wx{10000.0, 10000.0, 10000.0, 0.1, 0.1, 0.1};
        std::vector<double> Wu{0.001, 0.001};
        rtisqp_wrapper_.setWeights(Wx,Wu);

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
                Eigen::MatrixXd Xstarx = rtisqp_wrapper_.getStateTrajectory();
                Eigen::MatrixXd Xstaru = rtisqp_wrapper_.getControlTrajectory();
                //std::cout << "Xstarx" << Xstarx << std::endl;
                //std::cout << "Xstaru" << Xstaru << std::endl;

                // set trajstar
                std::vector<float> Xstar_s;
                for (uint k = 0; k < N; ++k){
                    Xstar_s.push_back(float(Xstarx(0,k)));
                }
                std::vector<float> Xc = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.X,false);
                std::vector<float> Yc = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.Y,false);
                std::vector<float> psic = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.psi_c,false);
                trajstar_.kappac = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.kappa_c,false);
                //std::vector<float> kappac = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.kappa_c,false);

                for (uint k = 0; k < N; ++k){
                    // states
                    trajstar_.s.push_back(float(Xstarx(0,k)));
                    trajstar_.d.push_back(float(Xstarx(1,k)));
                    trajstar_.deltapsi.push_back(float(Xstarx(2,k)));
                    trajstar_.psidot.push_back(float(Xstarx(3,k)));
                    trajstar_.vx.push_back(float(Xstarx(4,k)));
                    trajstar_.vy.push_back(float(Xstarx(5,k)));

                    // cartesian pose
                    trajstar_.X.push_back(Xc.at(k) - trajstar_.d.at(k)*std::sin(psic.at(k)));
                    trajstar_.Y.push_back(Yc.at(k) + trajstar_.d.at(k)*std::cos(psic.at(k)));
                    trajstar_.psi.push_back(psic.at(k) + trajstar_.deltapsi.at(k));

                    // forces
                    trajstar_.Fyf.push_back(float(Xstaru(0,k)));
                    trajstar_.Fx.push_back(float(Xstaru(1,k)));
                    trajstar_.Fxf.push_back(0.5f*trajstar_.Fx.at(k));
                    trajstar_.Fxr.push_back(0.5f*trajstar_.Fx.at(k));
                }

                //traj_star.X = traj_star.Xc - traj_star.d.*sin(traj_star.psi_c);
                //traj_star.Y = traj_star.Yc + traj_star.d.*cos(traj_star.psi_c);
                //traj_star.psi = traj_star.psi_c + traj_star.deltapsi;




                // publish trajstar
                //trajstar_ = rtisqp_wrapper_.getTrajectory();
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
    ros::Subscriber pathlocal_sub_;
    ros::Subscriber trajhat_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher trajstar_pub_;
    //ACADOvariables acadoVariables;
    //ACADOworkspace acadoWorkspace;
    common::PathLocal pathlocal_;
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
