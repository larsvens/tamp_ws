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
    void tmp_trajhat_callback(const common::Trajectory::ConstPtr& msg){
        tmp_trajhat_ = *msg;
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
        trajhat_pub_ = nh.advertise<common::Trajectory>("trajhat",1);
        trajstar_pub_ = nh.advertise<common::Trajectory>("trajstar",1);
        pathlocal_sub_ = nh.subscribe("pathlocal", 1, &SAASQP::pathlocal_callback,this);
        tmp_trajhat_sub_  = nh.subscribe("tmp_trajhat", 1, &SAASQP::tmp_trajhat_callback,this);
        state_sub_ = nh.subscribe("state", 1,  &SAASQP::state_callback,this);

        // init wrapper for rtisqp solver
        rtisqp_wrapper_ = RtisqpWrapper();

        // set weights
        std::vector<double> Wx{100.0, 100.0, 100.0, 0.1, 0.1, 0.1};
        std::vector<double> Wu{0.001, 0.001};
        rtisqp_wrapper_.setWeights(Wx,Wu);

        // main loop
        while (ros::ok())
        {
            std::cout << std::endl;
            ROS_INFO_STREAM("main loop");

            // todo: selection step (gives trajhat)

            // check for valid trajhat
            if((tmp_trajhat_.s.size() != 0) && (state_.s > 0) ){

                common::Trajectory trajhat = tmp_trajhat_;

                // update current state
                ROS_INFO_STREAM("setting state..");
                rtisqp_wrapper_.setInitialState(state_);

                // update adaptive constraints
                // todo

                // set initial guess
                ROS_INFO_STREAM("setting initial guess..");
                rtisqp_wrapper_.setInitialGuess(trajhat);

                // set reference
                ROS_INFO_STREAM("setting reference..");
                rtisqp_wrapper_.setReference(trajhat);

                // set state constraint
                ROS_INFO_STREAM("setting state constraints..");
                rtisqp_wrapper_.setStateConstraints(trajhat); // todo add obstacle list as input

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
                common::Trajectory trajstar;
                std::vector<float> Xstar_s;
                for (uint k = 0; k < N; ++k){
                    Xstar_s.push_back(float(Xstarx(0,k)));
                }
                std::vector<float> Xc = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.X,false);
                std::vector<float> Yc = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.Y,false);
                std::vector<float> psic = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.psi_c,false);
                trajstar.kappac = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.kappa_c,false);
                //std::vector<float> kappac = cpp_utils::interp(Xstar_s,pathlocal_.s,pathlocal_.kappa_c,false);

                for (uint k = 0; k < N; ++k){
                    // states
                    trajstar.s.push_back(float(Xstarx(0,k)));
                    trajstar.d.push_back(float(Xstarx(1,k)));
                    trajstar.deltapsi.push_back(float(Xstarx(2,k)));
                    trajstar.psidot.push_back(float(Xstarx(3,k)));
                    trajstar.vx.push_back(float(Xstarx(4,k)));
                    trajstar.vy.push_back(float(Xstarx(5,k)));

                    // cartesian pose
                    trajstar.X.push_back(Xc.at(k) - trajstar.d.at(k)*std::sin(psic.at(k)));
                    trajstar.Y.push_back(Yc.at(k) + trajstar.d.at(k)*std::cos(psic.at(k)));
                    trajstar.psi.push_back(psic.at(k) + trajstar.deltapsi.at(k));

                    // forces
                    trajstar.Fyf.push_back(float(Xstaru(0,k)));
                    trajstar.Fx.push_back(float(Xstaru(1,k)));
                    trajstar.Fxf.push_back(0.5f*trajstar.Fx.at(k));
                    trajstar.Fxr.push_back(0.5f*trajstar.Fx.at(k));
                }

                //traj_star.X = traj_star.Xc - traj_star.d.*sin(traj_star.psi_c);
                //traj_star.Y = traj_star.Yc + traj_star.d.*cos(traj_star.psi_c);
                //traj_star.psi = traj_star.psi_c + traj_star.deltapsi;



                // publish trajhat
                trajhat.header.stamp = ros::Time::now();
                trajhat_pub_.publish(trajhat);

                // publish trajstar
                trajstar.header.stamp = ros::Time::now();
                trajstar_pub_.publish(trajstar);

                // todo shift X and U


            } else {
                ROS_INFO_STREAM("waiting for state and tmp_trajhat");
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
private:
    double dt;
    ros::NodeHandle nh_;
    ros::Subscriber pathlocal_sub_;
    ros::Subscriber tmp_trajhat_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher trajstar_pub_;
    ros::Publisher trajhat_pub_;
    //ACADOvariables acadoVariables;
    //ACADOworkspace acadoWorkspace;
    common::PathLocal pathlocal_;
    //common::Trajectory trajstar_;
    common::Trajectory tmp_trajhat_;
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
