#include "ros/ros.h"
#include "std_msgs/String.h"
#include <common/PathLocal.h>
#include <common/Obstacles.h>
#include <common/Trajectory.h>
#include <common/State.h>
#include <sstream>
#include "planning/rtisqp_wrapper.h"

class SAARTI
{
public:
    // constructor
    SAARTI(ros::NodeHandle nh){
        nh_ = nh;
        dt = 0.1;
        ros::Rate loop_rate(1/dt);

        // pubs & subs
        trajhat_pub_ = nh.advertise<common::Trajectory>("trajhat",1);
        trajstar_pub_ = nh.advertise<common::Trajectory>("trajstar",1);
        //trajset_pub_ = nh.advertise<common::TrajectorySet>("trajset",1);
        pathlocal_sub_ = nh.subscribe("pathlocal", 1, &SAARTI::pathlocal_callback,this);
        obstacles_sub_ = nh.subscribe("obstacles", 1, &SAARTI::obstacles_callback,this);
        state_sub_ = nh.subscribe("state", 1,  &SAARTI::state_callback,this);

        // init wrapper for rtisqp solver
        rtisqp_wrapper_ = RtisqpWrapper();

        // set weights
        rtisqp_wrapper_.setWeights(Wx,Wu,Wslack);

        // wait until tmp_trajhat, state and path_local is received
        while( !(state_msg_.s > 0) || pathlocal_msg.s.size() == 0 ){
            ROS_INFO_STREAM("waiting for state and path local");
            ros::spinOnce();
            loop_rate.sleep();
        }

        // initialize trajhat last
        common::Trajectory trajstar_last_msg;

        // main loop
        while (ros::ok())
        {
            std::cout << std::endl;
            ROS_INFO_STREAM("main loop");
            ros::Time t_start = ros::Time::now();

            // update adaptive constraints
            rtisqp_wrapper_.setInputConstraints(1.0,1000);

            // set refs
            refs_ = setRefs(2); // 0: tracking(unused todo remove), 1: min s, 2: max s,

            // rollout
            ROS_INFO_STREAM("generating trajectory set");
            rtisqp_wrapper_.computeTrajset(trajset_,state_,pathlocal_msg,4);
            trajset2cart(); // only for visualization, comment out to save time

            // cost eval and select
            int trajhat_idx = trajset_eval_cost(); // error if negative
            planning_util::trajstruct trajhat;
            if(trajhat_idx >= 0){
                trajhat = trajset_.at(uint(trajhat_idx));
            } else {
                ROS_ERROR("no traj selected");
            }
            // todo publish rviz markers (line strip)

            // update current state
            ROS_INFO_STREAM("setting state..");
            rtisqp_wrapper_.setInitialState(state_msg_);

            // set initial guess and shift fwd
            ROS_INFO_STREAM("setting trajstar as initial guess..");
            rtisqp_wrapper_.setInitialGuess(trajhat);
            //rtisqp_wrapper_.shiftStateAndControls();

            // set refs in solver
            ROS_INFO_STREAM("setting reference..");
            rtisqp_wrapper_.setOptReference(trajhat,refs_);

            // set state constraint
            ROS_INFO_STREAM("setting state constraints..");
            std::vector<float> lld = cpp_utils::interp(trajhat.s,pathlocal_msg.s,pathlocal_msg.dub,false);
            std::vector<float> rld = cpp_utils::interp(trajhat.s,pathlocal_msg.s,pathlocal_msg.dlb,false);
            planning_util::posconstrstruct posconstr = rtisqp_wrapper_.setStateConstraints(trajhat,obstacles_msg,lld,rld);

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
            common::Trajectory trajstar_msg;
            std::vector<float> Xstar_s;
            for (uint k = 0; k < N+1; ++k){
                Xstar_s.push_back(float(Xstarx(0,k)));
            }
            std::vector<float> Xc = cpp_utils::interp(Xstar_s,pathlocal_msg.s,pathlocal_msg.X,false);
            std::vector<float> Yc = cpp_utils::interp(Xstar_s,pathlocal_msg.s,pathlocal_msg.Y,false);
            std::vector<float> psic = cpp_utils::interp(Xstar_s,pathlocal_msg.s,pathlocal_msg.psi_c,false);
            trajstar_msg.kappac = cpp_utils::interp(Xstar_s,pathlocal_msg.s,pathlocal_msg.kappa_c,false);

            for (uint k = 0; k < N+1; ++k){
                // states
                trajstar_msg.s.push_back(float(Xstarx(0,k)));
                trajstar_msg.d.push_back(float(Xstarx(1,k)));
                trajstar_msg.deltapsi.push_back(float(Xstarx(2,k)));
                trajstar_msg.psidot.push_back(float(Xstarx(3,k)));
                trajstar_msg.vx.push_back(float(Xstarx(4,k)));
                trajstar_msg.vy.push_back(float(Xstarx(5,k)));

                // cartesian pose
                trajstar_msg.X.push_back(Xc.at(k) - trajstar_msg.d.at(k)*std::sin(psic.at(k)));
                trajstar_msg.Y.push_back(Yc.at(k) + trajstar_msg.d.at(k)*std::cos(psic.at(k)));
                trajstar_msg.psi.push_back(psic.at(k) + trajstar_msg.deltapsi.at(k));

                // forces (we have N+1 states but only N controls)
                if(k < N){
                    trajstar_msg.Fyf.push_back(float(Xstaru(0,k)));
                    trajstar_msg.Fx.push_back(float(Xstaru(1,k)));
                    trajstar_msg.Fxf.push_back(0.5f*trajstar_msg.Fx.at(k));
                    trajstar_msg.Fxr.push_back(0.5f*trajstar_msg.Fx.at(k));
                }
            }

            // publish trajhat message
            common::Trajectory trajhat_msg;
            trajhat_msg.slb = posconstr.slb;
            trajhat_msg.sub = posconstr.sub;
            trajhat_msg.dlb = posconstr.dlb;
            trajhat_msg.dub = posconstr.dub;
            trajhat_msg.header.stamp = ros::Time::now();
            trajhat_pub_.publish(trajhat_msg);

            // publish trajstar
            trajstar_msg.header.stamp = ros::Time::now();
            trajstar_pub_.publish(trajstar_msg);

            // store fwd shifted trajstar for next iteration
            trajstar_last_msg = trajstar_msg;
            rtisqp_wrapper_.shiftTrajectoryFwdSimple(trajstar_last_msg);

            // print loop time
            ros::Duration planningtime = ros::Time::now() - t_start;
            ROS_INFO_STREAM("planningtime = " << planningtime);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    planning_util::refstruct setRefs(int ctrlmode){
        planning_util::refstruct refs;
        switch (ctrlmode) {
        case 1:  // minimize vx (emg brake)
            refs.sref.assign(N+1,state_.s);
            refs.vxref.assign(N+1,0.0);
            break;
        case 2: // maximize s (racing)
            refs.sref.assign(N+1, state_.s + 300);
            refs.vxref.assign(N+1, state_.vx + 25);
            break;
        }
        return refs;
    }

    // computes cartesian coordinates of a trajectory
    void trajset2cart(){
        for (uint i=0;i<trajset_.size();i++) {
            planning_util::trajstruct traj = trajset_.at(i);
            std::vector<float> Xc = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.X,false);
            std::vector<float> Yc = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.Y,false);
            std::vector<float> psic = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.psi_c,false);

            for (uint j=0; j<traj.s.size();j++) {
                //X = Xc - d*sin(psic);
                //Y = Yc + d*cos(psic);
                float X = Xc.at(j) - traj.d.at(j)*sin(psic.at(j));
                float Y = Yc.at(j) + traj.d.at(j)*cos(psic.at(j));
                traj.X.push_back(X);
                traj.Y.push_back(Y);
            }
        }
    }

    // cost evaluation and collision checking of trajset
    int trajset_eval_cost(){
        std::cout << "debug1" << std::endl;
        float mincost = float(Wslack)*10;
        int trajhat_idx = -1;
        for (uint i=0;i<trajset_.size();i++) {
            planning_util::trajstruct traj = trajset_.at(i);
            bool colliding = false;
            bool exitroad = false;
            float cost = 0;
            std::vector<float> dub = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.dub,false);
            std::vector<float> dlb = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.dlb,false);
            std::cout << "debug2" << std::endl;
            for (uint j=0; j<traj.s.size();j++){
                float s = traj.s.at(j);
                float d = traj.d.at(j);
                float vx = traj.vx.at(j);
                // check obstacle (in frenet)
                float dist;
                for (uint k=0; k<obst_.s.size();k++){
                    dist = sqrt( (s-obst_.s.at(k))*(s-obst_.s.at(k)) + (d-obst_.d.at(k))*(d-obst_.d.at(k)) );
                    if(dist < obst_.Rmgn.at(k)){
                        colliding = true;
                    }
                }
                std::cout << "debug3" << std::endl;
                // check outside road (in frenet)
                if((d > dub.at(j)) || d < dlb.at(j) ){
                    exitroad = true;
                }
                std::cout << "debug4" << std::endl;
                // running cost
                std::cout << "refs_.sref.size()" << refs_.sref.size() << std::endl;
                std::cout << "traj.s.size()" << traj.s.size() << std::endl;
                float sref = float(refs_.sref.at(j));
                float vxref = float(refs_.vxref.at(j));
                cost += (sref-s)*float(Wx.at(0))*(sref-s) + (vxref-vx)*float(Wx.at(4))*(vxref-vx);
                std::cout << "debug5" << std::endl;
            }
            if(colliding){
                cost += float(Wslack);
            }
            if(exitroad){
                cost += float(Wslack);
            }
            traj.cost = cost;
            traj.colliding = colliding;
            traj.exitroad = exitroad;

            // keep track of minimum cost traj
            if(cost < mincost){
                mincost = cost;
                trajhat_idx = int(i);
            }
        }
        return trajhat_idx;
    }

    void state_callback(const common::State::ConstPtr& msg){
        state_msg_ = *msg;
        state_.s = msg->s;
        state_.d = msg->d;
        state_.deltapsi = msg->deltapsi;
        state_.psidot = msg->psidot;
        state_.vx = msg->vx;
        state_.vy = msg->vy;
    }

    void pathlocal_callback(const common::PathLocal::ConstPtr& msg){
        pathlocal_msg = *msg;
        pathlocal_.X = msg->X;
        pathlocal_.Y = msg->Y;
        pathlocal_.s = msg->s;
        pathlocal_.psi_c = msg->psi_c;
        pathlocal_.kappa_c = msg->kappa_c;
        pathlocal_.theta_c = msg->theta_c;
        pathlocal_.psi_c = msg->psi_c;
        pathlocal_.dub = msg->dub;
        pathlocal_.dlb = msg->dlb;
    }

    void obstacles_callback(const common::Obstacles::ConstPtr& msg){
        obstacles_msg = *msg;
        obst_.s = msg->s;
        obst_.d = msg->d;
        obst_.R = msg->R;
        obst_.Rmgn = msg->Rmgn;
    }

private:
    double dt;
    ros::NodeHandle nh_;
    ros::Subscriber pathlocal_sub_;
    ros::Subscriber obstacles_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher trajstar_pub_;
    ros::Publisher trajhat_pub_;
    common::PathLocal pathlocal_msg; // todo remove
    common::Obstacles obstacles_msg; // todo remove
    common::State state_msg_; // todo remove
    planning_util::statestruct state_;
    planning_util::pathstruct pathlocal_;
    std::vector<planning_util::trajstruct> trajset_;
    planning_util::obstastruct obst_;
    planning_util::refstruct refs_;
    RtisqpWrapper rtisqp_wrapper_;

    // weights
    std::vector<double> Wx{10.0, 1.0, 1.0, 0.01, 0.01, 0.01};
    std::vector<double> Wu{0.1, 0.1};
    double Wslack = 10000000;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motionplanner");
    ros::NodeHandle nh;
    SAARTI saarti(nh);
    return 0;
}
