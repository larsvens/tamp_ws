#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
#include <common/Path.h>
#include <common/Obstacles.h>
#include <common/Trajectory.h>
#include <common/State.h>
#include <sstream>
#include "saarti/rtisqp_wrapper.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "geometry_msgs/PolygonStamped.h"

#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant" // supress warning at ros prints

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
        pathlocal_sub_ = nh.subscribe("pathlocal", 1, &SAARTI::pathlocal_callback,this);
        obstacles_sub_ = nh.subscribe("obstacles", 1, &SAARTI::obstacles_callback,this);
        state_sub_ = nh.subscribe("state", 1,  &SAARTI::state_callback,this);
        // visualization
        trajhat_vis_pub_ = nh.advertise<nav_msgs::Path>("trajhat_vis",1);
        trajstar_vis_pub_ = nh.advertise<nav_msgs::Path>("trajstar_vis",1);
        trajset_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trajset_vis",1);
        posconstr_vis_pub_ = nh.advertise<jsk_recognition_msgs::PolygonArray>("posconstr_vis",1);

        // init wrapper for rtisqp solver
        rtisqp_wrapper_ = RtisqpWrapper();

        // set weights
        rtisqp_wrapper_.setWeights(Wx,Wu,Wslack);

        // wait until tmp_trajhat, state and path_local is received
        while( (state_.s <= 0) || pathlocal_.s.size() == 0 ){
            ROS_INFO_STREAM("waiting for state and path local");
            ros::spinOnce();
            loop_rate.sleep();
        }

        // initialize trajhat last
        planning_util::trajstruct trajstar_last;

        // main loop
        while (ros::ok())
        {
            std::cout << std::endl;
            ROS_INFO_STREAM("main_ loop_");
            ros::Time t_start = ros::Time::now();

            // update adaptive constraints
            rtisqp_wrapper_.setInputConstraints(1.0,1000);

            // set refs
            refs_ = setRefs(ctrl_mode_); // 0: tracking(unused todo remove), 1: min s, 2: max s,

            // rollout
            ROS_INFO_STREAM("generating trajectory set");
            trajset_.clear();
            int Nsamples = 16;
            rtisqp_wrapper_.computeTrajset(trajset_,state_,pathlocal_,uint(Nsamples),ctrl_mode_);
            if(trajstar_last.s.size()>0){ // append trajstar last
                trajset_.push_back(trajstar_last);
            }
            trajset2cart(); // only for visualization, comment out to save time
            trajset2ma();

            // cost eval and select
            int trajhat_idx = trajset_eval_cost(); // error if negative
            std::cout << "TMP! forcing trajhat idx for now" << std::endl;

//            // TMP UNTIL FINISHED FSSIM INTEGRATION
//            std::cout << " trajset_.size(): " << trajset_.size() << std::endl;
//            //trajhat_idx = 0;
//            if(trajstar_last.s.size()>0){
//                trajhat_idx = Nsamples + 1; // select trajstarlast
//            }
//            else {
//                trajhat_idx = 8; // select the single sampled traj
//            }

            planning_util::trajstruct trajhat;
            if(trajhat_idx >= 0){
                trajhat = trajset_.at(uint(trajhat_idx));
            } else {
                ROS_ERROR("no traj selected");
            }
            ROS_INFO_STREAM("trajhat_idx = " << trajhat_idx);
            ROS_INFO_STREAM("trajhat.cost = " << trajhat.cost);
            nav_msgs::Path p_trajhat = traj2navpath(trajhat);

            // update current state
            ROS_INFO_STREAM("setting state..");
            rtisqp_wrapper_.setInitialState(state_);

            // set initial guess and shift fwd
            ROS_INFO_STREAM("setting initial guess..");
            rtisqp_wrapper_.setInitialGuess(trajhat);
            //print_obj(trajhat);
            //rtisqp_wrapper_.shiftStateAndControls();

            // set refs in solver
            ROS_INFO_STREAM("setting reference..");
            rtisqp_wrapper_.setOptReference(trajhat,refs_);

            // set state constraint
            ROS_INFO_STREAM("setting state constraints..");
            std::vector<float> lld = cpp_utils::interp(trajhat.s,pathlocal_.s,pathlocal_.dub,false);
            std::vector<float> rld = cpp_utils::interp(trajhat.s,pathlocal_.s,pathlocal_.dlb,false);
            planning_util::posconstrstruct posconstr = rtisqp_wrapper_.setStateConstraints(trajhat,obst_,lld,rld);
            // visualize state constraint
            jsk_recognition_msgs::PolygonArray polarr = stateconstr2polarr(posconstr);

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

            // extract trajstar from acado
            planning_util::trajstruct trajstar = rtisqp_wrapper_.getTrajectory();
            traj2cart(trajstar);
            nav_msgs::Path p_trajstar = traj2navpath(trajstar);

            // publish trajhat
            common::Trajectory trajhat_msg = traj2msg(trajhat);
            trajhat_msg.slb = posconstr.slb;
            trajhat_msg.sub = posconstr.sub;
            trajhat_msg.dlb = posconstr.dlb;
            trajhat_msg.dub = posconstr.dub;
            trajhat_msg.header.stamp = ros::Time::now();
            trajhat_pub_.publish(trajhat_msg);

            // publish trajstar
            common::Trajectory trajstar_msg = traj2msg(trajstar);
            trajstar_msg.header.stamp = ros::Time::now();
            trajstar_pub_.publish(trajstar_msg);

            // publish visualization msgs
            trajhat_vis_pub_.publish(p_trajhat);
            trajstar_vis_pub_.publish(p_trajstar);
            trajset_vis_pub_.publish(trajset_ma_);
            posconstr_vis_pub_.publish(polarr);

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

    // print size of object for debugging
    void print_obj(planning_util::trajstruct traj){
        // state
        std::cout << "length of s: " << traj.s.size() << std::endl;
        std::cout << "length of d: " << traj.d.size() << std::endl;
        std::cout << "length of deltapsi: " << traj.deltapsi.size() << std::endl;
        std::cout << "length of psidot: " << traj.psidot.size() << std::endl;
        std::cout << "length of vx: " << traj.vx.size() << std::endl;
        std::cout << "length of vy: " << traj.vy.size() << std::endl;
        // control
        std::cout << "length of Fyf: " << traj.Fyf.size() << std::endl;
        std::cout << "length of Fx: " << traj.Fx.size() << std::endl;
        // cartesian pose
        std::cout << "length of X: " << traj.X.size() << std::endl;
        std::cout << "length of Y: " << traj.Y.size() << std::endl;
        std::cout << "length of psi: " << traj.psi.size() << std::endl;
    }

    // sets refs to be used in rollout and optimization
    planning_util::refstruct setRefs(uint ctrlmode){
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
    void traj2cart(planning_util::trajstruct &traj){
        if(!traj.s.size()){
            ROS_ERROR("traj2cart on traj of 0 length");
        }
        else {
            std::vector<float> Xc = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.X,false);
            std::vector<float> Yc = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.Y,false);
            std::vector<float> psic = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.psi_c,false);
            for (uint j=0; j<traj.s.size();j++) {
                if(isnan(traj.s.at(j))){
                    ROS_ERROR("trajectory has nans");
                }
                // X = Xc - d*sin(psic);
                // Y = Yc + d*cos(psic);
                // psi = deltapsi + psic;
                float X = Xc.at(j) - traj.d.at(j)*sin(psic.at(j));
                float Y = Yc.at(j) + traj.d.at(j)*cos(psic.at(j));
                float psi = traj.deltapsi.at(j) + psic.at(j);
                traj.X.push_back(X);
                traj.Y.push_back(Y);
                traj.psi.push_back(psi);
            }
            traj.kappac = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.kappa_c,false);
        }
    }

    // computes cartesian coordinates of a trajectory set
    void trajset2cart(){
        for (uint i=0;i<trajset_.size();i++) {
            traj2cart(trajset_.at(i));
        }
    }

    // computes cartesian coordinates of a set of s,d pts
    void sd_pts2cart(std::vector<float> &s, std::vector<float> &d, std::vector<float> &Xout, std::vector<float> &Yout){
        std::vector<float> Xc = cpp_utils::interp(s,pathlocal_.s,pathlocal_.X,false);
        std::vector<float> Yc = cpp_utils::interp(s,pathlocal_.s,pathlocal_.Y,false);
        std::vector<float> psic = cpp_utils::interp(s,pathlocal_.s,pathlocal_.psi_c,false);
        for (uint j=0; j<s.size();j++) {
            // X = Xc - d*sin(psic);
            // Y = Yc + d*cos(psic);
            // psi = deltapsi + psic;
            float X = Xc.at(j) - d.at(j)*sin(psic.at(j));
            float Y = Yc.at(j) + d.at(j)*cos(psic.at(j));
            Xout.push_back(X);
            Yout.push_back(Y);
        }
    }

    // cost evaluation and collision checking of trajset
    int trajset_eval_cost(){
        float mincost = float(Wslack)*10;
        int trajhat_idx = -1;
        for (uint i=0;i<trajset_.size();i++) {
            planning_util::trajstruct traj = trajset_.at(i);
            bool colliding = false;
            bool exitroad = false;
            float cost = 0;
            std::vector<float> dub = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.dub,false);
            std::vector<float> dlb = cpp_utils::interp(traj.s,pathlocal_.s,pathlocal_.dlb,false);
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
                // check outside road (in frenet)
                if((d > dub.at(j)) || d < dlb.at(j) ){
                    exitroad = true;
                }
                // running cost
                float sref = float(refs_.sref.at(j));
                float vxref = float(refs_.vxref.at(j));
                //std::cout << "sref before rc add = " << sref << std::endl;
                //std::cout << "vxref before rc add = " << vxref << std::endl;
                //std::cout << "s before rc add = " << s << std::endl;
                //std::cout << "vx before rc add = " << vx << std::endl;
                //std::cout << "cost before rc add = " << cost << std::endl;
                cost += (sref-s)*float(Wx.at(0))*(sref-s) + (vxref-vx)*float(Wx.at(4))*(vxref-vx);
                //std::cout << "cost after rc add = " << cost << std::endl;
            }
            if(colliding){
                cost += float(Wslack);
                //cost = float(Wslack);
            }
            if(exitroad){
                cost += float(Wslack);
                //cost = float(Wslack);
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

    common::Trajectory traj2msg(planning_util::trajstruct traj){
        common::Trajectory trajmsg;
        // state
        trajmsg.s = traj.s;
        trajmsg.d = traj.d;
        trajmsg.deltapsi = traj.deltapsi;
        trajmsg.psidot = traj.psidot;
        trajmsg.vx = traj.vx;
        trajmsg.vy = traj.vy;
        // ctrl
        trajmsg.Fyf = traj.Fyf;
        trajmsg.Fx = traj.Fx;
        // cart pose
        trajmsg.X = traj.X;
        trajmsg.Y = traj.Y;
        trajmsg.psi = traj.psi;

        return trajmsg;
    }

    // represent traj as navmsgs path for visualization
    nav_msgs::Path traj2navpath(planning_util::trajstruct traj){
        nav_msgs::Path p;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "map";
        for (uint i=0;i<traj.s.size();i++){
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = double(traj.X.at(i));
            pose.pose.position.y = double(traj.Y.at(i));
            // todo heading
            p.poses.push_back(pose);
        }
        return p;
    }

    // fill trajset marker array for visualization
    void trajset2ma(){
        trajset_ma_.markers.clear();
        int count = 0;
        for (uint i=0; i<trajset_.size(); i++) {
            planning_util::trajstruct traj = trajset_.at(i);
            for (uint j=0; j<traj.s.size();j++) {
                visualization_msgs::Marker m;
                m.header.stamp = ros::Time::now();
                m.header.frame_id = "map";
                m.id = count;
                m.pose.position.x = double(traj.X.at(j));
                m.pose.position.y = double(traj.Y.at(j));
                // style
                m.type = m.CUBE;
                m.scale.x = 0.1;
                m.scale.y = 0.1;
                m.scale.z = 0.01;
                m.color.a = 1.0;
                m.color.r = 0.0;
                m.color.g = 1.0;
                m.color.b = 0.0;
                trajset_ma_.markers.push_back(m);
                count++;
            }
        }
    }

    // create visualization obj for state constraints
    jsk_recognition_msgs::PolygonArray stateconstr2polarr(planning_util::posconstrstruct pc){
        jsk_recognition_msgs::PolygonArray polarr;
        polarr.header.stamp = ros::Time::now();
        polarr.header.frame_id = "map";
        for (uint i=0;i<pc.dlb.size();i++){
            geometry_msgs::PolygonStamped poly;
            poly.header.stamp = ros::Time::now();
            poly.header.frame_id = "map";
            std::vector<float> s{pc.slb.at(i),pc.sub.at(i),pc.sub.at(i),pc.slb.at(i)};
            std::vector<float> d{pc.dub.at(i),pc.dub.at(i),pc.dlb.at(i),pc.dlb.at(i)};
            std::vector<float> X;
            std::vector<float> Y;
            sd_pts2cart(s, d, X, Y);
            for (uint j=0;j<4;j++){
                geometry_msgs::Point32 pt;
                pt.x = X.at(j);
                pt.y = Y.at(j);
                poly.polygon.points.push_back(pt);
            }
            polarr.polygons.push_back(poly);
        }
        return polarr;
    }

    void state_callback(const common::State::ConstPtr& msg){
        state_.s = msg->s;
        state_.d = msg->d;
        state_.deltapsi = msg->deltapsi;
        state_.psidot = msg->psidot;
        state_.vx = msg->vx;
        state_.vy = msg->vy;

        // curvilinear dynamics breaks when vx == 0
        if (state_.vx <= 0.1f){
            state_.vx = 0.1f;
        }
    }

    void pathlocal_callback(const common::Path::ConstPtr& msg){
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
    ros::Publisher trajset_vis_pub_;
    ros::Publisher trajhat_vis_pub_;
    ros::Publisher trajstar_vis_pub_;
    ros::Publisher posconstr_vis_pub_;
    planning_util::statestruct state_;
    planning_util::pathstruct pathlocal_;
    std::vector<planning_util::trajstruct> trajset_;
    planning_util::obstastruct obst_;
    planning_util::refstruct refs_;
    RtisqpWrapper rtisqp_wrapper_;
    visualization_msgs::MarkerArray trajset_ma_;

    // modes
    uint ctrl_mode_ = 2; // 0: tracking(unused todo remove), 1: min s, 2: max s,

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
