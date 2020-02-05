#ifndef PLANNING_UTIL_H
#define PLANNING_UTIL_H

#include <cstdlib>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

namespace planning_util {

/*
 * Variable containers
 */

// trajectory representation
struct trajstruct{
    // state
    std::vector<float> s;
    std::vector<float> d;
    std::vector<float> deltapsi;
    std::vector<float> psidot;
    std::vector<float> vx;
    std::vector<float> vy;
    // ctrl
    std::vector<float> Fyf;
    std::vector<float> Fxf;
    std::vector<float> Fxr;
    // additional forces
    std::vector<float> Fyr;
    std::vector<float> Fzf;
    std::vector<float> Fzr;
    // od
    std::vector<float> kappac;
    // cartesian pose
    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> psi;
    // frictionestimate
    std::vector<float> mu;
    std::vector<float> Cf;
    std::vector<float> Cr;
    // misc
    std::vector<float> ax;
    // eval
    float cost;
    bool colliding;
    bool exitroad;
};

// state vector representation
struct statestruct{
    float s;
    float d;
    float deltapsi;
    float psidot;
    float vx;
    float vy;
};

// ctrl vector representation
struct ctrlstruct{
    float Fyf;
    float Fxf;
    float Fxr;
};

// pathlocal representation
struct pathstruct{
    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> s;
    std::vector<float> psi_c;
    std::vector<float> kappa_c;
    std::vector<float> theta_c;
    std::vector<float> phi_c;
    std::vector<float> dub;
    std::vector<float> dlb;
    std::vector<float> mu;
};

// obstacle representation
struct obstastruct{
    // state
    std::vector<float> s;
    std::vector<float> d;
    std::vector<float> R;
    std::vector<float> Rmgn;
    std::vector<float> X;
    std::vector<float> Y;
};

// position constraint container
struct posconstrstruct{
    std::vector<float> slb;
    std::vector<float> sub;
    std::vector<float> dlb;
    std::vector<float> dub;
};

// reference container
struct refstruct{
    int ref_mode;
    std::vector<float> sref;
    std::vector<float> vxref_path;
    float vxref_cc;
    float dref_cc;
};

// static params container
struct staticparamstruct{
    float m;
    float g;
    float lf;
    float lr;
    float h_cg;
    float l_width;
};

/*
 * Helper functions
 */

void traj_push_back_state(trajstruct &traj, statestruct &state){
    traj.s.push_back(state.s);
    traj.d.push_back(state.d);
    traj.deltapsi.push_back(state.deltapsi);
    traj.psidot.push_back(state.psidot);
    traj.vx.push_back(state.vx);
    traj.vy.push_back(state.vy);
}

void state_at_idx_in_traj(trajstruct &traj, statestruct &state, uint idx){
    if (idx>=traj.s.size() ){
        throw "Error in state_at_idx_in_traj, idx out of range";
    }
    state.s = traj.s.at(idx);
    state.d = traj.d.at(idx);
    state.deltapsi = traj.deltapsi.at(idx);
    state.psidot = traj.psidot.at(idx);
    state.vx = traj.vx.at(idx);
    state.vy = traj.vy.at(idx);
}

float get_cornering_stiffness(float mu, float Fz){
    float B, C, D; //, E;
    // todo adjust thresholds

    if(0.0f <= mu && mu <0.3f){ // ice
        B = 4.0f;
        C = 2.0f;
        D = mu;
        //E = 1.0f;
    } else if (0.3f <= mu && mu <0.5f) { // snow
        B = 5.0f;
        C = 2.0f;
        D = mu;
        //E = 1.0f;
    } else if (0.5f <= mu && mu <0.9f) { // wet
        B = 12.0f;
        C = 2.3f;
        D = mu;
        //E = 1.0f;
    } else if (0.9f <= mu && mu <2.5f) { // dry
        B = 10.0f;
        C = 1.9f;
        D = mu;
        //E = 0.97f;
    } else {
        throw std::invalid_argument("faulty mu value in get_cornering_stiffness");
    }
    return B*C*D*Fz; // Rajamani
}

Eigen::MatrixXf get_vehicle_corners(float X, float Y, float psi, float lf, float lr, float w){
    Eigen::MatrixXf R(2,2);
    Eigen::MatrixXf dims(5,2);
    Eigen::MatrixXf Xoffset(5,2);
    Eigen::MatrixXf Yoffset(5,2);
    Eigen::MatrixXf corners(5,2);

    R << std::cos(psi), std::sin(psi),
        -std::sin(psi), std::cos(psi);

    dims << lf, w/2,
            -lr, w/2,
            -lr, -w/2,
            lf, -w/2,
            lf, w/2;

    // rotate
    corners = dims*R;

    // add position offset
    Xoffset << X, 0.0f,
               X, 0.0f,
               X, 0.0f,
               X, 0.0f,
               X, 0.0f;
    Yoffset << 0.0f, Y,
               0.0f, Y,
               0.0f, Y,
               0.0f, Y,
               0.0f, Y;

    return corners + Xoffset + Yoffset;
}


}; // END NAMESPACE

#endif // PLANNING_UTIL_H
