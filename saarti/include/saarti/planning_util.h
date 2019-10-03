#ifndef PLANNING_UTIL_H
#define PLANNING_UTIL_H

#include <cstdlib>
#include <vector>

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
    std::vector<float> Fx;
    // od
    std::vector<float> kappac;
    // cartesian pose
    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> psi;
    // eval
    float cost;
    bool colliding;
    bool exitroad;
};

// state vector representation
struct statestruct{
    // state
    float s;
    float d;
    float deltapsi;
    float psidot;
    float vx;
    float vy;
};

// ctrl vector representation
struct ctrlstruct{
    // state
    float Fyf;
    float Fx;
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
    std::vector<float> sref;
    std::vector<float> vxref;
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


}; // END NAMESPACE

#endif // PLANNING_UTIL_H
