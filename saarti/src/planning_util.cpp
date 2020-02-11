#include "saarti/planning_util.h"


void planning_util::traj_push_back_state(trajstruct &traj, statestruct &state){
    traj.s.push_back(state.s);
    traj.d.push_back(state.d);
    traj.deltapsi.push_back(state.deltapsi);
    traj.psidot.push_back(state.psidot);
    traj.vx.push_back(state.vx);
    traj.vy.push_back(state.vy);
}

void planning_util::state_at_idx_in_traj(trajstruct &traj, statestruct &state, uint idx){
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

float planning_util::get_cornering_stiffness(float mu, float Fz){
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
