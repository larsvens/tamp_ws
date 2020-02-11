#ifndef PLANNING_UTIL_H
#define PLANNING_UTIL_H

#include <cstdlib>
#include <vector>
#include <iostream>
#include "containers.h"
//#include <eigen3/Eigen/Dense>
#include <math.h>
#include <common/interp.h>
#include "common/cpp_utils.h"


using std::vector;

namespace planning_util {

void traj_push_back_state(containers::trajstruct &traj, containers::statestruct &state);
void state_at_idx_in_traj(containers::trajstruct &traj, containers::statestruct &state, uint idx);
void angle_to_interval(std::vector<float> &psi);
std::vector<float> angle_to_continous(std::vector<float> &psi);
void sd_pts2cart(vector<float> &Xout, vector<float> &Yout, vector<float> &s, vector<float> &d, containers::pathstruct &pathlocal);
void traj2cart(containers::trajstruct &traj, containers::pathstruct &pathlocal);
void trajset2cart(vector<containers::trajstruct> &trajset, containers::pathstruct &pathlocal);


float get_cornering_stiffness(float mu, float Fz);

}; // END NAMESPACE

#endif // PLANNING_UTIL_H
