//#pragma once

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <cmath>

// acado codegen
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

// util
#include "planning_util.h"
#include <common/interp.h>
#include "common/cpp_utils.h"


//# include "rtisqp_wrapper.h" // need acado integrator

class Rollout
{
public:

    // constructors
    Rollout();

    // functions
    bool computeTrajset(std::vector<planning_util::trajstruct> &trajset,
                        planning_util::statestruct &state,
                        planning_util::pathstruct &pathlocal,
                        uint N,
                        uint Ntrajs,
                        uint ctrl_mode);
};
