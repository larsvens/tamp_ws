#pragma once

#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <cmath>

using namespace std;

// acado codegen
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

// ros
#include "common/Trajectory.h"
#include "common/State.h"
#include "common/cpp_utils.h"

// define variables
#define NX         ACADO_NX	/* number of differential states */
#define NXA        ACADO_NXA	/* number of alg. states */
#define NU         ACADO_NU	/* number of control inputs */
#define N          ACADO_N	/* number of control intervals */
#define NY	   ACADO_NY	/* number of references, nodes 0..N - 1 */
#define NYN	   ACADO_NYN
#define NUM_STEPS  100		/* number of simulation steps */
#define VERBOSE    1		/* show iterations: 1, silent: 0 */

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class RtisqpWrapper
{
public:

    // constructors
    RtisqpWrapper();

    // functions
    bool setInitialGuess(common::Trajectory);
    bool setInitialState(common::State);
    bool setReference(common::Trajectory);
    bool shiftStateAndControls();
    bool doPreparationStep();
    int  doFeedbackStep();
    std::vector<double> polyfit(const std::vector<double>, const std::vector<double>, int order);
    common::Trajectory getTrajectory();
};

