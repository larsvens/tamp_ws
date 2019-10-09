#ifndef RTISQP_WRAPPER_H
#define RTISQP_WRAPPER_H

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

// define variables
#define NX         ACADO_NX	/* number of differential states */
#define NXA        ACADO_NXA	/* number of alg. states */
#define NU         ACADO_NU	/* number of control inputs */
#define N          ACADO_N	/* number of control intervals */
#define NY	       ACADO_NY	/* number of references, nodes 0..N - 1 */
#define NYN	       ACADO_NYN
#define NOD        ACADO_NOD
#define NUM_STEPS  100		/* number of simulation steps */
#define VERBOSE    1		/* show iterations: 1, silent: 0 */

// timing
#include <chrono>

// namespaces
using std::vector;
using std::cout;
using std::endl;

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class RtisqpWrapper
{
public:

    // constructors
    RtisqpWrapper();

    // functions
    void setWeights(vector<float>, vector<float>, vector<float>, float);
    void setInitialGuess(planning_util::trajstruct);
    void setInitialState(planning_util::statestruct);
    void setOptReference(planning_util::trajstruct, planning_util::refstruct refs);
    void setInputConstraints(float mu, float Fzf);
    planning_util::posconstrstruct setStateConstraints(planning_util::trajstruct &traj,
                                                       planning_util::obstastruct obs,
                                                       vector<float> lld,
                                                       vector<float> rld,
                                                       float w);
    void shiftStateAndControls();
    void shiftTrajectoryFwdSimple(planning_util::trajstruct &traj);
    planning_util::trajstruct shiftTrajectoryByIntegration(planning_util::trajstruct &traj,
                                                           planning_util::statestruct state,
                                                           planning_util::pathstruct &pathlocal,
                                                           planning_util::staticparamstruct &sp);
    void doPreparationStep();
    int  doFeedbackStep();
    Eigen::MatrixXd getStateTrajectory();
    Eigen::MatrixXd getControlTrajectory();
    planning_util::trajstruct getTrajectory();
    void computeTrajset(vector<planning_util::trajstruct> &trajset,
                        planning_util::statestruct &state,
                        planning_util::pathstruct &pathlocal,
                        uint Ntrajs);
    void rolloutSingleTraj(planning_util::trajstruct  &traj,
                           planning_util::statestruct &initstate,
                           planning_util::pathstruct  &pathlocal,
                           planning_util::staticparamstruct &sp);

    void setIntegratorState(real_t *acadoWSstate,
                            planning_util::statestruct state,
                            planning_util::ctrlstruct ctrl,
                            float kappac);

};

#endif // RTISQP_WRAPPER_H
