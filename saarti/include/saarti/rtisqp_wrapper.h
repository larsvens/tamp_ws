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

// define variables
#define NX         ACADO_NX	/* number of differential states */
#define NXA        ACADO_NXA	/* number of alg. states */
#define NU         ACADO_NU	/* number of control inputs */
#define N          ACADO_N	/* number of control intervals */
#define NY	       ACADO_NY	/* number of references, nodes 0..N - 1 */
#define NYN	       ACADO_NYN
#define NOD        ACADO_NOD
//#define NUM_STEPS  100		/* number of simulation steps */
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
    void setInitialGuess(containers::trajstruct);
    void setInitialState(containers::statestruct);
    void setOptReference(containers::trajstruct, containers::refstruct refs);
    void setInputConstraints(containers::trajstruct traj);
    containers::posconstrstruct setStateConstraints(containers::trajstruct &traj,
                                                    containers::obstastruct obs,
                                                    vector<float> lld,
                                                    vector<float> rld,
                                                    containers::staticparamstruct &sp);
    void shiftStateAndControls();
    void shiftTrajectoryFwdSimple(containers::trajstruct &traj);
    containers::trajstruct shiftTrajectoryByIntegration(containers::trajstruct traj,
                                                        containers::statestruct state,
                                                        containers::pathstruct &pathlocal,
                                                        containers::staticparamstruct &sp,
                                                        int adaptive_mode,
                                                        float mu_static);
    void doPreparationStep();
    int  doFeedbackStep();
    Eigen::MatrixXd getStateTrajectory();
    Eigen::MatrixXd getControlTrajectory();
    containers::trajstruct getTrajectory();
    void computeTrajset(vector<containers::trajstruct> &trajset,
                        containers::statestruct &initstate,
                        containers::pathstruct &pathlocal,
                        containers::staticparamstruct & sp,
                        int adaptive_mode,
                        float mu_nominal,
                        float vxref_nominal,
                        containers::refstruct ref_mode,
                        uint Ntrajs);
    void rolloutSingleTraj(containers::trajstruct  &traj,
                           containers::statestruct &initstate,
                           containers::pathstruct  &pathlocal,
                           containers::staticparamstruct &sp,
                           int adaptive_mode,
                           float mu_static);

    void setIntegratorState(real_t *acadoWSstate,
                            containers::statestruct state,
                            containers::ctrlstruct ctrl,
                            float kappac);

};

#endif // RTISQP_WRAPPER_H
