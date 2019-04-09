#include "saasqp/rtisqp_wrapper.h"

// constructor
RtisqpWrapper::RtisqpWrapper()
{

    // Reset all solver memory
    memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
    memset(&acadoVariables, 0, sizeof( acadoVariables ));

    // Initialize the solver
    acado_initializeSolver();

}

bool RtisqpWrapper::setInitialGuess(common::Trajectory traj){
    // set state trajectory guess
    for (uint k = 0; k < N + 1; ++k)
    {
        //std::cout << "k = " << k << "     traj.s.at(k) = " << traj.s.at(k) << std::endl;
        acadoVariables.x[k * NX + 0] = double(traj.s.at(k)); // s
        acadoVariables.x[k * NX + 1] = double(traj.d.at(k)); // d
        acadoVariables.x[k * NX + 2] = double(traj.deltapsi.at(k)); // deltapsi
        acadoVariables.x[k * NX + 3] = double(traj.psidot.at(k)); // psidot
        acadoVariables.x[k * NX + 4] = double(traj.vx.at(k)); // vx
        acadoVariables.x[k * NX + 5] = double(traj.vy.at(k)); // vy
    }
    //std::cout << "state traj guess set sucessfully" << std::endl;

    // set kappac
    for (uint k = 0; k < N + 1; ++k)
    {
        acadoVariables.od[k] = double(traj.kappac.at(k));
    }
    //std::cout << "kappac set sucessfully" << std::endl;

    // set kappac polynomial coeffs
    //std::vector<double> s = cpp_utils::tcast_vector<float,double>(traj.s);
    //std::vector<double> kappac = cpp_utils::tcast_vector<float,double>(traj.kappac);
    //std::vector<double> a = polyfit(s,kappac,3);

    return true;
}

bool RtisqpWrapper::setReference(common::Trajectory traj){
    for (uint k = 0; k < N; ++k)
    {
        acadoVariables.y[k * NY + 0] = double(traj.s.at(k)); // s
        acadoVariables.y[k * NY + 1] = double(traj.d.at(k)); // d
        acadoVariables.y[k * NY + 2] = double(traj.deltapsi.at(k)); // deltapsi
        acadoVariables.y[k * NY + 3] = double(traj.psidot.at(k)); // psidot
        acadoVariables.y[k * NY + 4] = double(traj.vx.at(k)); // vx
        acadoVariables.y[k * NY + 5] = double(traj.vy.at(k)); // vy
        acadoVariables.y[k * NY + 7] = double(traj.Fyf.at(k)); // Fyf
        acadoVariables.y[k * NY + 8] = double(traj.Fx.at(k)); // Fx
    }

    acadoVariables.yN[ 0 ] = double(traj.s.at(N)); // s
    acadoVariables.yN[ 1 ] = double(traj.d.at(N)); // d
    acadoVariables.yN[ 2 ] = double(traj.deltapsi.at(N)); // deltapsi
    acadoVariables.yN[ 3 ] = double(traj.psidot.at(N)); // psidot
    acadoVariables.yN[ 4 ] = double(traj.vx.at(N)); // vx
    acadoVariables.yN[ 5 ] = double(traj.vy.at(N)); // vy

    //std::cout << "reference set sucessfully" << std::endl;

    return true;
}

bool RtisqpWrapper::setInitialState(common::State state){
    acadoVariables.x0[0] = double(state.s);
    acadoVariables.x0[1] = double(state.d);
    acadoVariables.x0[2] = double(state.deltapsi);
    acadoVariables.x0[3] = double(state.psidot);
    acadoVariables.x0[4] = double(state.vx);
    acadoVariables.x0[5] = double(state.vy);
    return true;
}

bool RtisqpWrapper::shiftStateAndControls(){
    acado_shiftStates(2, 0, 0);
    acado_shiftControls( 0 );
    return true;
}

bool RtisqpWrapper::doPreparationStep(){
    acado_preparationStep();
    return true;
}

int RtisqpWrapper::doFeedbackStep(){
    int status = acado_feedbackStep();
    std::cout << "KKT value: " << scientific << acado_getKKT()
              << ", objective value: " << scientific << acado_getObjective()
              << endl;
    return status;
}

common::Trajectory RtisqpWrapper::getTrajectory(){
    common::Trajectory traj;

    // get state variables from acadovariables
    for (uint k = 0; k < N + 1; ++k){
        traj.s.push_back(float(acadoVariables.x[k * NX + 0]));
        traj.d.push_back(float(acadoVariables.x[k * NX + 1]));
        traj.deltapsi.push_back(float(acadoVariables.x[k * NX + 2]));
        traj.psidot.push_back(float(acadoVariables.x[k * NX + 3]));
        traj.vx.push_back(float(acadoVariables.x[k * NX + 4]));
        traj.vy.push_back(float(acadoVariables.x[k * NX + 5]));
    }

    // get X Y by frenet -> cartesian transformation


    return traj;
}

// todo move to common, utils
std::vector<double> polyfit(const std::vector<double> &xv, const std::vector<double> &yv, int order)
{
    Eigen::MatrixXd A(xv.size(), order+1);
    Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
    Eigen::VectorXd result;

    assert(xv.size() == yv.size());
    assert(xv.size() >= order+1);

    // create matrix
    for (size_t i = 0; i < xv.size(); i++)
        for (size_t j = 0; j < order+1; j++)
            A(i, j) = pow(xv.at(i), j);

    // solve for linear least squares fit
    result = A.householderQr().solve(yv_mapped);

    std::vector<double> coeff;
    coeff.resize(order+1);
    for (size_t i = 0; i < order+1; i++)
        coeff[i] = result[i];
    return coeff;
}


