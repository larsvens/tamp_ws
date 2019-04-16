// Keep this general. Clean out ros stuff and problem specific things


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

bool RtisqpWrapper::setWeights(std::vector<double> Wx, std::vector<double> Wu){
    // set diagonal elements of acadoVariables.w matrix. Size [Nx+Nu,Nx+Nu] (row major format)
    // states
    acadoVariables.W[0*(NX+NU) + 0] = Wx.at(0);
    acadoVariables.W[1*(NX+NU) + 1] = Wx.at(1);
    acadoVariables.W[2*(NX+NU) + 2] = Wx.at(2);
    acadoVariables.W[3*(NX+NU) + 3] = Wx.at(3);
    acadoVariables.W[4*(NX+NU) + 4] = Wx.at(4);
    acadoVariables.W[5*(NX+NU) + 5] = Wx.at(5);
    // controls
    acadoVariables.W[6*(NX+NU) + 6] = Wu.at(0);
    acadoVariables.W[7*(NX+NU) + 7] = Wu.at(1);

    // construct eigen matrix to check
    Eigen::MatrixXd W(NX+NU,NX+NU);
    uint row = 0;
    uint col = 0;
    for (uint i = 0;i<(NX+NU)*(NX+NU);i++) {
        W(row,col) = acadoVariables.W[i];
        col++;
        if(col >= (NX+NU)){
            col = 0;
            row++;
        }
    }

    std::cout << "Setting Weigth matrix W: " << std::endl << W  << std::endl;
    return true;
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

    // set kappac
    for (uint k = 0; k < N + 1; ++k)
    {
        acadoVariables.od[k * NOD + 0] = double(traj.kappac.at(k));
    }

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

bool RtisqpWrapper::setStateConstraints(common::Trajectory &traj){
    for (uint k = 0; k < N + 1; ++k)
    {
        float s_diff = 2;
        float d_diff = 1;

        // todo adjust diffs for obstacles



        // add lb and ub to traj struct for visualization
        traj.slb.push_back(traj.s.at(k) - s_diff);
        traj.sub.push_back(traj.s.at(k) + s_diff);
        traj.dlb.push_back(traj.d.at(k) - d_diff);
        traj.dub.push_back(traj.d.at(k) + d_diff);

        // set acadovariable
        acadoVariables.od[k * NOD + 1] = double(traj.slb.at(k));
        acadoVariables.od[k * NOD + 2] = double(traj.sub.at(k));
        acadoVariables.od[k * NOD + 3] = double(traj.dlb.at(k));
        acadoVariables.od[k * NOD + 4] = double(traj.dub.at(k));
    }
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

Eigen::MatrixXd RtisqpWrapper::getStateTrajectory(){
    // row: state, column: time
    // todo: put as loops s.t. not problem dependent
    Eigen::MatrixXd Xstarx(NX,N+1);
    for (uint k = 0; k < N + 1; ++k){
        Xstarx(0,k) = acadoVariables.x[k * NX + 0]; // s
        Xstarx(1,k) = acadoVariables.x[k * NX + 1];
        Xstarx(2,k) = acadoVariables.x[k * NX + 2];
        Xstarx(3,k) = acadoVariables.x[k * NX + 3];
        Xstarx(4,k) = acadoVariables.x[k * NX + 4];
        Xstarx(5,k) = acadoVariables.x[k * NX + 5];
    }
    return Xstarx;
}

Eigen::MatrixXd RtisqpWrapper::getControlTrajectory(){
    // row: control variable, column: time
    Eigen::MatrixXd Xstaru(NU,N);
    for (uint k = 0; k < N; ++k){
        Xstaru(0,k) = acadoVariables.u[k * NU + 0];
        Xstaru(1,k) = acadoVariables.u[k * NU + 1];
    }
    return Xstaru;
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


