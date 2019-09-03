// Keep this general. Clean out ros stuff and problem specific things


#include "planning/rtisqp_wrapper.h"



// constructor
RtisqpWrapper::RtisqpWrapper()
{

    // Reset all solver memory
    memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
    memset(&acadoVariables, 0, sizeof( acadoVariables ));

    // Initialize the solver
    acado_initializeSolver();

}

bool RtisqpWrapper::setWeights(std::vector<double> Wx, std::vector<double> Wu, double Wslack){
    // set diagonal elements of acadoVariables.w matrix. Size [Nx+Nu,Nx+Nu] (row major format)
    // states
    acadoVariables.W[0*(NX+NU) + 0] = Wx.at(0); // s
    acadoVariables.W[1*(NX+NU) + 1] = Wx.at(1); // d
    acadoVariables.W[2*(NX+NU) + 2] = Wx.at(2); // deltapsi
    acadoVariables.W[3*(NX+NU) + 3] = Wx.at(3); // psidot
    acadoVariables.W[4*(NX+NU) + 4] = Wx.at(4); // vx
    acadoVariables.W[5*(NX+NU) + 5] = Wx.at(5); // vy
    acadoVariables.W[6*(NX+NU) + 6] = 0.0;      // dummy state for slack
    // controls
    acadoVariables.W[7*(NX+NU) + 7] = Wu.at(0); // Fyf
    acadoVariables.W[8*(NX+NU) + 8] = Wu.at(1); // Fx
    acadoVariables.W[9*(NX+NU) + 9] = Wslack;   // slack variable

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
        acadoVariables.x[k * NX + 0] = double(traj.s.at(k)); // s
        acadoVariables.x[k * NX + 1] = double(traj.d.at(k)); // d
        acadoVariables.x[k * NX + 2] = double(traj.deltapsi.at(k)); // deltapsi
        acadoVariables.x[k * NX + 3] = double(traj.psidot.at(k)); // psidot
        acadoVariables.x[k * NX + 4] = double(traj.vx.at(k)); // vx
        acadoVariables.x[k * NX + 5] = double(traj.vy.at(k)); // vy
        acadoVariables.x[k * NX + 6] = 0.0;                   // dummy
        //std::cout << "debug, k = " << k << std::endl;
    }

    // set kappac
    for (uint k = 0; k < N + 1; ++k)
    {
        acadoVariables.od[k * NOD + 0] = double(traj.kappac.at(k));
        //std::cout << "debug kappa, k = " << k << std::endl;
    }

    return true;
}

bool RtisqpWrapper::setReference(common::Trajectory traj, int ctrlmode){
    std::vector<float> sref;
    std::vector<float> vxref;
    switch (ctrlmode) {
    case 0: // tracking
        //std::cout << "case 0, ctrlmode = " << ctrlmode << std::endl;
        sref = traj.s;
        vxref = traj.vx;
        break;
    case 1: // minimize s
        //std::cout << "case 1, ctrlmode = " << ctrlmode << std::endl;

        //minimize s
        //sref.assign(N+1, traj.s.at(0));
        //vxref = traj.vx;

        // minimize vx
        sref = traj.s;
        vxref.assign(N+1,0.0);
        break;
    case 2: // maximize s
        //std::cout << "case 2, ctrlmode = " << ctrlmode << std::endl;
        sref.assign(N+1, traj.s.at(0) + 300);
        vxref = traj.vx;
        break;
    }

    // set ref for intermediate states
    for (uint k = 0; k < N; ++k)
    {
        acadoVariables.y[k * NY + 0] = double(sref.at(k));           // s
        acadoVariables.y[k * NY + 1] = double(traj.d.at(k));         // d
        acadoVariables.y[k * NY + 2] = double(traj.deltapsi.at(k));  // deltapsi
        acadoVariables.y[k * NY + 3] = double(traj.psidot.at(k));    // psidot
        acadoVariables.y[k * NY + 4] = double(vxref.at(k));          // vx
        acadoVariables.y[k * NY + 5] = double(traj.vy.at(k));        // vy
        acadoVariables.y[k * NY + 6] = 0.0;                          // dummy
        acadoVariables.y[k * NY + 7] = double(traj.Fyf.at(k));       // Fyf
        acadoVariables.y[k * NY + 8] = double(traj.Fx.at(k));        // Fx
        acadoVariables.y[k * NY + 9] = 0.0;                          // slack
    }
    // set ref for final state
    acadoVariables.yN[ 0 ] = double(sref.at(N));           // s
    acadoVariables.yN[ 1 ] = double(traj.d.at(N));         // d
    acadoVariables.yN[ 2 ] = double(traj.deltapsi.at(N));  // deltapsi
    acadoVariables.yN[ 3 ] = double(traj.psidot.at(N));    // psidot
    acadoVariables.yN[ 4 ] = double(traj.vx.at(N));        // vx
    acadoVariables.yN[ 5 ] = double(traj.vy.at(N));        // vy
    acadoVariables.yN[ 6 ] = 0;                            // dummy
    return true;
}

bool RtisqpWrapper::setInputConstraints(double mu, double Fzf){
    uint N_ineq = 8; // nr of inequalities, todo read from file!
    uint N_ubA = sizeof(acadoVariables.ubAValues) / sizeof(*acadoVariables.ubAValues);
    uint N_per_k = N_ubA/N;
    std::cout << "size of acadoVariables.ubAValues: " << N_ubA << std::endl;
    for (uint k = 0; k < N ; ++k){
        for (uint j = 0; j < N_ineq; j++){ // edit the N_ineq first values of N_per_k
            uint idx = k*N_per_k + j;
            //std::cout <<"at idx: " << idx << ", ubAval = " << acadoVariables.ubAValues[idx] << std::endl;
            acadoVariables.ubAValues[idx] = mu; // tmp just to test effect (affine input const after state const?)
        }
    }
}


//for (uint k = 0; k < N + 1; ++k){
//    Xstarx(0,k) = acadoVariables.x[k * NX + 0]; // s
//    Xstarx(1,k) = acadoVariables.x[k * NX + 1];
//    Xstarx(2,k) = acadoVariables.x[k * NX + 2];
//    Xstarx(3,k) = acadoVariables.x[k * NX + 3];
//    Xstarx(4,k) = acadoVariables.x[k * NX + 4];
//    Xstarx(5,k) = acadoVariables.x[k * NX + 5];
//}

bool RtisqpWrapper::setStateConstraints(common::Trajectory &traj,
                                        common::Obstacles obs,
                                        std::vector<float> lld,
                                        std::vector<float> rld){

    std::vector <float> slb_vec;
    std::vector <float> sub_vec;
    std::vector <float> dlb_vec;
    std::vector <float> dub_vec;

    for (uint k = 0; k < N + 1; ++k)
    {
        // "funnel" - larger deviation allowed farther ahead
        float s_diff_default = 2+(k*0.20f);
        float d_diff_default = 2+(k*0.04f);

        float slb = traj.s.at(k)-s_diff_default;
        float sub = traj.s.at(k)+s_diff_default;
        float dlb = traj.d.at(k)-d_diff_default;
        float dub = traj.d.at(k)+d_diff_default;

        // adjust lbs and ubs for obstacles
        uint Nobs = uint(obs.s.size());
        for (uint i = 0; i<Nobs; i++) {
            float sobs = obs.s.at(i);
            float dobs = obs.d.at(i);
            float Rmgn = obs.Rmgn.at(i);
            if (slb - Rmgn <= sobs && sobs <= sub + Rmgn){
                if(dobs >= traj.d.at(k)){ // obs left of vehicle, adjust dub
                    dub = std::min(dub, dobs-Rmgn);
                    dub = std::max(dub,traj.d.at(k)); // s.t. d-interval is nonzero
                } else{ // obs right of vehicle, adjust dlb
                    dlb = std::max(dlb, dobs+Rmgn);
                    dlb = std::min(dlb, traj.d.at(k)); // s.t. d-interval is nonzero
                }
            }
        }

        // adjust for lane boundaries
        if(dub > lld.at(k)){ // left lane boundary
            dub = lld.at(k);
            if(dub-dlb < d_diff_default){
                dlb = dub-d_diff_default;
            }
        }
        if(dlb < rld.at(k)){ // right lane boundary
            dlb = rld.at(k);
            if(dub-dlb < d_diff_default){
                dub = dlb + d_diff_default;
            }
        }

        // add lb and ub to traj struct for visualization
        slb_vec.push_back(slb);
        sub_vec.push_back(sub);
        dlb_vec.push_back(dlb);
        dub_vec.push_back(dub);

        traj.slb = slb_vec;
        traj.sub = sub_vec;
        traj.dlb = dlb_vec;
        traj.dub = dub_vec;

        // set acadovariable
        acadoVariables.od[k * NOD + 1] = double(slb);
        acadoVariables.od[k * NOD + 2] = double(sub);
        acadoVariables.od[k * NOD + 3] = double(dlb);
        acadoVariables.od[k * NOD + 4] = double(dub);
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
    acadoVariables.x0[6] = 0.0; // dummy
    return true;
}

bool RtisqpWrapper::shiftStateAndControls(){
    acado_shiftStates(2, 0, 0);
    acado_shiftControls( 0 );
    return true;
}

bool RtisqpWrapper::shiftTrajectoryFwdSimple(common::Trajectory &traj){
    // will shift all states fwd except the final one that is duplicated
    for (uint k = 0; k < N-1; ++k){
        traj.X.at(k) = traj.X.at(k+1);
        traj.Y.at(k) = traj.Y.at(k+1);
        traj.psi.at(k) = traj.psi.at(k+1);
        traj.s.at(k) = traj.s.at(k+1);
        traj.d.at(k) = traj.d.at(k+1);
        traj.deltapsi.at(k) = traj.deltapsi.at(k+1);
        traj.psidot.at(k) = traj.psidot.at(k+1);
        traj.vx.at(k) = traj.vx.at(k+1);
        traj.vy.at(k) = traj.vy.at(k+1);
        //traj.ax.at(k) = traj.ax.at(k+1);
        //traj.ay.at(k) = traj.ay.at(k+1);
        //traj.Fxf.at(k) = traj.Fxf.at(k+1);
        traj.Fyf.at(k) = traj.Fyf.at(k+1);
        //traj.Fxr.at(k) = traj.Fxr.at(k+1);
        //traj.Fyr.at(k) = traj.Fyr.at(k+1);
        traj.Fx.at(k) = traj.Fx.at(k+1);
    }
}

bool RtisqpWrapper::doPreparationStep(){
    acado_preparationStep();
    return true;
}

int RtisqpWrapper::doFeedbackStep(){
    int status = acado_feedbackStep();
//    std::cout << "KKT value: " << scientific << acado_getKKT()
//              << ", objective value: " << scientific << acado_getObjective()
//              << endl;
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

bool computeTrajset(std::vector<trajstruct> &trajset, int Ntrajs){
    // TODO!

    return true;
}

