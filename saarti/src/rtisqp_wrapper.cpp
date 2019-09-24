#include "saarti/rtisqp_wrapper.h"

// constructor
RtisqpWrapper::RtisqpWrapper()
{

    // Reset all solver memory
    memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
    memset(&acadoVariables, 0, sizeof( acadoVariables ));

    // Initialize the solver
    acado_initializeSolver();

}

bool RtisqpWrapper::setWeights(vector<double> Wx, vector<double> Wu, double Wslack){
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

    cout << "Setting Weigth matrix W: " << endl << W  << endl;
    return true;
}

bool RtisqpWrapper::setInitialGuess(planning_util::trajstruct traj){
    // set state trajectory guess
    if(traj.s.size() != N+1){
        throw std::invalid_argument("faulty state trajectory in setInitialGuess");
    }
    for (uint k = 0; k < N + 1; ++k)
    {
        acadoVariables.x[k * NX + 0] = double(traj.s.at(k)); // s
        acadoVariables.x[k * NX + 1] = double(traj.d.at(k)); // d
        acadoVariables.x[k * NX + 2] = double(traj.deltapsi.at(k)); // deltapsi
        acadoVariables.x[k * NX + 3] = double(traj.psidot.at(k)); // psidot
        acadoVariables.x[k * NX + 4] = double(traj.vx.at(k)); // vx
        acadoVariables.x[k * NX + 5] = double(traj.vy.at(k)); // vy
        acadoVariables.x[k * NX + 6] = 0.0;                   // dummy
    }

    // set kappac
    if(traj.kappac.size() != N+1){
        throw std::invalid_argument("faulty kappa_c in setInitialGuess");
    }
    for (uint k = 0; k < N + 1; ++k)
    {
        acadoVariables.od[k * NOD + 0] = double(traj.kappac.at(k));
    }

    return true;
}

bool RtisqpWrapper::setOptReference(planning_util::trajstruct traj, planning_util::refstruct refs){
    vector<float> sref = refs.sref;
    vector<float> vxref = refs.vxref;

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
    //cout << "size of acadoVariables.ubAValues: " << N_ubA << endl;
    for (uint k = 0; k < N ; ++k){
        for (uint j = 0; j < N_ineq; j++){ // edit the N_ineq first values of N_per_k
            uint idx = k*N_per_k + j;
            //cout <<"at idx: " << idx << ", ubAval = " << acadoVariables.ubAValues[idx] << endl;
            acadoVariables.ubAValues[idx] = mu; // tmp just to test effect (affine input const after state const?)
        }
    }
    return true;
}

planning_util::posconstrstruct RtisqpWrapper::setStateConstraints(planning_util::trajstruct &traj,
                                                                  planning_util::obstastruct obs,
                                                                  vector<float> lld,
                                                                  vector<float> rld){

    planning_util::posconstrstruct posconstr;

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

        posconstr.slb.push_back(slb);
        posconstr.sub.push_back(sub);
        posconstr.dlb.push_back(dlb);
        posconstr.dub.push_back(dub);

        // set acadovariable
        acadoVariables.od[k * NOD + 1] = double(slb);
        acadoVariables.od[k * NOD + 2] = double(sub);
        acadoVariables.od[k * NOD + 3] = double(dlb);
        acadoVariables.od[k * NOD + 4] = double(dub);
    }
    return posconstr;
}

bool RtisqpWrapper::setInitialState(planning_util::statestruct state){
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

bool RtisqpWrapper::shiftTrajectoryFwdSimple(planning_util::trajstruct &traj){
    // will shift all states fwd except the final one that is duplicated
    for (uint k = 0; k < N-1; ++k){
        if(!traj.s.size()){
            throw std::invalid_argument("trying to shift empty trajectory");
        }
        // state
        traj.s.at(k) = traj.s.at(k+1);
        traj.d.at(k) = traj.d.at(k+1);
        traj.deltapsi.at(k) = traj.deltapsi.at(k+1);
        traj.psidot.at(k) = traj.psidot.at(k+1);
        traj.vx.at(k) = traj.vx.at(k+1);
        traj.vy.at(k) = traj.vy.at(k+1);
        // ctrl
        traj.Fyf.at(k) = traj.Fyf.at(k+1);
        traj.Fx.at(k) = traj.Fx.at(k+1);
        // cartesian pose
        if(!traj.X.size()){
            throw std::invalid_argument("no cartesian poses to shift");
        }
        traj.X.at(k) = traj.X.at(k+1);
        traj.Y.at(k) = traj.Y.at(k+1);
        traj.psi.at(k) = traj.psi.at(k+1);
    }
    return true;
}

//bool RtisqpWrapper::shiftTrajectoryByIntegration(planning_util::trajstruct &traj, planning_util::statestruct &state){
//    real_t acadoWSstate[85];
//    planning_util::ctrlstruct ctrl;
//    bool is_initstate = true;
//    for (size_t k=0;k<N+1;k++) {
//        ctrl.Fyf = traj.Fyf.at(k);
//        ctrl.Fx  = traj.Fx.at(k);
//        if (is_initstate){
//            RtisqpWrapper::setIntegratorState(acadoWSstate,state,ctrl,kappac);
//        }
//    }
//    return true;
//}

bool RtisqpWrapper::doPreparationStep(){
    acado_preparationStep();
    return true;
}

int RtisqpWrapper::doFeedbackStep(){
    int status = acado_feedbackStep();
//    cout << "KKT value: " << scientific << acado_getKKT()
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

planning_util::trajstruct RtisqpWrapper::getTrajectory(){
    planning_util::trajstruct traj_out;
    // state
    for (uint k = 0; k < N + 1; ++k){
        traj_out.s.push_back(float(acadoVariables.x[k * NX + 0])); // s
        traj_out.d.push_back(float(acadoVariables.x[k * NX + 1]));
        traj_out.deltapsi.push_back(float(acadoVariables.x[k * NX + 2]));
        traj_out.psidot.push_back(float(acadoVariables.x[k * NX + 3]));
        traj_out.vx.push_back(float(acadoVariables.x[k * NX + 4]));
        traj_out.vy.push_back(float(acadoVariables.x[k * NX + 5]));
    }
    // ctrl
    for (uint k = 0; k < N; ++k){
        traj_out.Fyf.push_back(float(acadoVariables.u[k * NU + 0]));
        traj_out.Fx.push_back(float(acadoVariables.u[k * NU + 1]));
    }
    return traj_out;
}

bool RtisqpWrapper::setIntegratorState(real_t *acadoWSstate,
                                       planning_util::statestruct state,
                                       planning_util::ctrlstruct ctrl,
                                       float kappac){
    // state
    acadoWSstate[0] = double(state.s);
    acadoWSstate[1] = double(state.d);
    acadoWSstate[2] = double(state.deltapsi);
    acadoWSstate[3] = double(state.psidot);
    acadoWSstate[4] = double(state.vx);
    acadoWSstate[5] = double(state.vy);
    acadoWSstate[6] = 0.0; // dummy state
    // ctrl
    acadoWSstate[77] = double(ctrl.Fyf);
    acadoWSstate[78] = double(ctrl.Fx);
    acadoWSstate[79] = 0.0; // slack varreal_t[85]
    // onlinedata
    acadoWSstate[80] = double(kappac);
    acadoWSstate[81] = 0.0; // slb
    acadoWSstate[82] = 0.0; // sub
    acadoWSstate[83] = 0.0; // dlb
    acadoWSstate[84] = 0.0; // dub

    return true;
}


void rolloutSingleTraj(planning_util::statestruct &initstate,
                       planning_util::pathstruct &pathlocal){

}

/* Description: state space sampling trajectory rollout. Rolls out trajectories using a fast RK4
 * integrator from the acado toolkit. The control input is recomputed at every stage. A vector of
 * references is constructed as [dlb ... dub dlb ... dub], where the first half of the trajset has
 * positive Fx and the second half has negative */
bool RtisqpWrapper::computeTrajset(vector<planning_util::trajstruct> &trajset,
                                   planning_util::statestruct &state,
                                   planning_util::pathstruct &pathlocal,
                                   uint Ntrajs){
    if(Ntrajs % 2 !=0){
        throw "Error, Ntrajs must be even";
    }

    // todo input Fh_MAX
    float Fyfmax = 500;
    float Fxmax = 1000;

    // generate reference vectors
    vector<float> dref;
    // use dlb and dub @ 0 + some margin for dref interval
    float mgn = 1.0;
    float dlb = pathlocal.dlb.at(0)-mgn;
    float dub = pathlocal.dub.at(0)+mgn;
    // build dref vector
    dref = cpp_utils::linspace(dlb, dub, Ntrajs/2);
    vector<float> dref_copy = dref;
    dref.insert(dref.end(), dref_copy.begin(), dref_copy.end());

    // generate trajs
    for (uint i=0;i<Ntrajs;i++) {
        real_t acadoWSstate[85];
        planning_util::statestruct rollingstate;
        rollingstate.s = state.s;
        rollingstate.d = state.d;
        planning_util::ctrlstruct ctrl;

        // integrator loop
        auto t1_single = std::chrono::high_resolution_clock::now();
        planning_util::trajstruct traj;
        bool is_initstate = true;
        bool integrator_initiated = false;
        float kappac;
        double t_intgr_tot = 0;
        double t_interp_tot = 0;
        for (size_t j=0;j<N+1;j++) {
            // compute control input
            float derror = dref.at(i) - rollingstate.d;
            ctrl.Fyf = 500*derror - 1500*rollingstate.deltapsi;
            // saturate at Fyfmax
            if(ctrl.Fyf >= Fyfmax){
                ctrl.Fyf = Fyfmax;
            }
            if(ctrl.Fyf<=-Fyfmax){
                ctrl.Fyf = -Fyfmax;
            }
            // Fx from standard parametric function of ellipse
            float t = std::asin(ctrl.Fyf/Fyfmax);
            ctrl.Fx = Fxmax*std::cos(t);

            // if in second half of dref, use the negative value of Fx
            if (i >= N/2){
                ctrl.Fx = -ctrl.Fx;
            }
            acadoWSstate[77] = double(ctrl.Fyf);
            acadoWSstate[78] = double(ctrl.Fx);

            // compute kappac at rollingstate.s
            auto t1_interp = std::chrono::high_resolution_clock::now();
            vector<float> kappac_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.kappa_c,false);
            kappac = kappac_vec.at(0);
            acadoWSstate[80] = double(kappac);
            auto t2_interp = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> t_interp = t2_interp - t1_interp;
            t_interp_tot += t_interp.count();

            if (is_initstate){
                // set init state in integrator
                RtisqpWrapper::setIntegratorState(acadoWSstate,state,ctrl,kappac);
                is_initstate = false;
            }
            else {
                // integrate fwd
                auto t1_intgr = std::chrono::high_resolution_clock::now();
                if(!integrator_initiated){
                    acado_integrate(acadoWSstate,1);
                } else {
                    acado_integrate(acadoWSstate,0);
                }
                auto t2_intgr = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> t_intgr = t2_intgr - t1_intgr;
                t_intgr_tot += t_intgr.count();
            }

            // set s and d of the state that rolls fwd
            rollingstate.s = float(acadoWSstate[0]);
            rollingstate.d = float(acadoWSstate[1]);
            rollingstate.deltapsi = float(acadoWSstate[2]);

            // extract variables from integrator
            traj.s.push_back(float(acadoWSstate[0]));
            traj.d.push_back(float(acadoWSstate[1]));
            traj.deltapsi.push_back(float(acadoWSstate[2]));
            traj.psidot.push_back(float(acadoWSstate[3]));
            traj.vx.push_back(float(acadoWSstate[4]));
            traj.vy.push_back(float(acadoWSstate[5]));

            traj.kappac.push_back(kappac);
            if(j<N){ // N+1 states and N ctrls
                traj.Fyf.push_back(float(acadoWSstate[77]));
                traj.Fx.push_back(float(acadoWSstate[78]));
            }
        }
        trajset.push_back(traj);
        auto t2_single = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> t_single_rollout = t2_single - t1_single;
        bool printtimings = false;
        if(printtimings){
            cout << "Rollout timings:" << endl;
            cout << "single rollout took " << t_single_rollout.count() << " ms " << endl;
            cout << "integration took " << t_intgr_tot << " ms " << endl;
            cout << "interp took " << t_interp_tot << " ms " << endl;
        }
    }

    return true;
}

