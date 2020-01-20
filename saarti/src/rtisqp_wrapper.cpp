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

void RtisqpWrapper::setWeights(vector<float> Wx, vector<float> WNx, vector<float> Wu, float Wslack){
    // set diagonal elements of acadoVariables.w matrix. Size [Nx+Nu,Nx+Nu] (row major format)
    // running cost
    // states
    acadoVariables.W[0*(NX+NU) + 0] = Wx.at(0); // s
    acadoVariables.W[1*(NX+NU) + 1] = Wx.at(1); // d
    acadoVariables.W[2*(NX+NU) + 2] = Wx.at(2); // deltapsi
    acadoVariables.W[3*(NX+NU) + 3] = Wx.at(3); // psidot
    acadoVariables.W[4*(NX+NU) + 4] = Wx.at(4); // vx
    acadoVariables.W[5*(NX+NU) + 5] = Wx.at(5); // vy
    acadoVariables.W[6*(NX+NU) + 6] = 0.0001f;      // dummy state for slack
    // controls
    acadoVariables.W[7*(NX+NU) + 7] = Wu.at(0); // Fyf
    acadoVariables.W[8*(NX+NU) + 8] = Wu.at(1); // Fxf
    acadoVariables.W[9*(NX+NU) + 9] = Wu.at(2); // Fxr

    acadoVariables.W[10*(NX+NU) + 10] = Wslack;   // slack variable

    // construct eigen matrix to check W
    Eigen::MatrixXd W(NX+NU,NX+NU);
    uint row = 0;
    uint col = 0;
    for (uint i = 0;i<(NX+NU)*(NX+NU);i++) {
        W(row,col) = double(acadoVariables.W[i]);
        col++;
        if(col >= (NX+NU)){
            col = 0;
            row++;
        }
    }

    // terminal cost
    // states
    acadoVariables.WN[0*(NX) + 0] = WNx.at(0); // s
    acadoVariables.WN[1*(NX) + 1] = WNx.at(1); // d
    acadoVariables.WN[2*(NX) + 2] = WNx.at(2); // deltapsi
    acadoVariables.WN[3*(NX) + 3] = WNx.at(3); // psidot
    acadoVariables.WN[4*(NX) + 4] = WNx.at(4); // vx
    acadoVariables.WN[5*(NX) + 5] = WNx.at(5); // vy
    acadoVariables.WN[6*(NX) + 6] = 0.0;      // dummy state for slack

    // construct eigen matrix to check WN
    Eigen::MatrixXd WN(NX,NX);
    row = 0;
    col = 0;
    for (uint i = 0;i<NX*NX;i++) {
        WN(row,col) = double(acadoVariables.WN[i]);
        col++;
        if(col >= (NX)){
            col = 0;
            row++;
        }
    }

    cout << "Setting Weigth matrices:" << endl << "W = " << W  << endl << "WN = " << WN << endl;
}

void RtisqpWrapper::setInitialGuess(planning_util::trajstruct traj){
    // set state trajectory guess
    if(traj.s.size() != N+1){
        throw std::invalid_argument("faulty state trajectory in setInitialGuess");
    }
    for (uint k = 0; k < N + 1; ++k)
    {
        acadoVariables.x[k * NX + 0] = traj.s.at(k); // s
        acadoVariables.x[k * NX + 1] = traj.d.at(k); // d
        acadoVariables.x[k * NX + 2] = traj.deltapsi.at(k); // deltapsi
        acadoVariables.x[k * NX + 3] = traj.psidot.at(k); // psidot
        acadoVariables.x[k * NX + 4] = traj.vx.at(k); // vx
        acadoVariables.x[k * NX + 5] = traj.vy.at(k); // vy
        acadoVariables.x[k * NX + 6] = 0.0;                   // dummy
    }

    // set control sequence guess
    if(traj.Fyf.size() != N){
        throw std::invalid_argument("faulty control sequence in setInitialGuess");
    }
    for (uint k = 0; k < N; ++k){
        acadoVariables.u[k * NU + 0] = traj.Fyf.at(k);
        acadoVariables.u[k * NU + 1] = traj.Fxf.at(k);
        acadoVariables.u[k * NU + 2] = traj.Fxr.at(k);
    }

    // set kappac
    if(traj.kappac.size() != N+1){
        throw std::invalid_argument("faulty kappa_c in setInitialGuess");
    }
    for (uint k = 0; k < N + 1; ++k){
        acadoVariables.od[k * NOD + 0] = traj.kappac.at(k);
    }
}

void RtisqpWrapper::setOptReference(planning_util::trajstruct traj, planning_util::refstruct refs){
    vector<float> sref = refs.sref;
    //vector<float> vxref = refs.vxref;
    // todo rm traj


    // set ref for intermediate states and controls
    for (uint k = 0; k < N; ++k)
    {
        acadoVariables.y[k * NY + 0] = refs.sref.at(k);        // s
        acadoVariables.y[k * NY + 1] = refs.dref_cc;           // d
        acadoVariables.y[k * NY + 2] = 0;                      // deltapsi test
        acadoVariables.y[k * NY + 3] = 0;                      // psidot
        acadoVariables.y[k * NY + 4] = refs.vxref_cc;          // vx
        acadoVariables.y[k * NY + 5] = 0;                      // vy
        acadoVariables.y[k * NY + 6] = 0.0;                    // dummy
        acadoVariables.y[k * NY + 7] = 0;                      // Fyf
        acadoVariables.y[k * NY + 8] = 0;                      // Fxf
        acadoVariables.y[k * NY + 9] = 0;                      // Fxf
        acadoVariables.y[k * NY + 10] = 0.0;                   // slack
    }
    // set ref for final state
    acadoVariables.yN[ 0 ] = refs.sref.at(N);                  // s
    acadoVariables.yN[ 1 ] = refs.dref_cc;                     // d
    acadoVariables.yN[ 2 ] = 0;                                // deltapsi
    acadoVariables.yN[ 3 ] = 0;                                // psidot
    acadoVariables.yN[ 4 ] = refs.vxref_cc;                    // vx
    acadoVariables.yN[ 5 ] = 0;                                // vy
    acadoVariables.yN[ 6 ] = 0;                                // dummy
}

void RtisqpWrapper::setInputConstraints(planning_util::trajstruct traj){
    // note: mu and Fz have been previously set either static or adaptive

    // get max forces front and back
    vector<float> Ffmax;
    vector<float> Frmax;
    for (uint k = 0; k < N ; ++k){
        Ffmax.push_back(traj.mu.at(k)*traj.Fzf.at(k));
        Frmax.push_back(traj.mu.at(k)*traj.Fzr.at(k));
    }

    // set friction constraints front wheel
    uint N1 = 6; // nr of lbA, ubA values associated with friction circle constraints N_vertices/2
    uint N2 = 4; // nr of lbA, ubA values associated with state constraints
    uint Nu = 4; // nr of ctrl inputs
    for (uint k = 0; k < N ; ++k){
        for (uint i = 0; i < N1; i++){
            uint idx = k*(N1+N2) + i;
            acadoVariables.lbAValues[idx] = -Ffmax.at(k)/1000.0f; // scaled
            acadoVariables.ubAValues[idx] = Ffmax.at(k)/1000.0f; // scaled
        }
    }
//    // set drive constraints front wheel
//    for (uint k = 0; k < N ; ++k){
//        uint idx = k*Nu + 0; // first input
//        acadoVariables.ubValues[idx] = 0; // this car is rear wheel drive
//    }

    // set friction and drive constraints for rear wheel
    for (uint k = 0; k < N ; ++k){
        uint idx = k*Nu+2; // third input
        acadoVariables.lbValues[idx] = -Frmax.at(k); // todo for adaptive, include from traj.Fyr value
        acadoVariables.ubValues[idx] = Frmax.at(k);
    }
}

planning_util::posconstrstruct RtisqpWrapper::setStateConstraints(planning_util::trajstruct &traj,
                                                                  planning_util::obstastruct obs,
                                                                  vector<float> lld,
                                                                  vector<float> rld,
                                                                  float w){

    planning_util::posconstrstruct posconstr;

    for (uint k = 0; k < N + 1; ++k)
    {
        // "funnel" - larger deviation allowed farther ahead
        float s_diff_default = 2+(k*0.1f);

        float slb = traj.s.at(k)-s_diff_default;
        float sub = traj.s.at(k)+s_diff_default;

        // initialize at lane boundaries
        float dlb = rld.at(k)+0.5f*w;
        float dub = lld.at(k)-0.5f*w;

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

        // make sure constraint area does not diminish on narrow tracks
        float d_rangemin = 0.5;
        if(dub-dlb < d_rangemin){
            dlb = -0.5f*d_rangemin;
            dub = 0.5f*d_rangemin;
        }
        posconstr.slb.push_back(slb);
        posconstr.sub.push_back(sub);
        posconstr.dlb.push_back(dlb);
        posconstr.dub.push_back(dub);

        // set acadovariable
        acadoVariables.od[k * NOD + 2] = slb;
        acadoVariables.od[k * NOD + 3] = sub;
        acadoVariables.od[k * NOD + 4] = dlb;
        acadoVariables.od[k * NOD + 5] = dub;
    }
    return posconstr;
}

void RtisqpWrapper::setInitialState(planning_util::statestruct state){
    acadoVariables.x0[0] = state.s;
    acadoVariables.x0[1] = state.d;
    acadoVariables.x0[2] = state.deltapsi;
    //acadoVariables.x0[3] = 0.0;//state.psidot;
    acadoVariables.x0[3] = state.psidot;
    acadoVariables.x0[4] = state.vx;
    //acadoVariables.x0[5] = 0.0;
    acadoVariables.x0[5] = state.vy;
    acadoVariables.x0[6] = 0.0; // dummy
}

void RtisqpWrapper::shiftStateAndControls(){
    acado_shiftStates(2, 0, 0);
    acado_shiftControls( 0 );
}

// not used atm
void RtisqpWrapper::shiftTrajectoryFwdSimple(planning_util::trajstruct &traj){


    if(!traj.s.size()){
        throw std::invalid_argument("trying to shift empty trajectory");
    }

    // will shift all states fwd except the final one that is duplicated
    for (uint k = 0; k < N-1; ++k){

        // state
        traj.s.at(k) = traj.s.at(k+1);
        traj.d.at(k) = traj.d.at(k+1);
        traj.deltapsi.at(k) = traj.deltapsi.at(k+1);
        traj.psidot.at(k) = traj.psidot.at(k+1);
        traj.vx.at(k) = traj.vx.at(k+1);
        traj.vy.at(k) = traj.vy.at(k+1);
        // ctrl
        traj.Fyf.at(k) = traj.Fyf.at(k+1);
        traj.Fxf.at(k) = traj.Fxf.at(k+1);
        traj.Fxr.at(k) = traj.Fxr.at(k+1);
        // cartesian pose
        if(!traj.X.size()){
            throw std::invalid_argument("no cartesian poses to shift");
        }
        traj.X.at(k) = traj.X.at(k+1);
        traj.Y.at(k) = traj.Y.at(k+1);
        traj.psi.at(k) = traj.psi.at(k+1);
    }
}

planning_util::trajstruct RtisqpWrapper::shiftTrajectoryByIntegration(planning_util::trajstruct traj,
                                                                      planning_util::statestruct state, // todo rm?
                                                                      planning_util::pathstruct &pathlocal,
                                                                      planning_util::staticparamstruct &sp,
                                                                      int traction_adaptive,
                                                                      float mu_nominal){
    if(!traj.s.size()){
        throw std::invalid_argument("Error in shiftTrajectoryByIntegration: trying to shift empty trajectory");
    }

    // grab ctrl sequence and init state from traj
    planning_util::trajstruct traj_out;
    traj_out.Fyf = traj.Fyf;
    traj_out.Fxf = traj.Fxf;
    traj_out.Fxr = traj.Fxr;

    planning_util::statestruct initstate;

    // if moving, shift u one step fwd before rollout and roll from x_{1|t-1}
    if(state.vx < 1.0f){
        // get state at 0 in traj
        planning_util::state_at_idx_in_traj(traj, initstate, 0);
        // adjust ctrl sequence
//        float factor = 0.95f;
//        for (uint k = 0; k < N; ++k){
//            traj_out.Fyf.at(k) = factor*traj_out.Fyf.at(k);
//            traj_out.Fxf.at(k) = factor*traj_out.Fxf.at(k);
//            traj_out.Fxr.at(k) = factor*traj_out.Fxr.at(k);
//        }
    } else {
        // get state at 1 in traj
        planning_util::state_at_idx_in_traj(traj, initstate, 1);
        // shift ctrl sequence
        for (uint k = 0; k < N-1; ++k){
            traj_out.Fyf.at(k) = traj_out.Fyf.at(k+1);
            traj_out.Fxf.at(k) = traj_out.Fxf.at(k+1);
            traj_out.Fxr.at(k) = traj_out.Fxr.at(k+1);
        }
    }

    RtisqpWrapper::rolloutSingleTraj(traj_out,initstate,pathlocal,sp,traction_adaptive,mu_nominal);
    return traj_out;
}

void RtisqpWrapper::doPreparationStep(){
    acado_preparationStep();
}

int RtisqpWrapper::doFeedbackStep(){
    int status = acado_feedbackStep();
//    cout << "KKT value: " << scientific << acado_getKKT()
//              << ", objective value: " << scientific << acado_getObjective()
//              << endl;
    return status;
}

planning_util::trajstruct RtisqpWrapper::getTrajectory(){
    planning_util::trajstruct traj_out;
    // state
    for (uint k = 0; k < N + 1; ++k){
        traj_out.s.push_back(acadoVariables.x[k * NX + 0]);
        traj_out.d.push_back(acadoVariables.x[k * NX + 1]);
        traj_out.deltapsi.push_back(acadoVariables.x[k * NX + 2]);
        traj_out.psidot.push_back(acadoVariables.x[k * NX + 3]);
        traj_out.vx.push_back(acadoVariables.x[k * NX + 4]);
        traj_out.vy.push_back(acadoVariables.x[k * NX + 5]);
    }
    // ctrl
    for (uint k = 0; k < N; ++k){
        traj_out.Fyf.push_back(acadoVariables.u[k * NU + 0]);
        traj_out.Fxf.push_back(acadoVariables.u[k * NU + 1]);
        traj_out.Fxr.push_back(acadoVariables.u[k * NU + 2]);
    }

    return traj_out;
}

// usage: set (ONLY) control sequence of traj ahead of time, the function will roll dynamics fwd according to those controls
void RtisqpWrapper::rolloutSingleTraj(planning_util::trajstruct  &traj,
                                      planning_util::statestruct &initstate,
                                      planning_util::pathstruct  &pathlocal,
                                      planning_util::staticparamstruct &sp,
                                      int traction_adaptive,
                                      float mu_nominal){

    if (traj.s.size() != 0){
        throw std::invalid_argument("Error in rolloutSingleTraj, state sequence is nonzero at entry");
    }
    if (traj.mu.size() != 0){
        throw std::invalid_argument("Error in rolloutSingleTraj, mu sequence is nonzero at entry");
    }
    if (traj.kappac.size() != 0){
        throw std::invalid_argument("Error in rolloutSingleTraj, kappac sequence is nonzero at entry");
    }
    if (traj.Fyf.size()!=N || traj.Fxf.size()!=N || traj.Fxr.size()!=N ){
        throw std::invalid_argument("Error in rolloutSingleTraj, control sequence has not been set or is wrong");
    }

    // initialize variables
    planning_util::statestruct rollingstate = initstate;

    // initialize vector same size as ACADOvariables.state (see acado_solver.c, acado_initializeNodesByForwardSimulation)
    real_t acadoWSstate[94];
    // get kappac at initstate
    vector<float> kappac_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.kappa_c,false);
    float kappac = kappac_vec.at(0);
    // get mu at initstate
    float mu;
    if(traction_adaptive == 1){
        vector<float> mu_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.mu,false);
        mu = mu_vec.at(0);
    } else { // (traction_adaptive == 0)
        mu = mu_nominal;
    }
    traj.mu.push_back(mu);

    // set approximate ax for Fz computation (todo compute kinematic ax in loop to increase accuracy)
    for (uint k=0;k<N;k++) {
        traj.ax.push_back( (traj.Fxf.at(k)+traj.Fyf.at(k))/sp.m );
    }
    traj.ax.push_back(traj.ax.back()); // ax of lenght N+1

    // set initial state in traj
    planning_util::traj_push_back_state(traj,rollingstate);
    traj.kappac.push_back(kappac);

    // set initial state in integrator (indices from acado_solver.c, acado_initializeNodesByForwardSimulation)
    acadoWSstate[0] = rollingstate.s;
    acadoWSstate[1] = rollingstate.d;
    acadoWSstate[2] = rollingstate.deltapsi;
    acadoWSstate[3] = rollingstate.psidot;
    acadoWSstate[4] = rollingstate.vx;
    acadoWSstate[5] = rollingstate.vy;
    acadoWSstate[5] = 0; // dummyforslack
    acadoWSstate[88] = kappac;
    acadoWSstate[89] = 778178.0f; // Cr tmp!

    // roll loop
    bool integrator_initiated = false;
    for (size_t k=0;k<N;k++) {

        // compute normal forces front and back
        float Fzf;
        float Fzr;
        if(traction_adaptive == 1){
            float theta = 0; // grade angle todo get from pathlocal via traj
            Fzf = (1.0f/(sp.lf+sp.lr))*( sp.m*traj.ax.at(k)*sp.h_cg - sp.m*sp.g*sp.h_cg*std::sin(theta) + sp.m*sp.g*sp.lr*std::cos(theta));
            Fzr = (1.0f/(sp.lf+sp.lr))*(-sp.m*traj.ax.at(k)*sp.h_cg + sp.m*sp.g*sp.h_cg*std::sin(theta) + sp.m*sp.g*sp.lf*std::cos(theta));
        } else { // (traction_adaptive == 0)
            Fzf = (1.0f/(sp.lf+sp.lr))*(sp.m*sp.g*sp.lr);
            Fzr = (1.0f/(sp.lf+sp.lr))*(sp.m*sp.g*sp.lf);
        }
        traj.Fzf.push_back(Fzf);
        traj.Fzr.push_back(Fzr);

        // compute maximum tire forces
        float Ffmax = Fzf*mu;
        float Frmax = Fzr*mu;

        // grab and adjust control inputs according to new force limits
        float Fyf = traj.Fyf.at(k);
        float Fxf = traj.Fxf.at(k);
        float Fxr = traj.Fxr.at(k);

        float Ff = std::sqrt(Fyf*Fyf + Fxf*Fxf);
        if (Ff > Ffmax){
            float r = Ffmax/Ff;
            Fyf *=r;
            Fxf *=r;
        }
        // todo account also for Fyr
        if (Fxr < -Frmax){
            Fxr = -Frmax;
        }
        if(Fxr > Frmax){
            Fxr = Frmax;
        }

        acadoWSstate[84] = Fyf;
        acadoWSstate[85] = Fxf;
        acadoWSstate[86] = Fxr;
        acadoWSstate[87] = 0; //dummyforslack

        // integrate fwd
        if(!integrator_initiated){
            acado_integrate(acadoWSstate,1);
            integrator_initiated = true;
        } else {
            acado_integrate(acadoWSstate,0);
        }

        // extract acadostate
        rollingstate.s = acadoWSstate[0];
        rollingstate.d = acadoWSstate[1];
        rollingstate.deltapsi = acadoWSstate[2];
        rollingstate.psidot = acadoWSstate[3];
        rollingstate.vx = acadoWSstate[4];
        rollingstate.vy = acadoWSstate[5];
        planning_util::traj_push_back_state(traj,rollingstate);

        // update kappac at rollingstate
        vector<float> kappac_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.kappa_c,false);
        kappac = kappac_vec.at(0);
        acadoWSstate[88] = kappac;
        acadoWSstate[89] = 778178.0f; // Cr tmp!
        traj.kappac.push_back(kappac);
        // update mu at rollingstate
        if(traction_adaptive == 1){
            vector<float> mu_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.mu,false);
            mu = mu_vec.at(0);
        } else if (traction_adaptive == 0) {
            mu = mu_nominal;
        }
        traj.mu.push_back(mu);
    }
    if (traj.s.size() != N+1){
        throw std::invalid_argument("Error in rolloutSingleTraj, state sequence is not equal to N+1 at return");
    }
    if (traj.mu.size() != N+1){
        throw std::invalid_argument("Error in rolloutSingleTraj, length of mu estimate is not equal to N+1 at return");
    }
}


/* Description: state space sampling trajectory rollout. Rolls out trajectories using a fast RK4
 * integrator from the acado toolkit. The control input is recomputed at every stage. A vector of
 * references is constructed as [dlb ... dub dlb ... dub], where the first half of the trajset has
 * positive Fx and the second half has negative */
void RtisqpWrapper::computeTrajset(vector<planning_util::trajstruct> &trajset,
                                   planning_util::statestruct &initstate,
                                   planning_util::pathstruct &pathlocal,
                                   planning_util::staticparamstruct & sp,
                                   int traction_adaptive,
                                   float mu_nominal,
                                   float vxref_nominal,
                                   planning_util::refstruct refs,
                                   uint Ntrajs){
    if(Ntrajs % 2 !=0){
        throw "Error in computeTrajset, Ntrajs must be even";
    }

    // set dref
    vector<float> dref;
    // use dlb and dub @ 0 + some margin for dref interval
    float mgn = 1.0; // todo - select in terms of vehicle width
    float dlb = pathlocal.dlb.at(0)-mgn;
    float dub = pathlocal.dub.at(0)+mgn;
    // build dref vector
    dref = cpp_utils::linspace(dlb, dub, Ntrajs);

    // generate trajs
    for (uint i=0;i<Ntrajs;i++) { // loop over trajectory set
        real_t acadoWSstate[94];
        planning_util::statestruct rollingstate = initstate;
        planning_util::ctrlstruct ctrl;

        // roll loop
        auto t1_single = std::chrono::high_resolution_clock::now();
        planning_util::trajstruct traj;
        bool is_initstate = true;
        bool integrator_initiated = false;
        float kappac;

        double t_intgr_tot = 0;
        double t_interp_tot = 0;
        for (size_t k=0;k<N+1;k++) {

            // interp to get kappac at rollingstate.s
            auto t1_interp = std::chrono::high_resolution_clock::now();
            vector<float> kappac_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.kappa_c,false);
            kappac = kappac_vec.at(0);
            acadoWSstate[88] = kappac;
            acadoWSstate[89] = 778178.0f; // Cr tmp!

            // interp timing
            auto t2_interp = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> t_interp = t2_interp - t1_interp;
            t_interp_tot += t_interp.count();

            // set ax for Fz computation from previous cmd
            if(k==0){
                traj.ax.push_back(0);
            } else {
                traj.ax.push_back( (traj.Fxf.at(k-1)+traj.Fyf.at(k-1))/sp.m );
            }

            // compute normal forces front and back
            float Fzf;
            float Fzr;
            float mu;
            if(traction_adaptive == 1){
                float theta = 0; // grade angle todo get from pathlocal via traj
                Fzf = (1.0f/(sp.lf+sp.lr))*( sp.m*traj.ax.at(k)*sp.h_cg - sp.m*sp.g*sp.h_cg*std::sin(theta) + sp.m*sp.g*sp.lr*std::cos(theta));
                Fzr = (1.0f/(sp.lf+sp.lr))*(-sp.m*traj.ax.at(k)*sp.h_cg + sp.m*sp.g*sp.h_cg*std::sin(theta) + sp.m*sp.g*sp.lf*std::cos(theta));
                // interp to get mu from pathlocal at rollingstate.s
                vector<float> mu_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.mu,false);
                mu = mu_vec.at(0);
                //cout << "selecting mu from pathlocal" << endl;
            } else { // (traction_adaptive == 0)
                Fzf = (1.0f/(sp.lf+sp.lr))*(sp.m*sp.g*sp.lr);
                Fzr = (1.0f/(sp.lf+sp.lr))*(sp.m*sp.g*sp.lf);
                mu = mu_nominal;
                //cout << "selecting mu nominal" << endl;
            }

            // compute max horizontal forces front and back
            float Ffmax = mu*Fzf;
            float Frmax = mu*Fzr;

            // get local vxref and vxerror
            float vxref;
            if(refs.ref_mode == 1){ // cc
                vxref = vxref_nominal;
            } else if(refs.ref_mode == 2) { // max s
                vector<float> vxrefv = cpp_utils::interp({rollingstate.s},pathlocal.s,refs.vxref_path,false);
                vxref = vxrefv.at(0)*0.8f; // tmp!
            } else {
                vxref = 0; // minimize s (refmode 0)
            }
            float vxerror = vxref-rollingstate.vx;

            // select Fyf
            // todo: params for tuning separate platforms
            float feedfwd = 0.5f*sp.m*rollingstate.vx*rollingstate.vx*kappac*std::cos(rollingstate.deltapsi) ;
            float derror = dref.at(i) - rollingstate.d;
            float feedback = 3000*derror - 500*rollingstate.deltapsi;
            ctrl.Fyf = feedfwd + feedback;
            //cout << "feedfwd = " << feedfwd << endl;
            //cout << "feedback = " << feedback << endl;
            // saturate at Ffmax
            if(ctrl.Fyf >= Ffmax){
                //cout << "Fyf saturating! FyfRequest = " << ctrl.Fyf << "Ffmax = " << Ffmax << endl;
                ctrl.Fyf = Ffmax;
            }
            if(ctrl.Fyf<=-Ffmax){
                //cout << "Fyf saturating! FyfRequest = " << ctrl.Fyf << "Ffmax = " << Ffmax << endl;
                ctrl.Fyf = -Ffmax;
            }

            // select Fxf
            float Fxfmax = std::sqrt(Ffmax*Ffmax-ctrl.Fyf*ctrl.Fyf);
            if (refs.ref_mode == 0){ // minimize s
                // select max negative Fxf until stop
                ctrl.Fxf = -Fxfmax;
            } else if (refs.ref_mode == 1 || refs.ref_mode == 2){ // maximize s or cc
                if(vxerror > 0){ // accelerating
                    ctrl.Fxf = 0; // rear wheel drive - no drive on front wheel
                } else { // braking
                    ctrl.Fxf = 1000*vxerror;
                    if(ctrl.Fxf<=-Fxfmax){
                        ctrl.Fxf = -Fxfmax;
                    }
                }
            }
            // select Fxr
            float Fxrmax = mu*Fzr;
            if (refs.ref_mode == 0){ // minimize s
                ctrl.Fxr = -Fxrmax;
            } else if (refs.ref_mode == 1 || refs.ref_mode == 2){ // maximize s or cc

                ctrl.Fxr = 1000*vxerror;
                // saturate
                if(ctrl.Fxr >= Frmax){
                    ctrl.Fxr = Frmax;
                }
                if(ctrl.Fxr<=-Frmax){
                    ctrl.Fxr = -Frmax;
                }
            }

            // avoid singularity in dynamics at vx = 0
            if (rollingstate.vx < 0.5f && ctrl.Fxf < 0){
                ctrl.Fxf = 0;
            }
            if (rollingstate.vx < 0.5f && ctrl.Fxr < 0){
                ctrl.Fxr = 0;
            }

            // set controls
            acadoWSstate[84] = ctrl.Fyf;
            acadoWSstate[85] = ctrl.Fxf;
            acadoWSstate[86] = ctrl.Fxr;
            acadoWSstate[87] = 0; // dummyforslack

            if (is_initstate){
                // set init state in integrator

                // state
                acadoWSstate[0] = rollingstate.s;
                acadoWSstate[1] = rollingstate.d;
                acadoWSstate[2] = rollingstate.deltapsi;
                acadoWSstate[3] = rollingstate.psidot;
                acadoWSstate[4] = rollingstate.vx;
                acadoWSstate[5] = rollingstate.vy;
                acadoWSstate[6] = 0.0; // dummy state
                // ctrl
                acadoWSstate[84] = ctrl.Fyf;
                acadoWSstate[85] = ctrl.Fxf;
                acadoWSstate[86] = ctrl.Fxr;
                acadoWSstate[87] = 0.0; // slack
                // onlinedata
                acadoWSstate[88] = kappac;
                acadoWSstate[89] = 778178.0f; // Cr tmp!
                is_initstate = false;
            }

            // integrate fwd
            auto t1_intgr = std::chrono::high_resolution_clock::now();
            if(!integrator_initiated){
                acado_integrate(acadoWSstate,1);
                integrator_initiated = true;
            } else {
                acado_integrate(acadoWSstate,0);
            }
            auto t2_intgr = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> t_intgr = t2_intgr - t1_intgr;
            t_intgr_tot += t_intgr.count();

            // set rollingstate for computing control errors
            rollingstate.s = acadoWSstate[0];
            rollingstate.d = acadoWSstate[1];
            rollingstate.deltapsi = acadoWSstate[2];
            rollingstate.vx = acadoWSstate[4];

            // extract variables from integrator
            traj.s.push_back(acadoWSstate[0]);
            traj.d.push_back(acadoWSstate[1]);
            traj.deltapsi.push_back(acadoWSstate[2]);
            traj.psidot.push_back(acadoWSstate[3]);
            traj.vx.push_back(acadoWSstate[4]);
            traj.vy.push_back(acadoWSstate[5]);

            // tire forces
            traj.Fzf.push_back(Fzf);
            traj.Fzr.push_back(Fzr);
            if(k<N){ // N+1 states and N ctrls
                traj.Fyf.push_back(acadoWSstate[84]);
                traj.Fxf.push_back(acadoWSstate[85]);
                traj.Fxr.push_back(acadoWSstate[86]);
            }
            // push back interpolated vars
            traj.kappac.push_back(kappac);
            traj.mu.push_back(mu);
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
}


