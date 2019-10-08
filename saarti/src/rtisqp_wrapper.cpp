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
    acadoVariables.W[6*(NX+NU) + 6] = 0.0;      // dummy state for slack
    // controls
    acadoVariables.W[7*(NX+NU) + 7] = Wu.at(0); // Fyf
    acadoVariables.W[8*(NX+NU) + 8] = Wu.at(1); // Fx
    acadoVariables.W[9*(NX+NU) + 9] = Wslack;   // slack variable

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

    // set kappac
    if(traj.kappac.size() != N+1){
        throw std::invalid_argument("faulty kappa_c in setInitialGuess");
    }
    for (uint k = 0; k < N + 1; ++k)
    {
        acadoVariables.od[k * NOD + 0] = traj.kappac.at(k);
    }
}

void RtisqpWrapper::setOptReference(planning_util::trajstruct traj, planning_util::refstruct refs){
    vector<float> sref = refs.sref;
    vector<float> vxref = refs.vxref;

    // TODO: rm vx-ref

    // set ref for intermediate states
    for (uint k = 0; k < N; ++k)
    {
        acadoVariables.y[k * NY + 0] = sref.at(k);           // s
        acadoVariables.y[k * NY + 1] = traj.d.at(k);         // d
        acadoVariables.y[k * NY + 2] = traj.deltapsi.at(k);  // deltapsi
        acadoVariables.y[k * NY + 3] = traj.psidot.at(k);    // psidot
        acadoVariables.y[k * NY + 4] = traj.vx.at(k);        // vx
        acadoVariables.y[k * NY + 5] = traj.vy.at(k);        // vy
        acadoVariables.y[k * NY + 6] = 0.0;                  // dummy
        acadoVariables.y[k * NY + 7] = traj.Fyf.at(k);       // Fyf
        acadoVariables.y[k * NY + 8] = traj.Fx.at(k);        // Fx
        acadoVariables.y[k * NY + 9] = 0.0;                  // slack
    }
    // set ref for final state
    acadoVariables.yN[ 0 ] = sref.at(N);           // s
    acadoVariables.yN[ 1 ] = traj.d.at(N);         // d
    acadoVariables.yN[ 2 ] = traj.deltapsi.at(N);  // deltapsi
    acadoVariables.yN[ 3 ] = traj.psidot.at(N);    // psidot
    acadoVariables.yN[ 4 ] = traj.vx.at(N);        // vx
    acadoVariables.yN[ 5 ] = traj.vy.at(N);        // vy
    acadoVariables.yN[ 6 ] = 0;                    // dummy
}

void RtisqpWrapper::setInputConstraints(float mu, float Fzf){
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
        float d_diff_default = 2+(k*0.04f);

        float slb = traj.s.at(k)-s_diff_default;
        float sub = traj.s.at(k)+s_diff_default;

        //float dlb = traj.d.at(k)-d_diff_default;
        //float dub = traj.d.at(k)+d_diff_default;

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

        // adjust for lane boundaries (including vehicle width)

//        if(dub > lld.at(k) - 0.5f*w){ // left lane boundary
//            dub = lld.at(k) - 0.5f*w;


////            if(dub-dlb < d_diff_default){
////                dlb = dub-d_diff_default;
////            }
//        }
//        if(dlb < rld.at(k) + 0.5f*w){ // right lane boundary
//            dlb = rld.at(k) + 0.5f*w;


////            if(dub-dlb < d_diff_default){
////                dub = dlb + d_diff_default;
////            }
//        }

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
        acadoVariables.od[k * NOD + 1] = slb;
        acadoVariables.od[k * NOD + 2] = sub;
        acadoVariables.od[k * NOD + 3] = dlb;
        acadoVariables.od[k * NOD + 4] = dub;
    }
    return posconstr;
}

void RtisqpWrapper::setInitialState(planning_util::statestruct state){
    acadoVariables.x0[0] = state.s;
    acadoVariables.x0[1] = state.d;
    acadoVariables.x0[2] = state.deltapsi;
    acadoVariables.x0[3] = state.psidot;
    acadoVariables.x0[4] = state.vx;
    acadoVariables.x0[5] = state.vy;
    acadoVariables.x0[6] = 0.0; // dummy
}

void RtisqpWrapper::shiftStateAndControls(){
    acado_shiftStates(2, 0, 0);
    acado_shiftControls( 0 );
}

// TODO REMOVE
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
        traj.Fx.at(k) = traj.Fx.at(k+1);
        // cartesian pose
        if(!traj.X.size()){
            throw std::invalid_argument("no cartesian poses to shift");
        }
        traj.X.at(k) = traj.X.at(k+1);
        traj.Y.at(k) = traj.Y.at(k+1);
        traj.psi.at(k) = traj.psi.at(k+1);
    }
}

planning_util::trajstruct RtisqpWrapper::shiftTrajectoryByIntegration(planning_util::trajstruct &traj,
                                                                      planning_util::statestruct &state,
                                                                      planning_util::pathstruct &pathlocal,
                                                                      planning_util::staticparamstruct &sp){
    if(!traj.s.size()){
        throw std::invalid_argument("Error in shiftTrajectoryByIntegration: trying to shift empty trajectory");
    }

    // grab ctrl sequence from trajstar last and integrate fwd from state
    planning_util::trajstruct traj_out;
    traj_out.Fyf = traj.Fyf;
    traj_out.Fx = traj.Fx;
    // if moving, shift u one step fwd before rollout
    if(state.vx > 1.0f){
        for (uint k = 0; k < N-1; ++k){
            traj_out.Fyf.at(k) = traj_out.Fyf.at(k+1);
            traj_out.Fx.at(k) = traj_out.Fx.at(k+1);
        }
    }

    RtisqpWrapper::rolloutSingleTraj(traj_out,state,pathlocal,sp);
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
        traj_out.s.push_back(acadoVariables.x[k * NX + 0]); // s
        traj_out.d.push_back(acadoVariables.x[k * NX + 1]);
        traj_out.deltapsi.push_back(acadoVariables.x[k * NX + 2]);
        traj_out.psidot.push_back(acadoVariables.x[k * NX + 3]);
        traj_out.vx.push_back(acadoVariables.x[k * NX + 4]);
        traj_out.vy.push_back(acadoVariables.x[k * NX + 5]);
    }
    // ctrl
    for (uint k = 0; k < N; ++k){
        traj_out.Fyf.push_back(acadoVariables.u[k * NU + 0]);
        traj_out.Fx.push_back(acadoVariables.u[k * NU + 1]);
    }
    return traj_out;
}

// usage: set control sequence of traj ahead of time, the function will roll dynamics fwd according to those controls
void RtisqpWrapper::rolloutSingleTraj(planning_util::trajstruct  &traj,
                                      planning_util::statestruct &initstate,
                                      planning_util::pathstruct  &pathlocal,
                                      planning_util::staticparamstruct &sp){

    if (traj.s.size() != 0){
        throw std::invalid_argument("Error in rolloutSingleTraj, state sequence is nonzero at entry");
    }
    if (traj.Fyf.size() != N){
        throw std::invalid_argument("Error in rolloutSingleTraj, control sequence has not been set or is wrong");
    }

    // initialize variables
    planning_util::statestruct rollingstate = initstate;
    real_t acadoWSstate[85];
    // get kappac at initstate
    vector<float> kappac_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.kappa_c,false);
    float kappac = kappac_vec.at(0);
    // get mu at initstate
    vector<float> mu_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.mu,false);
    float mu = mu_vec.at(0);

    // set initial state in traj
    planning_util::traj_push_back_state(traj,rollingstate);
    traj.kappac.push_back(kappac);
    traj.mu.push_back(mu);
    // set initial state in integrator
    acadoWSstate[0] = rollingstate.s;
    acadoWSstate[1] = rollingstate.d;
    acadoWSstate[2] = rollingstate.deltapsi;
    acadoWSstate[3] = rollingstate.psidot;
    acadoWSstate[4] = rollingstate.vx;
    acadoWSstate[5] = rollingstate.vy;
    acadoWSstate[80] = kappac;

    // roll loop
    bool integrator_initiated = false;
    for (size_t i=0;i<N;i++) {
        float Fyf = traj.Fyf.at(i);
        float Fx  = traj.Fx.at(i);

        // adjust values according to mu UPDATE after introducing Fxf input
        float Fz = sp.m*sp.g ;
        float Fhmax = Fz*mu;
        float Fh = std::sqrt(2*Fyf*2*Fyf + Fx*Fx); // assuming Fyfmax = 0.5*Fxmax
        if(Fh > Fhmax){
            float scalefactor = Fh/Fhmax;
            //cout << "Fh    " << Fh << endl;
            //cout << "Fhmax " << Fhmax << endl;
            Fyf = Fyf/scalefactor;
            Fx = Fx/scalefactor;
        }

        acadoWSstate[77] = Fyf;
        acadoWSstate[78] = Fx;

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
        acadoWSstate[80] = kappac_vec.at(0);
        traj.kappac.push_back(kappac);
        // update mu at rollingstate
        vector<float> mu_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.mu,false);
        mu = mu_vec.at(0);
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
                                   planning_util::statestruct &state,
                                   planning_util::pathstruct &pathlocal,
                                   uint Ntrajs){
    if(Ntrajs % 2 !=0){
        throw "Error in computeTrajset, Ntrajs must be even";
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

//    cout << "dref: ";
//    for (uint i=0;i<dref.size();i++) {
//        cout << dref.at(i) << "; " << endl;
//    }
    //cout << endl;

    // generate trajs
    for (uint i=0;i<Ntrajs;i++) { // loop over trajectory set
        real_t acadoWSstate[85];
        planning_util::statestruct rollingstate = state;
        planning_util::ctrlstruct ctrl;

        // integrator loop
        auto t1_single = std::chrono::high_resolution_clock::now();
        planning_util::trajstruct traj;
        bool is_initstate = true;
        bool integrator_initiated = false;
        float kappac;
        float mu;

        double t_intgr_tot = 0;
        double t_interp_tot = 0;
        for (size_t j=0;j<N+1;j++) { // loop over single trajectory
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

            // TODO: IMPROVE ROLLOUT CONTROLLER
            // TODO: acadohelper to clean this and singlerollout

            // if in second half of dref, use the negative value of Fx
//            if (i >= Ntrajs/2){
//                ctrl.Fx = -ctrl.Fx;
//            }
            acadoWSstate[77] = ctrl.Fyf;
            acadoWSstate[78] = ctrl.Fx;

            // interp to get kappac at rollingstate.s
            auto t1_interp = std::chrono::high_resolution_clock::now();
            vector<float> kappac_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.kappa_c,false);
            kappac = kappac_vec.at(0);
            acadoWSstate[80] = kappac;
            // interp to get mu at rollingstate.s
            vector<float> mu_vec = cpp_utils::interp({rollingstate.s},pathlocal.s,pathlocal.mu,false);
            mu = mu_vec.at(0);
            // interp timing
            auto t2_interp = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> t_interp = t2_interp - t1_interp;
            t_interp_tot += t_interp.count();

            if (is_initstate){
                // set init state in integrator
                RtisqpWrapper::setIntegratorState(acadoWSstate,rollingstate,ctrl,kappac);
                is_initstate = false;
            }
            else {
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
            }

            // set s and d of the state that rolls fwd
            rollingstate.s = acadoWSstate[0];
            rollingstate.d = acadoWSstate[1];
            rollingstate.deltapsi = acadoWSstate[2];

            // extract variables from integrator
            traj.s.push_back(acadoWSstate[0]);
            traj.d.push_back(acadoWSstate[1]);
            traj.deltapsi.push_back(acadoWSstate[2]);
            traj.psidot.push_back(acadoWSstate[3]);
            traj.vx.push_back(acadoWSstate[4]);
            traj.vy.push_back(acadoWSstate[5]);

            if(j<N){ // N+1 states and N ctrls
                traj.Fyf.push_back(acadoWSstate[77]);
                traj.Fx.push_back(acadoWSstate[78]);
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

// TODO REMOVE
void RtisqpWrapper::setIntegratorState(real_t *acadoWSstate,
                                       planning_util::statestruct state,
                                       planning_util::ctrlstruct ctrl,
                                       float kappac){
    // state
    acadoWSstate[0] = state.s;
    acadoWSstate[1] = state.d;
    acadoWSstate[2] = state.deltapsi;
    acadoWSstate[3] = state.psidot;
    acadoWSstate[4] = state.vx;
    acadoWSstate[5] = state.vy;
    acadoWSstate[6] = 0.0; // dummy state
    // ctrl
    acadoWSstate[77] = ctrl.Fyf;
    acadoWSstate[78] = ctrl.Fx;
    acadoWSstate[79] = 0.0; // slack varreal_t[85]
    // onlinedata
    acadoWSstate[80] = kappac;
    acadoWSstate[81] = 0.0; // slb
    acadoWSstate[82] = 0.0; // sub
    acadoWSstate[83] = 0.0; // dlb
    acadoWSstate[84] = 0.0; // dub
}

