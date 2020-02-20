#include "saarti/planning_util.h"

// put state at the end of a traj
void planning_util::traj_push_back_state(containers::trajstruct &traj, containers::statestruct &state){
    traj.s.push_back(state.s);
    traj.d.push_back(state.d);
    traj.deltapsi.push_back(state.deltapsi);
    traj.psidot.push_back(state.psidot);
    traj.vx.push_back(state.vx);
    traj.vy.push_back(state.vy);
}

// get state at specific index in traj
void planning_util::state_at_idx_in_traj(containers::trajstruct &traj, containers::statestruct &state, uint idx){
    if (idx>=traj.s.size() ){
        throw "Error in state_at_idx_in_traj, idx out of range";
    }
    state.s = traj.s.at(idx);
    state.d = traj.d.at(idx);
    state.deltapsi = traj.deltapsi.at(idx);
    state.psidot = traj.psidot.at(idx);
    state.vx = traj.vx.at(idx);
    state.vy = traj.vy.at(idx);
}

// wraps an angle variable on the interval [-pi pi]
void planning_util::angle_to_interval(vector<float> &psi){
    // default interval: [-pi pi]
    float pi = float(M_PI);
    for (uint i=0; i<psi.size(); i++){
        while(psi.at(i) > pi){
            psi.at(i) = psi.at(i) - 2*pi;
        }
        while(psi.at(i) <= -pi){
            psi.at(i) = psi.at(i) + 2*pi;
        }
    }
}

// unwraps an angle variable on the interval [-pi pi] to continous
vector<float> planning_util::angle_to_continous(vector<float> &psi){
    float pi = float(M_PI);
    float offset = 0;
    vector<float> psi_cont;
    for (uint i=0;i<psi.size()-1;i++) {
        psi_cont.push_back(psi.at(i) + offset);
        if(psi.at(i+1) - psi.at(i) > pi){ // detecting up-flip
            offset = offset - 2*pi;
        }
        if(psi.at(i+1) - psi.at(i) < -pi){ // detecting down-flip
            offset = offset + 2*pi;
        }
    }
    psi_cont.push_back(psi.back() + offset); // final value
    if (psi_cont.size() != psi.size()){
        throw std::invalid_argument("fault in angle_to_continous");
    }
    return psi_cont;
}

// computes cartesian coordinates of a set of s,d pts
void planning_util::sd_pts2cart(vector<float> &Xout, vector<float> &Yout, vector<float> &s, vector<float> &d, containers::pathstruct &pathlocal){
    vector<float> Xc = cpp_utils::interp(s,pathlocal.s,pathlocal.X,false);
    vector<float> Yc = cpp_utils::interp(s,pathlocal.s,pathlocal.Y,false);
    vector<float> pathlocal_psic_cont = planning_util::angle_to_continous(pathlocal.psi_c);
    vector<float> psic = cpp_utils::interp(s,pathlocal.s,pathlocal_psic_cont,false);
    planning_util::angle_to_interval(psic);

    for (uint j=0; j<s.size();j++) {
        // X = Xc - d*sin(psic);
        // Y = Yc + d*cos(psic);
        // psi = deltapsi + psic;
        float X = Xc.at(j) - d.at(j)*std::sin(psic.at(j));
        float Y = Yc.at(j) + d.at(j)*std::cos(psic.at(j));
        Xout.push_back(X);
        Yout.push_back(Y);
    }
}

// computes cartesian coordinates of a trajectory
void planning_util::traj2cart(containers::trajstruct &traj, containers::pathstruct &pathlocal){
    if(!traj.s.size()){
        throw std::invalid_argument("traj2cart on traj of 0 length");
    }
    else {
        // clear previous cartesian if exists
        if (traj.X.size() != 0){
            std::cout << "WARNING: traj already has cartesian, clearing X, Y, psi, kappac" << std::endl;
            traj.X.clear();
            traj.Y.clear();
            traj.psi.clear();
            traj.kappac.clear();
        }

        vector<float> Xc = cpp_utils::interp(traj.s,pathlocal.s,pathlocal.X,false);
        vector<float> Yc = cpp_utils::interp(traj.s,pathlocal.s,pathlocal.Y,false);
        // handle discontinuities in psic
        vector<float> pathlocal_psic_cont = planning_util::angle_to_continous(pathlocal.psi_c);
        vector<float> psic = cpp_utils::interp(traj.s,pathlocal.s,pathlocal_psic_cont,false);
        planning_util::angle_to_interval(psic); // bring traj.psic back to [-pi pi]

        for (uint j=0; j<traj.s.size();j++) {
            // X = Xc - d*sin(psic);
            // Y = Yc + d*cos(psic);
            // psi = deltapsi + psic;
            float X = Xc.at(j) - traj.d.at(j)*std::sin(psic.at(j));
            float Y = Yc.at(j) + traj.d.at(j)*std::cos(psic.at(j));
            float psi = traj.deltapsi.at(j) + psic.at(j);

            // build vectors
            traj.X.push_back(X);
            traj.Y.push_back(Y);
            traj.psi.push_back(psi);
        }
        traj.kappac = cpp_utils::interp(traj.s,pathlocal.s,pathlocal.kappa_c,false);
        // ensure psi is in [-pi pi]
        planning_util::angle_to_interval(traj.psi);
    }
}

// computes cartesian coordinates of a trajectory set
void planning_util::trajset2cart(vector<containers::trajstruct> &trajset, containers::pathstruct &pathlocal){
    for (uint i=0;i<trajset.size();i++) {
        planning_util::traj2cart(trajset.at(i),pathlocal);
    }
}

// get corners for visualization
Eigen::MatrixXf planning_util::get_vehicle_corners(float X, float Y, float psi, float lf, float lr, float w){
    Eigen::MatrixXf R(2,2);
    Eigen::MatrixXf dims(5,2);
    Eigen::MatrixXf Xoffset(5,2);
    Eigen::MatrixXf Yoffset(5,2);
    Eigen::MatrixXf corners(5,2);

    R << std::cos(psi), std::sin(psi),
        -std::sin(psi), std::cos(psi);

    dims << lf, w/2,
            -lr, w/2,
            -lr, -w/2,
            lf, -w/2,
            lf, w/2;

    // rotate
    corners = dims*R;

    // add position offset
    Xoffset << X, 0.0f,
               X, 0.0f,
               X, 0.0f,
               X, 0.0f,
               X, 0.0f;
    Yoffset << 0.0f, Y,
               0.0f, Y,
               0.0f, Y,
               0.0f, Y,
               0.0f, Y;

    return corners + Xoffset + Yoffset;
}

// computes normal forces of a trajectory
void planning_util::get_additional_traj_variables(containers::trajstruct &traj, containers::pathstruct &pathlocal, containers::staticparamstruct sp, uint N){
    // mu
    vector<float> mu = cpp_utils::interp(traj.s,pathlocal.s,pathlocal.mu,false);

    // forces
    float theta = 0; // grade angle todo get from pathlocal
    float ax;
    float Fzf;
    float Fzr;
    float Fyr;
    float Cf;
    for (size_t k=0;k<N;k++) {
        // normal forces
        ax = (traj.Fxf.at(k)+traj.Fxr.at(k))/sp.m;
        Fzf = (1.0f/(sp.lf+sp.lr))*( sp.m*ax*sp.h_cg - sp.m*sp.g*sp.h_cg*std::sin(theta) + sp.m*sp.g*sp.lr*std::cos(theta));
        Fzr = (1.0f/(sp.lf+sp.lr))*(-sp.m*ax*sp.h_cg + sp.m*sp.g*sp.h_cg*std::sin(theta) + sp.m*sp.g*sp.lf*std::cos(theta));
        traj.Fzf.push_back(Fzf);
        traj.Fzr.push_back(Fzr);
        // Fyr
        Fyr = 2*traj.Cr.at(k)*std::atan(sp.lr*traj.psidot.at(k)-traj.vy.at(k))/traj.vx.at(k); // need atan here?
        traj.Fyr.push_back(Fyr);


        // Cf
        Cf = planning_util::get_cornering_stiffness(mu.at(k),Fzf);
        traj.Cf.push_back(Cf);
    }
}

// linearizes paceica magic formula
float planning_util::get_cornering_stiffness(float mu, float Fz){
    float B, C, D; //, E;

    if(0.0f <= mu && mu <0.3f){ // ice
        B = 4.0f;
        C = 2.0f;
        D = mu;
        //E = 1.0f;
    } else if (0.3f <= mu && mu <0.5f) { // snow
        B = 5.0f;
        C = 2.0f;
        D = mu;
        //E = 1.0f;
    } else if (0.5f <= mu && mu <0.9f) { // wet
        B = 12.0f;
        C = 2.3f;
        D = mu;
        //E = 1.0f;
    } else if (0.9f <= mu && mu <1.5f) { // dry
        B = 10.0f;
        C = 1.9f;
        D = mu;
        //E = 0.97f;
    } else if (1.5f <= mu && mu <2.5f) { // dry + racing tires (gotthard default)
        B = 12.56f;
        C = 1.38f;
        D = mu;
        //E = 1.00f;
    } else {
        throw std::invalid_argument("faulty mu value in get_cornering_stiffness");
    }
    return B*C*D*Fz; // Rajamani
}
