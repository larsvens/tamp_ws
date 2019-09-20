#include "saarti/rollout.h"

bool Rollout::computeTrajset(std::vector<planning_util::trajstruct> &trajset,
                             planning_util::statestruct &state,
                             planning_util::pathstruct &pathlocal,
                             uint N,
                             uint Ntrajs,
                             uint ctrl_mode){

    // debug input state
    std::cout << "debug comptrajeset: state: " << std::endl;
    std::cout << "state.s =        " << state.s << std::endl;
    std::cout << "state.d =        " << state.d << std::endl;
    std::cout << "state.deltapsi = " << state.deltapsi << std::endl;
    std::cout << "state.psidot =   " << state.psidot << std::endl;
    std::cout << "state.vx =       " << state.vx << std::endl;
    std::cout << "state.vy =       " << state.vy << std::endl;

    // todo input Fh_MAX
    float Fyfmax = 1000;
    float Fxmax = 2000;

    // mode 0 input sampling
    std::vector<float> Fyf_vec;
    std::vector<float> Fx_vec;
    float angle = 0;
    if(ctrl_mode==2){
        angle = -float(M_PI/2.0);
    } else {
        angle = float(M_PI/2.0);
    }


    float step = float(M_PI)/Ntrajs;
    for (uint i=0;i<Ntrajs;i++) {
        Fyf_vec.push_back(Fyfmax*sin(angle));
        Fx_vec.push_back(Fxmax*cos(angle));
        std::cout << "angle = " << angle << std::endl;
        std::cout << "Fyf = " << Fyfmax*sin(angle) << std::endl;
        std::cout << "Fx = " << Fxmax*cos(angle) << std::endl;
        angle += step;
    }

    for (uint i=0;i<Ntrajs;i++) {

        float Fyf = Fyf_vec.at(i);
        float Fx = Fx_vec.at(i);

        // TMP for finalizing fssim integration
        //float Fyf = 0;
        //float Fx = 1000;

        // input initial state and control on integrator format
        real_t acadoWSstate[85];
        // state
        acadoWSstate[0] = double(state.s);
        acadoWSstate[1] = double(state.d);
        acadoWSstate[2] = double(state.deltapsi);
        acadoWSstate[3] = double(state.psidot);
        acadoWSstate[4] = double(state.vx);
        acadoWSstate[5] = double(state.vy);
        acadoWSstate[6] = 0.0; // dummy state
        // ctrl
        acadoWSstate[77] = double(Fyf);
        acadoWSstate[78] = double(Fx);
        acadoWSstate[79] = 0.0; // slack var
        // onlinedata
        std::vector<float> s_vec = {state.s};
        std::vector<float> kappac_vec = cpp_utils::interp(s_vec,pathlocal.s,pathlocal.kappa_c,false);
        acadoWSstate[80] = double(kappac_vec.at(0));
        acadoWSstate[81] = 0.0; // slb
        acadoWSstate[82] = 0.0; // sub
        acadoWSstate[83] = 0.0; // dlb
        acadoWSstate[84] = 0.0; // dub


        // integrate fwd
        planning_util::trajstruct traj;
        traj.s.push_back(state.s);

        traj.d.push_back(state.d);
        traj.deltapsi.push_back(state.deltapsi);
        traj.psidot.push_back(state.psidot);
        traj.vx.push_back(state.vx);
        traj.vy.push_back(state.vy);
        traj.Fyf.push_back(Fyf);
        traj.Fx.push_back(Fx);

        traj.kappac.push_back(kappac_vec.at(0));

        for (size_t j=1;j<N+1;j++) {
            acado_integrate(acadoWSstate,1); // reset or not reset?
            // extract variables
            traj.s.push_back(float(acadoWSstate[0]));
            traj.d.push_back(float(acadoWSstate[1]));
            traj.deltapsi.push_back(float(acadoWSstate[2]));
            traj.psidot.push_back(float(acadoWSstate[3]));
            traj.vx.push_back(float(acadoWSstate[4]));
            traj.vy.push_back(float(acadoWSstate[5]));
            if(j<N){ // N+1 states and N ctrls
                traj.Fyf.push_back(float(acadoWSstate[77]));
                traj.Fx.push_back(float(acadoWSstate[78]));
            }

            // get kappa at new s
            std::vector<float> s_vec = {traj.s.at(j)};
            std::vector<float> kappac_vec = cpp_utils::interp(s_vec,pathlocal.s,pathlocal.kappa_c,false);
            acadoWSstate[80] = double(kappac_vec.at(0));
            traj.kappac.push_back(kappac_vec.at(0));
        }

        trajset.push_back(traj);
    }

    return true;
}
