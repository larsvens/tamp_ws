#ifndef CONTAINERS_H
#define CONTAINERS_H

#include <cstdlib>
#include <vector>
#include <iostream>
#include <math.h>

namespace containers {

// trajectory 
struct trajstruct{
    // state
    std::vector<float> s;
    std::vector<float> d;
    std::vector<float> deltapsi;
    std::vector<float> psidot;
    std::vector<float> vx;
    std::vector<float> vy;
    // ctrl
    std::vector<float> Fyf;
    std::vector<float> Fxf;
    std::vector<float> Fxr;
    // additional forces
    std::vector<float> Fyr;
    std::vector<float> Fzf;
    std::vector<float> Fzr;
    // od
    std::vector<float> kappac;
    // cartesian pose
    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> psi;
    // frictionestimate
    std::vector<float> mu;
    std::vector<float> Cf;
    std::vector<float> Cr;
    // misc
    std::vector<float> ax;
    // eval
    float cost;
    bool colliding;
    bool exitroad;
};

// state vector 
struct statestruct{
    float s;
    float d;
    float deltapsi;
    float psidot;
    float vx;
    float vy;
};

// ctrl vector 
struct ctrlstruct{
    float Fyf;
    float Fxf;
    float Fxr;
};

// pathlocal 
struct pathstruct{
    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> s;
    std::vector<float> psi_c;
    std::vector<float> kappa_c;
    std::vector<float> theta_c;
    std::vector<float> phi_c;
    std::vector<float> dub;
    std::vector<float> dlb;
    std::vector<float> mu;
};

// obstacle 
struct obstastruct{
    // state
    std::vector<float> s;
    std::vector<float> d;
    std::vector<float> R;
    std::vector<float> Rmgn;
    std::vector<float> X;
    std::vector<float> Y;
};

// position constraint 
struct posconstrstruct{
    std::vector<float> slb;
    std::vector<float> sub;
    std::vector<float> dlb;
    std::vector<float> dub;
};

// reference 
struct refstruct{
    int ref_mode;
    std::vector<float> sref;
    std::vector<float> vxref_path;
    float vxref_cc;
    float dref_cc;
};

// static params 
struct staticparamstruct{
    float m;
    float Iz;
    float g;
    float lf;
    float lr;
    float h_cg;
    float l_width;
};

}; // END NAMESPACE

#endif // CONTAINERS_H

