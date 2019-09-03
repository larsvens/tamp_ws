#include <vector>

// move struct def
struct trajstruct{ // internal trajectory representation
    // state
    std::vector<float> s;
    std::vector<float> d;
    std::vector<float> deltapsi;
    std::vector<float> psidot;
    std::vector<float> vx;
    std::vector<float> vy;
    // ctrl
    std::vector<float> Fyf;
    std::vector<float> Fx;
    // od
    std::vector<float> kappa;
    bool colliding;
    float cost;
};
