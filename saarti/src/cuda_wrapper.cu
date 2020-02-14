#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>
#include <vector>
#include "containers.h"

// device code for rollout
__global__ void single_rollout(float *trajset_arr,
                               float *x_init,
                               uint Npath,
                               float *s_path,
                               float *vxref_path,
                               float *kappac_path,
                               float *mu_path,
                               float mu_nominal,
                               float m,
                               float Iz,
                               float lf,
                               float lr,
                               float h_cg,
                               float g,
                               uint Nt,
                               uint Ni,
                               uint Nx,
                               uint Nu,
                               uint Ntrajs,
                               uint Npp,
                               float dt,
                               int traction_adaptive,
                               int ref_mode,
                               float vxref_nominal,
                               float *d_ref)
{
    // notes on indexing:
    // trajset_arr:
    // row (i) <- state (Nx+Nu+Nmisc)
    // column (j) <- traj (Ntrajs)
    // page (k) <- time (Nt)
    // iii - index of the 1d array that represents the 3d array (increase order: rows - columns - pages)
    // threadIdx.x = index of the current thread within its block (replaces j)

    // get init state
    float s        = x_init[0];
    float d        = x_init[1];
    float deltapsi = x_init[2];
    float psidot   = x_init[3];
    float vx       = x_init[4];
    float vy       = x_init[5];

    // set init state in trajset_arr
    trajset_arr[threadIdx.x + 0*Ntrajs + 0*Npp] = s;
    trajset_arr[threadIdx.x + 1*Ntrajs + 0*Npp] = d;
    trajset_arr[threadIdx.x + 2*Ntrajs + 0*Npp] = deltapsi;
    trajset_arr[threadIdx.x + 3*Ntrajs + 0*Npp] = psidot;
    trajset_arr[threadIdx.x + 4*Ntrajs + 0*Npp] = vx;
    trajset_arr[threadIdx.x + 5*Ntrajs + 0*Npp] = vy;

    // rollout loop
    float Fyf = 0.0f;
    float Fxf = 0.0f;
    float Fxr = 0.0f;
    uint k = 0; // k is the regular iterator, ki is upsampled
    for (int ki=0; ki<((Nt+1)*Ni); ki++){

        // get kappac at s
        int path_idx;
        for (path_idx = 0; path_idx<Npath; path_idx++) {
            // break if s - spath is negative, save index
            if(s-s_path[path_idx] <= 0){
                break;
            }
        }
        float kappac = kappac_path[path_idx];

        // set ax for Fz computation from previous cmd
        float ax = (Fxf+Fxr)/m;

        // get normal forces front and back and mu
        float Fzf;
        float Fzr;
        float mu;
        if(traction_adaptive == 1){
            float theta = 0; // grade angle todo get from pathlocal via traj
            Fzf = (1.0f/(lf+lr))*(m*ax*h_cg  - m*g*h_cg*sin(theta) + m*g*lr*cos(theta));
            Fzr = (1.0f/(lf+lr))*(-m*ax*h_cg + m*g*h_cg*sin(theta) + m*g*lf*cos(theta));
            mu = mu_path[path_idx]; // get mu from pathlocal at s
        } else { // (traction_adaptive == 0)
            Fzf = (1.0f/(lf+lr))*(m*g*lr);
            Fzr = (1.0f/(lf+lr))*(m*g*lf);
            mu = mu_nominal;
        }

        // get rear cornering stiffness
        float B, C, D; //, E;
        // todo add case for racing
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
        } else if (0.9f <= mu && mu <2.5f) { // dry
            B = 10.0f;
            C = 1.9f;
            D = mu;
            //E = 0.97f;
        } else {
            // todo Error = nonzero nr - check at host and throw error
        }
        float Cr = B*C*D*Fzr; // Rajamani

        // set Fyr
        float Fyr = 2*Cr*atan(lr*psidot-vy)/vx; // help variable

        // compute maximum tire forces
        float Ffmax = mu*Fzf;
        float Frmax = mu*Fzr;

        // get local vxref and vxerror
        float vxref;
        if(ref_mode == 1){ // cc
            vxref = vxref_nominal;
        } else if(ref_mode == 2) { // max s
            vxref = vxref_path[path_idx];
        } else {
            vxref = 0; // minimize s (refmode 0)
        }
        float vxerror = vxref-vx; // todo use blockIdx.x here

        // get derror (one per thread)
        float derror = d_ref[threadIdx.x] - d;

        /*
         * ROLLOUT CONTROLLER
         */

        // TODO CHECK THAT WEE GET SIMILAR OUTPUT FROM EULER AND RK4
        // TODO GET PARAMS FOR STARNDARD STATE FEEDBACK

        // select Fyf
        float feedfwd = 0.5f*m*vx*vx*kappac*cos(deltapsi);
        float feedback = 3000*derror - 500*deltapsi;
        Fyf = feedfwd + feedback;

        // saturate Fyf at Ffmax
        if(Fyf >= Ffmax){
            Fyf = Ffmax;
        }
        if(Fyf<=-Ffmax){
            Fyf = -Ffmax;
        }

        // select Fxf
        float Fxfmax = sqrt(Ffmax*Ffmax-Fyf*Fyf);
        if (ref_mode == 0){ // minimize s
            // select max negative Fxf until stop
            Fxf = -Fxfmax;
        } else if (ref_mode == 1 || ref_mode == 2){ // maximize s or cc
            if(vxerror > 0){ // accelerating
                Fxf = 0; // rear wheel drive - no drive on front wheel
            } else { // braking
                Fxf = 1000*vxerror;
                // saturate
                if(Fxf<=-Fxfmax){
                    Fxf = -Fxfmax;
                }
            }
        }

        // select Fxr
        //float Fxrmax = mu*Fzr;
        float Fxrmax = sqrt(Frmax*Frmax-Fyr*Fyr);
        if (ref_mode == 0){ // minimize s
            Fxr = -Fxrmax;
        } else if (ref_mode == 1 || ref_mode == 2){ // maximize s or cc
            Fxr = 1000*vxerror;
            // saturate
            if(Fxr >= Fxrmax){
                Fxr = Fxrmax;
            }
            if(Fxr<=-Fxrmax){
                Fxr = -Fxrmax;
            }
        }

        // old
        // Fyf = 10000*((float(threadIdx.x)/float(Ntrajs))-0.5f);
        // Fxf = 500;
        // Fxr = 500;

        // euler fwd step
        s        = s + (dt/Ni)*((vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac));
        d        = d + (dt/Ni)*(vx*sin(deltapsi)+vy*cos(deltapsi));
        deltapsi = deltapsi + (dt/Ni)*(psidot-kappac*(vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac));
        psidot   = psidot + (dt/Ni)*((1/Iz)*(lf*Fyf - lr*Fyr));
        vx       = vx + (dt/Ni)*((1/m)*(Fxf+Fxr));
        vy       = vy + (dt/Ni)*((1/m)*(Fyf+Fyr)-vx*psidot);

        if(ki % Ni == 0){
            k = ki/Ni;
            // set x at k+1
            trajset_arr[threadIdx.x + 0*Ntrajs + (k+1)*Npp] = s;
            trajset_arr[threadIdx.x + 1*Ntrajs + (k+1)*Npp] = d;
            trajset_arr[threadIdx.x + 2*Ntrajs + (k+1)*Npp] = deltapsi;
            trajset_arr[threadIdx.x + 3*Ntrajs + (k+1)*Npp] = psidot;
            trajset_arr[threadIdx.x + 4*Ntrajs + (k+1)*Npp] = vx;
            trajset_arr[threadIdx.x + 5*Ntrajs + (k+1)*Npp] = vy;
            // set u at k
            trajset_arr[threadIdx.x + 6*Ntrajs + (k)*Npp] = Fyf;
            trajset_arr[threadIdx.x + 7*Ntrajs + (k)*Npp] = Fxf;
            trajset_arr[threadIdx.x + 8*Ntrajs + (k)*Npp] = Fxr;
            // set miscvars
            trajset_arr[threadIdx.x + 9*Ntrajs + (k)*Npp] = Fzf;
            trajset_arr[threadIdx.x + 10*Ntrajs + (k)*Npp] = Fzr;
            trajset_arr[threadIdx.x + 11*Ntrajs + (k)*Npp] = kappac;
            trajset_arr[threadIdx.x + 12*Ntrajs + (k)*Npp] = mu;
            trajset_arr[threadIdx.x + 13*Ntrajs + (k)*Npp] = Cr;
        }
    }
}

// main rollout fcn, called by saarti_node
void cuda_rollout(std::vector<containers::trajstruct> &trajset_struct,
                  containers::statestruct initstate,
                  containers::pathstruct pathlocal,
                  containers::staticparamstruct sp,
                  int traction_adaptive,
                  float mu_nominal,
                  containers::refstruct refs,
                  float vxref_nominal,
                  uint Nt, // N in planning horizon
                  uint Ni, // scaling factor in integration
                  uint Ntrajs, // Nr of trajs to roll out TODO replace w. Nd and Nvx
                  float dt)  // dt of planning horizon
{

    // init variables
    uint Nx = 6;
    uint Nu = 3;
    uint Nmisc = 5; // nr of additional traj vars
    uint Npp = (Nx+Nu+Nmisc)*Ntrajs; // elements_per_page
    float *trajset_arr;
    float *x_init;
    float *d_ref;

    uint Npath = pathlocal.s.size();
    float *s_path;
    float *vxref_path;
    float *kappac_path;
    float *mu_path;

    // allocate shared memory
    cudaMallocManaged(&trajset_arr, (Nx+Nu+Nmisc)*Ntrajs*Nt*sizeof(float));
    cudaMallocManaged(&x_init, Nx*sizeof(float));
    cudaMallocManaged(&d_ref, Ntrajs*sizeof(float));
    cudaMallocManaged(&s_path, Npath*sizeof(float));
    cudaMallocManaged(&vxref_path, Npath*sizeof(float));
    cudaMallocManaged(&kappac_path, Npath*sizeof(float));
    cudaMallocManaged(&mu_path, Npath*sizeof(float));

    // set init state
    x_init[0] = initstate.s;
    x_init[1] = initstate.d;
    x_init[2] = initstate.deltapsi;
    x_init[3] = initstate.psidot;
    x_init[4] = initstate.vx;
    x_init[5] = initstate.vy;

    // set dref
    float dlb = pathlocal.dlb.at(0);
    float dub = pathlocal.dub.at(0);
    float dstep = (dub-dlb)/Ntrajs;
    for(uint id=0; id<Ntrajs; ++id) {
        d_ref[id] = dlb+id*dstep;
    }

    // set path variables
    for(uint id=0; id<Npath; ++id) {
        s_path[id] = pathlocal.s.at(id);
        kappac_path[id] = pathlocal.kappa_c.at(id);
        //mu_path[id] = pathlocal.mu.at(id);
    }

    // run Ntrajs rollouts on Ntraj threads on gpu
    single_rollout<<<1, Ntrajs>>>(trajset_arr,
                                  x_init,
                                  Npath,
                                  s_path,
                                  vxref_path,
                                  kappac_path,
                                  mu_path,
                                  mu_nominal,
                                  sp.m,
                                  sp.Iz,
                                  sp.lf,
                                  sp.lr,
                                  sp.h_cg,
                                  sp.g,
                                  Nt,
                                  Ni,
                                  Nx,
                                  Nu,
                                  Ntrajs,
                                  Npp,
                                  dt,
                                  traction_adaptive,
                                  refs.ref_mode,
                                  vxref_nominal,
                                  d_ref);

    // wait for GPU to finish before accessing on host
    cudaDeviceSynchronize();

    // put result on struct format
    for (uint j=0;j<Ntrajs;j++) {
        containers::trajstruct traj;
        for (size_t k=0;k<Nt+1;k++) {
            // std::cout << "s = " << trajset_arr[j + 0*Ntrajs + k*Npp] << std::endl;
            traj.s.push_back(trajset_arr[j + 0*Ntrajs + k*Npp]);
            traj.d.push_back(trajset_arr[j + 1*Ntrajs + k*Npp]);
            traj.deltapsi.push_back(trajset_arr[j + 2*Ntrajs + k*Npp]);
            traj.psidot.push_back(trajset_arr[j + 3*Ntrajs + k*Npp]);
            traj.vx.push_back(trajset_arr[j + 4*Ntrajs + k*Npp]);
            traj.vy.push_back(trajset_arr[j + 5*Ntrajs + k*Npp]);
            if(k<Nt){ // N+1 states and N ctrls
                traj.Fyf.push_back(trajset_arr[j + 6*Ntrajs + k*Npp]);
                traj.Fxf.push_back(trajset_arr[j + 7*Ntrajs + k*Npp]);
                traj.Fxr.push_back(trajset_arr[j + 8*Ntrajs + k*Npp]);
                // miscvars
                traj.Fzf.push_back(trajset_arr[j + 9*Ntrajs + k*Npp]);
                traj.Fzr.push_back(trajset_arr[j + 10*Ntrajs + k*Npp]);
                traj.kappac.push_back(trajset_arr[j + 11*Ntrajs + k*Npp]);
                traj.mu.push_back(trajset_arr[j + 12*Ntrajs + k*Npp]);
                traj.Cr.push_back(trajset_arr[j + 13*Ntrajs + k*Npp]);
            }
        }
        // add last element of misc vars
        traj.Fzf.push_back(traj.Fzf.back());
        traj.Fzr.push_back(traj.Fzr.back());
        traj.kappac.push_back(traj.kappac.back());
        traj.mu.push_back(traj.mu.back());
        traj.Cr.push_back(traj.Cr.back());
        trajset_struct.push_back(traj);
    }
    //std::cout << "reached end of cuda_rollout" << std::endl;

    // Free memory
    cudaFree(trajset_arr);

}
