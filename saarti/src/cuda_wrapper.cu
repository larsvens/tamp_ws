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
                               float *kappac_path,
                               float m,
                               float Iz,
                               float lf,
                               float lr,
                               uint Nt,
                               uint Ni,
                               uint Nx,
                               uint Nu,
                               uint Ntrajs,
                               uint Npp,
                               float dt)
{
    // notes on indexing:
    // trajset_arr:
    // row (i) <- state (Nx+Nu)
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
    for (int ki=0; ki<(Nt*Ni)-1; ki++){

        // get kappac at s (todo!)
        for (int id= 0; id<Npath; ++id) {
            // break if spath - s is positive, save index
        }
        float kappac = 0;

        // set control input
        float Fyf = 10000*((float(threadIdx.x)/float(Ntrajs))-0.5f);
        float Fxf = 500;
        float Fxr = 500;

        // set Fyr
        float Cr = 200000; // todo get dynamically!
        float Fyr = 2*Cr*atan(lr*psidot-vy)/vx; // help variable

        // euler fwd step
        s        = s + (dt/Ni)*((vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac));
        d        = d + (dt/Ni)*(vx*sin(deltapsi)+vy*cos(deltapsi));
        deltapsi = deltapsi + (dt/Ni)*(psidot-kappac*(vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac));
        psidot   = psidot + (dt/Ni)*((1/Iz)*(lf*Fyf - lr*Fyr));
        vx       = vx + (dt/Ni)*((1/m)*(Fxf+Fxr));
        vy       = vy + (dt/Ni)*((1/m)*(Fyf+Fyr)-vx*psidot);

        if(ki % Ni == 0){
            uint k = ki/Ni;
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
                  uint Nt, // N in planning horizon
                  uint Ni, // scaling factor in integration
                  uint Ntrajs, // Nr of trajs to roll out TODO replace w. Nd and Nvx
                  float dt)  // dt of planning horizon
{

    // init variables
    uint Nx = 6;
    uint Nu = 3;
    uint Npp = (Nx+Nu)*Ntrajs; // elements_per_page
    float *trajset_arr;
    float *x_init;
    float *d_ref;

    uint Npath = pathlocal.s.size();
    float *s_path;
    float *kappac_path;
    //float *mu_path;

    // allocate shared memory
    cudaMallocManaged(&trajset_arr, (Nx+Nu)*Ntrajs*Nt*sizeof(float));
    cudaMallocManaged(&x_init, Nx*sizeof(float));
    cudaMallocManaged(&d_ref, Ntrajs*sizeof(float));
    cudaMallocManaged(&s_path, Npath*sizeof(float));
    cudaMallocManaged(&kappac_path, Npath*sizeof(float));
    //cudaMallocManaged(&mu_path, Npath*sizeof(float));

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
    for(int id=0; id<Ntrajs; ++id) {
        d_ref[id] = dlb+id*dstep;
    }

    // set path variables
    for(int id=0; id<Npath; ++id) {
        s_path[id] = pathlocal.s.at(id);
        kappac_path[id] = pathlocal.kappa_c.at(id);
        //mu_path[id] = pathlocal.mu.at(id);
    }

    // run Ntrajs rollouts on Ntraj threads on gpu
    single_rollout<<<1, Ntrajs>>>(trajset_arr,
                                  x_init,
                                  Npath,
                                  s_path,
                                  kappac_path,
                                  sp.m,
                                  sp.Iz,
                                  sp.lf,
                                  sp.lr,
                                  Nt,
                                  Ni,
                                  Nx,
                                  Nu,
                                  Ntrajs,
                                  Npp,
                                  dt);

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
        }
        trajset_struct.push_back(traj);
    }
    std::cout << "reached end of cuda_rollout" << std::endl;

    // Free memory
    cudaFree(trajset_arr);

}
