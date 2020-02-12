#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>
#include <vector>
#include "containers.h"

__global__
void single_rollout(float *trajset_arr,
                    uint Nt,
                    uint Ni,
                    uint Nx,
                    uint Nu,
                    uint Ntrajs,
                    uint Npp,
                    float dt)
{
    // params (todo give as inputs)
    float Iz = 8158.0;
    float m = 8350.0;
    float lf = 1.205;
    float lr = 2.188;

    uint elements_per_page = (Nx+Nu)*Ntrajs;

    // get init state
    float s        = trajset_arr[threadIdx.x + 0*Ntrajs + 0*elements_per_page];
    float d        = trajset_arr[threadIdx.x + 1*Ntrajs + 0*elements_per_page];
    float deltapsi = trajset_arr[threadIdx.x + 2*Ntrajs + 0*elements_per_page];
    float psidot   = trajset_arr[threadIdx.x + 3*Ntrajs + 0*elements_per_page];
    float vx       = trajset_arr[threadIdx.x + 4*Ntrajs + 0*elements_per_page];
    float vy       = trajset_arr[threadIdx.x + 5*Ntrajs + 0*elements_per_page];

    // threadIdx.x = index of the current thread within its block (replaces j)
    for (int ki=0; ki<(Nt*Ni)-1; ki++){

        // get kappac at s (todo!)
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
            trajset_arr[threadIdx.x + 0*Ntrajs + (k+1)*elements_per_page] = s;
            trajset_arr[threadIdx.x + 1*Ntrajs + (k+1)*elements_per_page] = d;
            trajset_arr[threadIdx.x + 2*Ntrajs + (k+1)*elements_per_page] = deltapsi;
            trajset_arr[threadIdx.x + 3*Ntrajs + (k+1)*elements_per_page] = psidot;
            trajset_arr[threadIdx.x + 4*Ntrajs + (k+1)*elements_per_page] = vx;
            trajset_arr[threadIdx.x + 5*Ntrajs + (k+1)*elements_per_page] = vy;
            trajset_arr[threadIdx.x + 6*Ntrajs + (k+1)*elements_per_page] = Fyf;
            trajset_arr[threadIdx.x + 7*Ntrajs + (k+1)*elements_per_page] = Fxf;
            trajset_arr[threadIdx.x + 8*Ntrajs + (k+1)*elements_per_page] = Fxr;
        }
    }
}

void cuda_rollout(std::vector<containers::trajstruct> &trajset_struct,
                  containers::statestruct initstate,
                  uint Nt, // N in planning horizon
                  uint Ni, // scaling factor in integration
                  uint Ntrajs, // Nr of trajs to roll out
                  float dt)  // dt of planning horizon
{

    uint Nx = 6;
    uint Nu = 3;
    uint Npp = (Nx+Nu)*Ntrajs; // elements_per_page
    float *trajset_arr;

    // Allocate shared memory for trajset as 3d array
    // row (i) <- state (Nx+Nu)
    // column (j) <- traj (Ntrajs)
    // page (k) <- time (Nt)
    // iii - index of the 1d array that represents the 3d array (increase order: rows - columns - pages)
    cudaMallocManaged(&trajset_arr, (Nx+Nu)*Ntrajs*Nt*sizeof(float));

    // set init state in trajset_arr
    for (uint j=0; j<Ntrajs; j++) {
        trajset_arr[j] = initstate.s;
    }
    for (uint j=0; j<Ntrajs; j++) {
        trajset_arr[j+Ntrajs] = initstate.d;
    }
    for (uint j=0; j<Ntrajs; j++) {
        trajset_arr[j+2*Ntrajs] = initstate.deltapsi;
    }
    for (uint j=0; j<Ntrajs; j++) {
        trajset_arr[j+3*Ntrajs] = initstate.psidot;
    }
    for (uint j=0; j<Ntrajs; j++) {
        trajset_arr[j+4*Ntrajs] = initstate.vx;
    }
    for (uint j=0; j<Ntrajs; j++) {
        trajset_arr[j+5*Ntrajs] = initstate.vy;
    }

    // Run Ntrajs rollouts on Ntraj threads on the GPU
    single_rollout<<<1, Ntrajs>>>(trajset_arr,Nt,Ni,Nx,Nu,Ntrajs,Npp,dt);

    // Wait for GPU to finish before accessing on host
    cudaDeviceSynchronize();

    // put result on struct format
    for (uint j=0;j<Ntrajs;j++) {
        containers::trajstruct traj;
        for (size_t k=0;k<Nt+1;k++) {
            std::cout << "s = " << trajset_arr[j + 0*Ntrajs + k*Npp] << std::endl;
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
