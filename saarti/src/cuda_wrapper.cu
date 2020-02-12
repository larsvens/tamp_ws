#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>
#include <vector>
#include "containers.h"

__global__
void single_rollout(int N,
                    float dt,
                    float *s,
                    float *d,
                    float *psi,
                    float *psidot,
                    float *vx,
                    float *vy,
                    float *Fyf,
                    float *Fxf,
                    float *Fxr)
{
    for (int i=0; i<N-1; i++)
        // set control input

        // euler fwd step tmp
        psi[i+1] = psi[i] + dt*1.0f;
}

float cuda_rollout(std::vector<containers::trajstruct> &trajset)
{
    int N = 40;
    float dt = 0.1;
    float *s, *d, *psi, *psidot, *vx, *vy, *Fyf, *Fxf, *Fxr;

    // Allocate Unified Memory – accessible from CPU or GPU
    // todo: malloc only first time
    // todo: each var is 2d array
    cudaMallocManaged(&s, N*sizeof(float));
    cudaMallocManaged(&d, N*sizeof(float));
    cudaMallocManaged(&psi, N*sizeof(float));
    cudaMallocManaged(&psidot, N*sizeof(float));
    cudaMallocManaged(&vx, N*sizeof(float));
    cudaMallocManaged(&vy, N*sizeof(float));
    cudaMallocManaged(&Fyf, N*sizeof(float));
    cudaMallocManaged(&Fxf, N*sizeof(float));
    cudaMallocManaged(&Fxr, N*sizeof(float));

    // set init state on the host
    s[0] = 1.0f;
    d[0] = 0.0f;
    psi[0] = 0.0f;
    psidot[0] = 0.1f;
    vx[0] = 0.0f;
    vy[0] = 0.0f;
    Fyf[0] = 0.0f;
    Fxf[0] = 0.0f;
    Fxr[0] = 0.0f;

    // Run Ntrajs rollouts on Ntraj threads on the GPU
    // for loop
    single_rollout<<<1, 1>>>(N, dt, s, d, psi, psidot, vx, vy, Fyf, Fxf, Fxr);

    // Wait for GPU to finish before accessing on host
    cudaDeviceSynchronize();

    // Check for errors (all values should be 3.0f)
//    float maxError = 0.0f;
//    for (int i = 0; i < N; i++)
//        maxError = fmax(maxError, fabs(y[i]-3.0f));
//    std::cout << "Max error: " << maxError << std::endl;

    float val = psi[N-1];

    // Free memory
    cudaFree(s);
    cudaFree(d);
    cudaFree(psi);
    cudaFree(psidot);
    cudaFree(vx);
    cudaFree(vy);
    cudaFree(Fyf);
    cudaFree(Fxf);
    cudaFree(Fxr);

    return val;
}
