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
                               float *dub_path,
                               float *dlb_path,
                               float mu_nominal,
                               float Ff_util,
                               float Fr_util,
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
                               uint Nd,
                               uint Nvx,
                               uint Npp,
                               uint Npb,
                               float dt,
                               int traction_adaptive,
                               float *d_ref_arr,
                               float *vx_ref_arr,
                               float *Kctrl)
{
    // notes on indexing:
    // trajset_arr:
    // row (i) <- state (Nx+Nu+Nmisc)
    // column (j) <- traj (Nd)
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

    // rollout loop
    float Fyf = 0.0f;
    float Fxf = 0.0f;
    float Fxr = 0.0f;
    float Fyr = 0.0f;
    float Fzf = 0.0f;
    float Fzr = 0.0f;
    float kappac = 0.0f;
    float mu = 0.0f;
    float Cr = 0.0f;
    uint k = 0; // k is the regular iterator, ki is upsampled
    for (int ki=0; ki<((Nt + 1)*Ni); ki++){

        if(ki % Ni == 0){ // only the euler step is performed every ki

            // init vars for cost eval
            bool colliding = false;
            bool exitroad = false;
            float cost = 0;

            // check if exitroad
            for (uint id = 0; id<Npath; id++) {
                if (d < dlb_path[id] || d > dub_path[id]){
                    exitroad = true;
                }
            }

            // check collision with obstacle

            // get path index
            uint path_idx;
            for (path_idx = 0; path_idx<Npath; path_idx++) {
                // break if s - spath is negative, save index
                if(s-s_path[path_idx] <= 0){
                    break;
                }
            }

            // get kappac
            kappac = kappac_path[path_idx];

            // set ax for Fz computation from previous cmd
            float ax = (Fxf+Fxr)/m;

            // get normal forces front and back and mu
            if(traction_adaptive == 1){
                float theta = 0; // grade angle todo get from pathlocal via traj
                Fzf = (1.0f/(lf+lr))*(-m*ax*h_cg  - m*g*h_cg*sin(theta) + m*g*lr*cos(theta));
                Fzr = (1.0f/(lf+lr))*(m*ax*h_cg + m*g*h_cg*sin(theta) + m*g*lf*cos(theta));
                mu = mu_path[path_idx]; // get mu from pathlocal at s
            } else { // (traction_adaptive == 0)
                Fzf = (1.0f/(lf+lr))*(m*g*lr);
                Fzr = (1.0f/(lf+lr))*(m*g*lf);
                mu = mu_nominal;
            }

            // get rear cornering stiffness
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
                // todo Error = nonzero nr - check at host and throw error
            }
            Cr = B*C*D*Fzr; // Rajamani

            // compute maximum tire forces
            float Ffmax = mu*Fzf*Ff_util;
            float Frmax = mu*Fzr*Fr_util;

         /*
         * ROLLOUT CONTROLLER
         */
            float s_ref        = x_init[0];
            float d_ref        = d_ref_arr[threadIdx.x];
            float deltapsi_ref = 0.0f;
            float psidot_ref   = 0.0f;
            float vx_ref       = vx_ref_arr[blockIdx.x];
            float vy_ref       = 0.0f;

            // LQR
            Fyf = -(Kctrl[0+0   ]*(s-s_ref) + Kctrl[1+0   ]*(d-d_ref) + Kctrl[2+0   ]*(deltapsi-deltapsi_ref) + Kctrl[3+0   ]*(psidot-psidot_ref) + Kctrl[4+0   ]*(vx-vx_ref) + Kctrl[5+0   ]*(vy-vy_ref));
            Fxf = -(Kctrl[0+Nx  ]*(s-s_ref) + Kctrl[1+Nx  ]*(d-d_ref) + Kctrl[2+Nx  ]*(deltapsi-deltapsi_ref) + Kctrl[3+Nx  ]*(psidot-psidot_ref) + Kctrl[4+Nx  ]*(vx-vx_ref) + Kctrl[5+Nx  ]*(vy-vy_ref));
            Fxr = -(Kctrl[0+2*Nx]*(s-s_ref) + Kctrl[1+2*Nx]*(d-d_ref) + Kctrl[2+2*Nx]*(deltapsi-deltapsi_ref) + Kctrl[3+2*Nx]*(psidot-psidot_ref) + Kctrl[4+2*Nx]*(vx-vx_ref) + Kctrl[5+2*Nx]*(vy-vy_ref));
            Fyf += 0.5f*m*vx*vx*kappac*cos(deltapsi); // feedfwd term for Fyf

            // saturate Fyf at Ffmax
            if(Fyf >= Ffmax){
                Fyf = Ffmax;
            }
            if(Fyf<=-Ffmax){
                Fyf = -Ffmax;
            }
            // saturate Fxf
            float Fxfmax = sqrt(Ffmax*Ffmax-Fyf*Fyf);
            if(Fxf > 0){ // accelerating
                Fxf = 0; // rear wheel drive - no drive on front wheel
            } else { // braking
                if(Fxf<=-Fxfmax){
                    Fxf = -Fxfmax;
                }
            }
            // saturate Fxr
            float Fxrmax = sqrt(Frmax*Frmax-Fyr*Fyr);
            if(Fxr >= Fxrmax){
                Fxr = Fxrmax;
            }
            if(Fxr<=-Fxrmax){
                Fxr = -Fxrmax;
            }
        }

        // set Fyr
        float Fyr = 2*Cr*atan(lr*psidot-vy)/vx; // help variable
        // saturate Fyr at Frmax
//        if(Fyr >= Frmax){
//            Fyr = Frmax;
//        }
//        if(Fyr<=-Frmax){
//            Fyr = -Frmax;
//        }

        // euler fwd step
        s        = s + (dt/Ni)*((vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac));
        d        = d + (dt/Ni)*(vx*sin(deltapsi)+vy*cos(deltapsi));
        deltapsi = deltapsi + (dt/Ni)*(psidot-kappac*(vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac));
        psidot   = psidot + (dt/Ni)*((1/Iz)*(lf*Fyf - lr*Fyr));
        vx       = vx + (dt/Ni)*((1/m)*(Fxf+Fxr));
        vy       = vy + (dt/Ni)*((1/m)*(Fyf+Fyr)-vx*psidot);

        // store data at Nt regular intervals
        if(ki % Ni == 0){
            k = ki/Ni;
            // set x at k+1
            if(ki==0){
                trajset_arr[threadIdx.x + 0*Nd + (k)*Npp + blockIdx.x*Npb] = x_init[0];
                trajset_arr[threadIdx.x + 1*Nd + (k)*Npp + blockIdx.x*Npb] = x_init[1];
                trajset_arr[threadIdx.x + 2*Nd + (k)*Npp + blockIdx.x*Npb] = x_init[2];
                trajset_arr[threadIdx.x + 3*Nd + (k)*Npp + blockIdx.x*Npb] = x_init[3];
                trajset_arr[threadIdx.x + 4*Nd + (k)*Npp + blockIdx.x*Npb] = x_init[4];
                trajset_arr[threadIdx.x + 5*Nd + (k)*Npp + blockIdx.x*Npb] = x_init[5];
            } else {
                trajset_arr[threadIdx.x + 0*Nd + (k)*Npp + blockIdx.x*Npb] = s;
                trajset_arr[threadIdx.x + 1*Nd + (k)*Npp + blockIdx.x*Npb] = d;
                trajset_arr[threadIdx.x + 2*Nd + (k)*Npp + blockIdx.x*Npb] = deltapsi;
                trajset_arr[threadIdx.x + 3*Nd + (k)*Npp + blockIdx.x*Npb] = psidot;
                trajset_arr[threadIdx.x + 4*Nd + (k)*Npp + blockIdx.x*Npb] = vx;
                trajset_arr[threadIdx.x + 5*Nd + (k)*Npp + blockIdx.x*Npb] = vy;
            }
            // set u at k
            trajset_arr[threadIdx.x + 6*Nd + (k)*Npp + blockIdx.x*Npb] = Fyf;
            trajset_arr[threadIdx.x + 7*Nd + (k)*Npp + blockIdx.x*Npb] = Fxf;
            trajset_arr[threadIdx.x + 8*Nd + (k)*Npp + blockIdx.x*Npb] = Fxr;
            // set miscvars
            trajset_arr[threadIdx.x + 9*Nd + (k)*Npp + blockIdx.x*Npb] = Fzf;
            trajset_arr[threadIdx.x + 10*Nd + (k)*Npp + blockIdx.x*Npb] = Fzr;
            trajset_arr[threadIdx.x + 11*Nd + (k)*Npp + blockIdx.x*Npb] = kappac;
            trajset_arr[threadIdx.x + 12*Nd + (k)*Npp + blockIdx.x*Npb] = mu;
            trajset_arr[threadIdx.x + 13*Nd + (k)*Npp + blockIdx.x*Npb] = Cr;
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
                  float Ff_util,
                  float Fr_util,
                  std::vector<float> Kctrl_,
                  uint Nt, // N in planning horizon
                  uint Nd, // Nr of goal pts in d (also nr of threads, multiples of 32 to maximize gpu utilization)
                  uint Nvx, // Nr of goal pts in vx (also nr of blocks)
                  float vxub, // upper bound on sampled vx
                  float dt)  // dt of planning horizon
{

    // init variables
    uint Ni = 10; // scaling factor in integration
    uint Nx = 6;
    uint Nu = 3;
    uint Nmisc = 5; // nr of additional traj vars
    uint Npp = (Nx+Nu+Nmisc)*Nd; // elements_per_page
    uint Npb = Npp*(Nt+1); // elements per block (state traj has Nt+1 elements and control has Nt)
    float *trajset_arr;
    float *x_init;
    float *d_ref_arr;
    float *vx_ref_arr;
    float *Kctrl;

    uint Npath = pathlocal.s.size();
    float *s_path;
    float *vxref_path;
    float *kappac_path;
    float *mu_path;
    float *dub_path;
    float *dlb_path;

    // allocate shared memory
    cudaMallocManaged(&trajset_arr, Npb*Nvx*sizeof(float));
    cudaMallocManaged(&x_init, Nx*sizeof(float));
    cudaMallocManaged(&d_ref_arr, Nd*sizeof(float));
    cudaMallocManaged(&vx_ref_arr, Nvx*sizeof(float));
    cudaMallocManaged(&s_path, Npath*sizeof(float));
    cudaMallocManaged(&vxref_path, Npath*sizeof(float));
    cudaMallocManaged(&kappac_path, Npath*sizeof(float));
    cudaMallocManaged(&mu_path, Npath*sizeof(float));
    cudaMallocManaged(&dub_path, Npath*sizeof(float));
    cudaMallocManaged(&dlb_path, Npath*sizeof(float));
    cudaMallocManaged(&Kctrl, Nu*Nx*sizeof(float));

    // set init state
    x_init[0] = initstate.s;
    x_init[1] = initstate.d;
    x_init[2] = initstate.deltapsi;
    x_init[3] = initstate.psidot;
    x_init[4] = initstate.vx;
    x_init[5] = initstate.vy;

    // set dref_arr
    float dub = pathlocal.dub.at(0);
    float dlb = pathlocal.dlb.at(0);
    float dstep = (dub-dlb)/(float(Nd)-1);
    for(uint id=0; id<Nd; ++id) {
        d_ref_arr[id] = dlb+float(id)*dstep;
        //std::cout << "d_ref_arr[id] = " << d_ref_arr[id] << std::endl;
    }

    // set vxref_arr
    float vxlb = 1.0f; // avoid singulatity at vx = 0
    float vxstep = (vxub-vxlb)/(float(Nvx)-1);
    //std::cout << "Nvx = " << Nvx << std::endl;
    for(uint id=0; id<Nvx; ++id) {
        vx_ref_arr[id] = vxlb+float(id)*vxstep;
        //std::cout << "vx_ref_arr[id] = " << vx_ref_arr[id] << std::endl;
    }

    // set control matrix K
    for (uint id=0; id<(Nu*Nx); id++){
        Kctrl[id] = Kctrl_.at(id);
    }


    // set path variables
    for(uint id=0; id<Npath; ++id) {
        s_path[id] = pathlocal.s.at(id);
        kappac_path[id] = pathlocal.kappa_c.at(id);
        mu_path[id] = pathlocal.mu.at(id);
        dub_path[id] = pathlocal.dub.at(id);
        dlb_path[id] = pathlocal.dlb.at(id);
    }

    // run Nd*Nvx rollouts on Ntraj threads on gpu
    single_rollout<<<Nvx, Nd>>>(trajset_arr,
                                x_init,
                                Npath,
                                s_path,
                                vxref_path,
                                kappac_path,
                                mu_path,
                                dub_path,
                                dlb_path,
                                mu_nominal,
                                Ff_util,
                                Fr_util,
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
                                Nd,
                                Nvx,
                                Npp,
                                Npb,
                                dt,
                                traction_adaptive,
                                d_ref_arr,
                                vx_ref_arr,
                                Kctrl);

    // wait for GPU to finish before accessing on host
    cudaDeviceSynchronize();

    // put result on struct format
    for (uint l=0;l<Nvx;l++){
        for (uint j=0;j<Nd;j++) {
            containers::trajstruct traj;
            for (size_t k=0;k<Nt+1;k++) {
                // std::cout << "s = " << trajset_arr[j + 0*Nd + k*Npp] << std::endl;
                traj.s.push_back(       trajset_arr[j + 0*Nd + k*Npp + l*Npb]);
                traj.d.push_back(       trajset_arr[j + 1*Nd + k*Npp + l*Npb]);
                traj.deltapsi.push_back(trajset_arr[j + 2*Nd + k*Npp + l*Npb]);
                traj.psidot.push_back(  trajset_arr[j + 3*Nd + k*Npp + l*Npb]);
                traj.vx.push_back(      trajset_arr[j + 4*Nd + k*Npp + l*Npb]);
                traj.vy.push_back(      trajset_arr[j + 5*Nd + k*Npp + l*Npb]);
                if(k<Nt){ // N+1 states and N ctrls
                    traj.Fyf.push_back( trajset_arr[j + 6*Nd + k*Npp + l*Npb]);
                    traj.Fxf.push_back( trajset_arr[j + 7*Nd + k*Npp + l*Npb]);
                    traj.Fxr.push_back( trajset_arr[j + 8*Nd + k*Npp + l*Npb]);
                    // miscvars
                    traj.Fzf.push_back( trajset_arr[j + 9*Nd + k*Npp + l*Npb]);
                    traj.Fzr.push_back( trajset_arr[j + 10*Nd + k*Npp + l*Npb]);
                    traj.kappac.push_back(trajset_arr[j + 11*Nd + k*Npp + l*Npb]);
                    traj.mu.push_back(  trajset_arr[j + 12*Nd + k*Npp + l*Npb]);
                    traj.Cr.push_back(  trajset_arr[j + 13*Nd + k*Npp + l*Npb]);
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
    }

    // Free memory
    cudaFree(trajset_arr);
    cudaFree(x_init);
    cudaFree(d_ref_arr);
    cudaFree(vx_ref_arr);
    cudaFree(s_path);
    cudaFree(vxref_path);
    cudaFree(kappac_path);
    cudaFree(mu_path);
    cudaFree(dub_path);
    cudaFree(dlb_path);
    cudaFree(Kctrl);

}
