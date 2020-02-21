clear;close all;
% Description: symbolic representation of dynamic vehicle model modified 
% from dyn model in Rajmani et al

% todo: link to drawing
% todo: python

% state vector (in order):
% x1 = s 
% x2 = d 
% x3 = deltapsi 
% x4 = psi_dot
% x5 = vx                    longitudinal velocity 
% x6 = vy                    lateral velocity

% controls:
% u1 = Fyf                   lateral force on front wheel
% u2 = Fxf
% u3 = Fxr

% params:
% Iz 
% lf 
% lr  
% m 
% g 
% phi                       bank angle (not included atm)
% theta                     pitch angle (not included atm)
% kappac                    curvature of the path at s

addpath(genpath('../linearization'));

% states:
syms s d deltapsi psi_dot vx vy;
% controls
syms Fyf Fxf Fxr;
% params
syms Iz lf lr m g; 
% help vars
syms Fyr kappac Cr;

Fyr = 2*Cr*(lr*psi_dot-vy)/vx;

% differential eqs:
syms f1 f2 f3 f4 f5 f6;
f1 = (vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac);
f2 = vx*sin(deltapsi) + vy*cos(deltapsi);
f3 = psi_dot - kappac*(vx*cos(deltapsi)-vy*sin(deltapsi))/(1-d*kappac);
f4 = (1/Iz)*(lf*Fyf - lr*Fyr);
f5 = (1/m)*(Fxf+Fxr);
f6 = (1/m)*(Fyf+Fyr)-vx*psi_dot;

model.x = {s d deltapsi psi_dot vx vy};
model.u = {Fyf Fxf Fxr};
model.f = {f1 f2 f3 f4 f5 f6};

% compute and print jacobian
print_symbolic_jacobian_of_nl_model(model);

