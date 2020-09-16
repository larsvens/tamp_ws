#!/usr/bin/env python

from sympy import * 

X,Y,psi,psidot,vx,vy,Fyf,Fyr,Fx, Iz, m, lf, lr, theta, phi, g = symbols('X Y psi psidot v_x v_y F_{yf} F_{yr} F_x I_z m l_f l_r theta phi g')

x = Matrix([X,Y,psi,psidot,vx,vy,Fyf,Fyr,Fx])
f = Matrix([vx*cos(psi) - vy*sin(psi), 
            vx*sin(psi) + vy*cos(psi),
            psidot,
            (1/Iz)*(lf*Fyf - lr*Fyr),
            (1/m)*Fx - g*sin(theta),
            (1/m)*(Fyf+Fyr)-vx*psidot+g*sin(phi) ])


f.jacobian(x)
