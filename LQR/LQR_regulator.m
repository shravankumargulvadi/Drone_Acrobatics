clc;
clear all;
close all;
%% LQR Gain calculation using Linearized model
m = 0.028;
g = 9.81;
Ix = 16.571710;
Iy = 16.655602;
Iz = 29.262652;
% Linearized Model
A = [0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 g 0 0 0 0;
     0 0 0 0 0 0 -g 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0];
B = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     1/m 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 1/Ix 0 0;
     0 0 1/Iy 0;
     0 0 0 1/Iz];
% LQR Gain Calculation
Q = 100*eye(12);
R = 0.75*eye(4);
N = zeros(12, 4);
K = lqr(A,B,Q,R,N);

%% Drone Simulation
tspan = 1:0.001:100;
X0 = [1, 2,2, 0, 0, 0, 0, pi/10, 0, 0, 0, 0]'; %initial condition
X_set = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]'; %set point
[t, X] = ode45(@(t,X) dronemodel(tspan, X, K, X_set), tspan, X0);
plot(t,X)
legend('x', 'y', 'z', 'u', 'v', 'w', '\phi', '\theta', '\psi', 'p', 'q', 'r');

%% Nonlinear Drone Model
function X_dot = dronemodel(t,X,K, X_set)
m = 0.028;
g = 9.81;
l = 0.05;
% k1 = 0.4;
% k2 = 0.05;
Ix = 16.571710;
Iy = 16.655602;
Iz = 29.262652;
I = [Ix 0 0; 0 Iy 0; 0 0 Iz];
x = X(1);
y = X(2);
z = X(3);
u = X(4);
v = X(5);
w = X(6);
phi = X(7);
theta = X(8);
psi = X(9);
p = X(10); 
q = X(11); 
r = X(12);
R_phi = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
R_theta = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
R_psi = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
R_be = R_phi * R_theta * R_psi;
R_eb = R_be';
R_r = [1 0 -sin(theta); 0 cos(phi) cos(theta)*sin(phi); 0 -sin(phi) cos(phi)*cos(theta)];
R_ir = [1 sin(phi)*tan(theta) cos(phi)*tan(theta); 0 cos(phi) -sin(phi); 0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
% R_u = [k1 k1 k1 k1; 0 -l*k1 0 l*k1; l*k1 0 -l*k1 0; -k2 k2 -k2 k2];
U = -K*(X- X_set);
% u_rotors = R_u*U; %currently not being used 
% State Space Representation
X_dot = zeros(12,1);
X_dot(1:3) = R_eb*X(4:6);
X_dot(4:6) = [r*v - q*w; p*w - r*u; q*u - p*v] +  g*[sin(theta); -cos(theta)*sin(phi); -cos(theta)*cos(phi)] + 1/m*[0; 0; (U(1)+m*g)];
X_dot(7:9) = R_ir*X(10:12);
X_dot(10:12) = [((Iy - Iz)/Ix)*q*r; ((Iz-Ix)/Iy)*p*r; ((Ix-Iy)/Iz)*p*q]+[(1/Ix)*U(2); (1/Iy)*U(3); (1/Iz)*U(4)];
end