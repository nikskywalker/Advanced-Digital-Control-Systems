% ------------------------------------------------------------------------
% PROBLEM 2: Constrained Finite Horizon Optimal Control
% ------------------------------------------------------------------------
% OBJECTIVE:
% Compute the optimal control sequence ensuring that none of the 
% predicted inputs violate the physical boundaries (Input Saturation).
% ------------------------------------------------------------------------

clear all
close all
clc

% =========================================================================
% 1. SYSTEM DEFINITION & WEIGHTS
% =========================================================================
A = [0.3 1.5; 
     0.5 -0.4];
B = [0; 
     1];
x_k = [20; 20];
Ts = 0.1;

Hp = 3;
Q = diag([20, 5]);
S = diag([20, 5]); 
R = 5;

% =========================================================================
% 2. PREDICTION MATRICES & QUADRATIC FORMULATION
% =========================================================================
A_cal = [A; 
         A^2; 
         A^3];

B_cal = [[B        zeros(2,1) zeros(2,1)]; 
         [A*B      B          zeros(2,1)]; 
         [(A^2)*B  A*B        B         ]];

Q_cal = blkdiag(Q, Q, S);
R_cal = blkdiag(R, R, R);

H = 2 * (B_cal' * Q_cal * B_cal + R_cal);
H = (H + H') / 2; % Force exact symmetry
F = 2 * A_cal' * Q_cal * B_cal;

% =========================================================================
% 3. INPUT CONSTRAINT DEFINITION (|u(k)| <= 13)
% =========================================================================
% This is a STATIC constraint because it acts directly on the input U, 
% independent of the current state.
u_sat = 13;

% We translate the physical limitation into the format "G * U <= h".
% G stacks an identity matrix (upper bounds) and a negative identity matrix (lower bounds).
G = [ eye(3); 
     -eye(3)];
     
h = u_sat * ones(6,1);

% =========================================================================
% 4. CONSTRAINED OPTIMIZATION (Quadratic Programming)
% =========================================================================
% quadprog will find the optimal U sequence that respects G*U <= h.
% It "walks" down the quadratic bowl but stops exactly at the walls we built.
options = optimoptions('quadprog', 'Display', 'off');
U_star = quadprog(H, x_k'*F, G, h, [], [], [], [], [], options);

disp('--- Optimal Constrained Control Sequence (U*) ---');
disp(U_star);

% =========================================================================
% 5. STATE PREDICTION
% =========================================================================
X_pred = A_cal * x_k + B_cal * U_star;
X_star_full = [x_k; X_pred];

disp('--- Corresponding State Ahead Predictions (X*) ---');
disp(X_star_full);