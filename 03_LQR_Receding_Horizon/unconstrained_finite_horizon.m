% ------------------------------------------------------------------------
% PROBLEM 1: Unconstrained Finite Horizon Optimal Control
% ------------------------------------------------------------------------
% OBJECTIVE:
% Compute the optimal control sequence U*(k) = [u(k), u(k+1), u(k+2)] 
% for the next 3 steps without any physical saturation constraints. 
% Then, predict how the system state will evolve.
% ------------------------------------------------------------------------

clear all
close all
clc

% =========================================================================
% 1. SYSTEM DEFINITION
% =========================================================================
A = [0.3  1.5; 
     0.5 -0.4];
B = [0; 
     1]; 

x_k = [20; 20];     % Initial measured state (the starting point)
Ts = 0.1;           % Sampling time [s]

% =========================================================================
% 2. COST FUNCTION WEIGHTS & HORIZON
% =========================================================================
Hp = 3;             % Prediction Horizon

Q = [20 0; 
      0 5];         % State penalty weight

S = [20 0; 
      0 5];         % Terminal state penalty weight (S = Q in this case)

R = 5;              % Control effort penalty weight

% =========================================================================
% 3. PREDICTION MATRICES (The Controller's "Imagination")
% =========================================================================
% We are predicting 3 steps ahead. 
% A_cal propagates the autonomous behavior of the system.
A_cal = [A; 
         A^2; 
         A^3];

% B_cal maps the future input sequence U onto the future states.
% It is padded with zeros in the upper triangular part to enforce causality:
% a future input cannot affect a present state.
B_cal = [[B        zeros(2,1) zeros(2,1)]; 
         [A*B      B          zeros(2,1)]; 
         [(A^2)*B  A*B        B         ]];

% =========================================================================
% 4. EXTENDED COST MATRICES 
% =========================================================================
% The last block of Q_cal must be the terminal weight 'S'.
Q_cal = blkdiag(Q, Q, S);
R_cal = blkdiag(R, R, R);

% =========================================================================
% 5. QUADRATIC FORMULATION (H and F)
% =========================================================================
% We rewrite the cost function J into a standard quadratic form.
H = 2 * (B_cal' * Q_cal * B_cal + R_cal);
H = (H + H') / 2; % Trick to ensure perfect symmetry and avoid numerical warnings
F = 2 * A_cal' * Q_cal * B_cal;

% =========================================================================
% 6. UNCONSTRAINED OPTIMIZATION
% =========================================================================
% Since there are no physical limits, we find the absolute mathematical 
% minimum by setting the derivative to zero.
% Algebraically: U* = -H^(-1) * F' * x_k.
U_star = -H \ (F' * x_k);

disp('--- Optimal Control Sequence (U*) ---');
disp(U_star);

% =========================================================================
% 7. STATE PREDICTION
% =========================================================================
% We apply our optimal 3-step control sequence to the mathematical model 
% to see exactly where the system will end up.
X_pred = A_cal * x_k + B_cal * U_star;

% The problem specifically asks for X*(k) which MUST include the current 
% measured state x(k) at the very top.
X_star_full = [x_k; X_pred];

disp('--- Corresponding State Ahead Predictions (X*) ---');
disp(X_star_full);