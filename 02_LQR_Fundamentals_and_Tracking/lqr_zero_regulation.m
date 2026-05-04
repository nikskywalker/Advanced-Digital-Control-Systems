% ------------------------------------------------------------------------
% PROBLEM 1: Discrete LQ Control for Zero Regulation (Mass-Damper)
% ------------------------------------------------------------------------
% OBJECTIVE:
% Drive the state of a continuous-time mass-damper system to the origin
% (x = 0) in approximately 5s, within a strict tolerance (norm < 10^-5).
% ------------------------------------------------------------------------

clear all
close all
clc

% =========================================================================
% 1. CONTINUOUS-TIME PLANT DEFINITION (The Physical Reality)
% =========================================================================
% State variables: x1 = position, x2 = velocity.
% Input variable: u = applied force[cite: 7, 8].
A = [0  1; 
     0 -1];
B = [0; 
     1];

% The physical sensor only measures position[cite: 8].
C = [1 0]; 
D = 0;

% INITIAL CONDITION:
% Starting from rest (x2 = 0) but displaced 0.8m from the origin[cite: 7, 8].
x0 = [0.8; 0]; 

% =========================================================================
% 2. SYSTEM DISCRETIZATION (The Digital Controller Realm)
% =========================================================================
Ts = 0.05; % Sampling time in seconds[cite: 7, 8]

sys = ss(A, B, C, D);

% Discretize the continuous plant for digital controller design.
% Zero-Order Hold (ZOH) is used to model the digital-to-analog converter[cite: 8, 9].
sys_dt = c2d(sys, Ts, 'zoh');

Ad = sys_dt.A;
Bd = sys_dt.B;
Cd = sys_dt.C;
Dd = sys_dt.D;

% =========================================================================
% 3. REACHABILITY CHECK
% =========================================================================
% Verify if the system can be driven to any arbitrary state.
% Full rank mathematically guarantees that a solution for the infinite 
% horizon LQ optimal control problem actually exists[cite: 8, 9].
Mr = ctrb(Ad, Bd);
rho_Mr = rank(Mr);
fprintf('Reachability Matrix Rank: %d (System is %s)\n', rho_Mr, ...
    (rho_Mr == size(Ad,1)) + "Reachable");

% =========================================================================
% 4. LQR TUNING AND OBSERVABILITY CHECK
% =========================================================================
% --- Tuning Phase ---
% Q: State weight matrix. R: Control effort weight[cite: 8].
% Heavy penalty on position error (150) vs velocity error (1) forces fast positioning[cite: 8].
Q = diag([150, 1]);
R = 1;

% --- Observability Phase ---
% Compute the Cholesky factor of Q to check observability wrt the cost function[cite: 8, 9].
% Full rank guarantees that all dynamics (especially unstable ones) 
% are "seen" by the cost function and will be controlled[cite: 8].
Cq = chol(Q);
Mo = obsv(Ad, Cq);
rho_Mo = rank(Mo);
fprintf('Observability Matrix Rank: %d (System is %s)\n', rho_Mo, ...
    (rho_Mo == size(Ad,1)) + "Observable");

% =========================================================================
% 5. COMPUTATION OF THE DISCRETE OPTIMAL CONTROL
% =========================================================================
% 'dlqr' solves the Discrete Algebraic Riccati Equation (DARE).
% K is the optimal gain vector for the static state feedback law: u(k) = -K * x(k)[cite: 8].
K = dlqr(Ad, Bd, Q, R);
disp('Optimal State Feedback Gain (K):');
disp(K);

% =========================================================================
% 6. SIMULATION SETUP (Simulink Integration)
% =========================================================================
% [!] CRUCIAL SIMULINK REQUIREMENT: 
% The continuous LTI block must output the FULL state vector x 
% (position AND velocity) to feed the complete state feedback loop (-K*x)[cite: 8, 9].
% Therefore, we define a specific sys_x with C = eye(2).
sys_x = ss(A, B, eye(2), 0);

t_sim = 7;

% Execute the Simulink model.
% IMPORTANT: Ensure the 'LAB04_Ex1.slx' file is in the same directory.
try
    out = sim('LAB04_Ex1');
catch
    warning('Simulink file ''LAB04_Ex1.slx'' not found. Simulation skipped.');
    return;
end

% =========================================================================
% 7. RESULTS VERIFICATION AND PLOT
% =========================================================================
figure(1)

% Plot the Euclidean norm of the state ||x(t)||_2 to verify zero regulation[cite: 7, 8].
norm_x = sqrt((out.x.data(:,1)).^2 + (out.x.data(:,2)).^2);
plot(out.x.time, norm_x, 'b', 'LineWidth', 1.2)
grid on, zoom on, hold on

ylim([-0.2, 1.4])
xlabel('Time [s]')
ylabel('Norm ||x(t)||_2')
title('State Zero Regulation Performance')

% Plot specification boundaries: target tolerance and target time (5s)[cite: 8].
yline(1e-5, 'r', 'Tolerance 10^{-5}', 'LineWidth', 1)
xline(5.1, '--r', 'LineWidth', 0.8)
xline(4.9, '--r', 'LineWidth', 0.8)