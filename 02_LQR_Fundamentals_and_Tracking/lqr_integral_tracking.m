% ------------------------------------------------------------------------
% PROBLEM 2: LQR Tracking with Integral Action and Saturation Analysis
% ------------------------------------------------------------------------
% OBJECTIVE 1: Achieve zero steady-state error for step tracking (ts ~2s)[cite: 7, 8].
% OBJECTIVE 2: Analyze the impact of hard input constraints (|u(t)| <= 1)[cite: 7, 8].
% ------------------------------------------------------------------------

clear all
close all
clc

% =========================================================================
% 1. CONTINUOUS-TIME PLANT DEFINITION & DISCRETIZATION
% =========================================================================
A = [0 1; 
     0 -1];
B = [0; 
     1];
C = [1 0];
D = 0;
x0 = [0.8; 0]; 

Ts = 0.05; 
sys_dt = c2d(ss(A,B,C,D), Ts, 'zoh');
Ad = sys_dt.A; Bd = sys_dt.B; Cd = sys_dt.C; Dd = sys_dt.D;
n = size(Ad, 1);

% Prepare model for Simulink full-state feedback[cite: 8].
sys_x = ss(A, B, eye(n), 0);

% =========================================================================
% 2. INTEGRAL ACTION (Augmented System Definition)
% =========================================================================
% To achieve zero steady-state error against a constant step reference, 
% we augment the system matrices with an artificial integral state[cite: 8].
Aaug = [1, -Ts*Cd; 
        zeros(n,1), Ad];
Baug = [0; 
        Bd];
Caug = [0, Cd];
Daug = 0;

x0i = 0; % Initial condition for the integrator state[cite: 8]

% =========================================================================
% 3. LQR TUNING AND OBSERVABILITY CHECK
% =========================================================================
% Q(1,1) is the weight of the INTEGRAL state. A massive weight (25000) 
% forces the controller to eliminate the tracking error extremely fast (~2s)[cite: 8, 9].
Q = diag([25000, 1, 1]); 
R = 1;

Cq = chol(Q);
Mo = obsv(Aaug, Cq);
fprintf('Augmented Observability Rank: %d\n', rank(Mo));

% =========================================================================
% 4. COMPUTATION OF THE DISCRETE OPTIMAL CONTROL
% =========================================================================
Kaug = dlqr(Aaug, Baug, Q, R);

% Split the augmented gain into the integral part (Kq) and state feedback part (Kx)[cite: 8, 9].
Kq = Kaug(1);
Kx = Kaug(2:n+1);

disp('Integral Gain (Kq):'), disp(Kq);
disp('State Feedback Gain (Kx):'), disp(Kx);

% =========================================================================
% 5. SIMULATION SETUP (Simulink Integration)
% =========================================================================
t_sim = 4;

% Execute the Simulink model.
% IMPORTANT: Ensure the 'LAB04_Ex2.slx' file is in the same directory.
try
    out = sim('LAB04_Ex2');
catch
    warning('Simulink file ''LAB04_Ex2.slx'' not found. Simulation skipped.');
    return;
end

% =========================================================================
% 6. RESULTS AND PLOTS
% =========================================================================
figure(1)

% --- PLOT 1: Tracking Performance ---
subplot(1,2,1)
plot(out.r.time, out.r.data, 'k', 'LineWidth', 1.2) % Reference
grid on, zoom on, hold on
plot(out.y.time, out.y.data, 'b', 'LineWidth', 1.2) % Actual Output 

title('Tracking Performance (Integral Action)')
xlabel('Time [s]'), ylabel('y(t) , r(t) [m]')

% Specification Boundaries: Settling time (~2s) and 1% tolerance[cite: 7, 8]
yline(1.01, '--r', 'LineWidth', 0.8)
yline(0.99, '--r', 'LineWidth', 0.8)
xline(2, '--r', 'LineWidth', 0.8)

% --- PLOT 2: Control Effort and Constraints ---
subplot(1,2,2)
plot(out.u.time, out.u.data, 'b', 'LineWidth', 1.2)
grid on, zoom on, hold on

xlabel('Time [s]'), ylabel('Control Effort u(t) [N]')
title('Actuator Demand')

% Physical constraint of the actuator (|u| <= 1)[cite: 7, 8]
yline(1, 'r', 'Actuator Limit (+1)', 'LineWidth', 1.5)
yline(-1, 'r', 'Actuator Limit (-1)', 'LineWidth', 1.5)

% =========================================================================
% ANALYTICAL CONCLUSION (The Bridge to MPC)
% =========================================================================
% The aggressive tuning required to reach the target in 2 seconds forces 
% the unconstrained LQR to demand an initial force that severely exceeds 
% the physical limits of the actuator (the red lines).
% To respect the physical limits, the controller must be detuned, sacrificing speed.
% This limitation of LQR (inability to handle hard constraints natively) 
% is the fundamental motivation for Model Predictive Control (MPC)[cite: 8].
% =========================================================================