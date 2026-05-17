% ------------------------------------------------------------------------
% MPC: Constrained Output Tracking on an Unstable Plant
% ------------------------------------------------------------------------
% OBJECTIVE:
% Design an MPC controller to track a unit step reference for an open-loop
% unstable second-order system (poles at +/- 30), enforcing hard constraints
% on actuator magnitude and output overshoot via Quadratic Programming.
% ------------------------------------------------------------------------
% DEPENDENCIES: MPCtools toolbox, Simulink model 'MPC_scheme.slx'
% ------------------------------------------------------------------------
clear all
close all
clc

% =========================================================================
% 1. PLANT DEFINITION & DISCRETIZATION
% =========================================================================
% Second-order system with open-loop unstable poles at s = +/- 30.
% The strong instability sets a strict lower bound on the prediction horizon.
A = [0   1 ;
     900 0];
B = [0  ;
    -10];
C = [600 0];
D = 0;

x0 = [0 ; 0];   % Initial condition: system starts at equilibrium
Ts = 0.001;     % Sampling period [s]

% Discretization via Zero-Order Hold
sys_dt = c2d(ss(A, B, C, D), Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sys_dt);

% Continuous-time plant with full state output (required by Simulink)
sys_x = ss(A, B, eye(2), 0);

% =========================================================================
% 2. MPCTOOLS SELECTOR MATRICES
% =========================================================================
Cy = eye(2);  % Measurement:          full state feedback assumed
Cz = Cd;      % Controlled variable:  physical output y tracks the reference
Dz = 0;
Cc = Cd;      % Constrained variable: output limits applied to y
Dc = 0;

% =========================================================================
% 3. HORIZONS
% =========================================================================
% The plant diverges in roughly 0.03 s. The prediction horizon must be
% long enough to anticipate this divergence and compute a stabilizing input.
%
%   Hp = 40, Hu = 40  ->  stable, feasible solution
%   Hp = 35, Hu = 18  ->  critical threshold (marginal feasibility)
%   Hp < 35           ->  controller becomes myopic; QP turns infeasible
Hp = 40;   % Prediction horizon [steps]
Hu = 40;   % Control horizon    [steps]
Hw = 1;    % Performance window start

z_blk = 1; % New control move computed at every step
u_blk = 1;

% =========================================================================
% 4. COST FUNCTION WEIGHTS
% =========================================================================
Q = 1;     % Penalty on tracking error (z - r)
R = 1;     % Penalty on control effort (u)

cmode = 0; % State feedback mode

% =========================================================================
% 5. CONSTRAINT DEFINITION
% =========================================================================
% [A] Output constraints
%     Hard ceiling at 1.01 enforces a maximum 1% overshoot.
%     The solver rejects any trajectory that violates this bound.
z_max = [1.01];
z_min = [-inf];

% [B] Input magnitude constraints (actuator saturation)
u_max = [ 0.5];
u_min = [-0.5];

% [C] Input rate constraints (unconstrained)
du_max = [ inf];
du_min = [-inf];

% =========================================================================
% 6. CONTROLLER INITIALIZATION
% =========================================================================
disp('Initializing MPC controller...');

md = MPCInit(Ad, Bd, Cy, Cz, Dz, Cc, Dc,      ...
             Hp, Hw, z_blk, Hu, u_blk,          ...
             du_max, du_min, u_max, u_min,       ...
             z_max, z_min, Q, R, [], [], Ts, cmode, 'qp_as');

% =========================================================================
% 7. SIMULATION
% =========================================================================
t_sim = 0.2;   % Total simulation time [s]

disp('Running simulation...');
out = sim('MPC_scheme_OutputTracking');
disp('Simulation complete.');

% =========================================================================
% 8. RESULTS
% =========================================================================
ts_target = 0.035;   % Target settling time [s]

figure('Name', 'MPC — Constrained Output Tracking', 'NumberTitle', 'off')

% --- Output response ---
subplot(1, 2, 1)
plot(out.y.time, out.y.signals.values, 'b', 'LineWidth', 1.5)
hold on; grid on; zoom on

yline(z_max, '--r', 'y_{max} (1% overshoot limit)', 'LineWidth', 1.2)
yline(0.99,  '--r', 'LineWidth', 1.2)

xline(ts_target * 0.95, ':g', 'LineWidth', 1.2)
xline(ts_target * 1.05, ':g', 'LineWidth', 1.2)
patch([ts_target*0.95, ts_target*1.05, ts_target*1.05, ts_target*0.95], ...
      [0, 0, 1.2, 1.2], 'g', 'FaceAlpha', 0.08, 'EdgeColor', 'none')

xlabel('t [s]'), ylabel('y(t)')
title('Output Tracking')
legend('y(t)', 'Overshoot limit', '', 'Location', 'southeast')

% --- Control effort ---
subplot(1, 2, 2)
stairs(out.u.time, out.u.signals.values, 'b', 'LineWidth', 1.5)
hold on; grid on; zoom on

yline( u_max, '--r', 'u_{max}', 'LineWidth', 1.2)
yline( u_min, '--r', 'u_{min}', 'LineWidth', 1.2)
ylim([u_min*1.2, u_max*1.2])

xlabel('t [s]'), ylabel('u(t)')
title('Control Effort')

% =========================================================================
% NOTE
% =========================================================================
% The controller stabilizes the plant and enforces both the overshoot ceiling
% (1.01) and the actuator saturation (+/- 0.5). The settling time target
% (0.035 s) is not met: with |u| <= 0.5 the actuator lacks the authority
% to drive the output into the target window faster. This is the physically
% optimal solution under the given constraints.
% =========================================================================