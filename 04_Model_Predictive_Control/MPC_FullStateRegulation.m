% ------------------------------------------------------------------------
% MPC: Full State Regulation on an Unstable Plant
% ------------------------------------------------------------------------
% OBJECTIVE:
% Design an MPC controller to regulate the full state vector x back to the
% origin after an initial perturbation, on an open-loop unstable plant
% (poles at +/- 30). Both states are independently constrained and penalized.
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
A = [0   1 ;
     900 0];
B = [0  ;
    -10];
C = [600 0];
D = 0;

% The system starts at a perturbed state. The MPC must drive it back to [0; 0].
x0 = [-0.25 ; 0];
Ts = 0.001;   % Sampling period [s]

% Discretization via Zero-Order Hold
sys_dt = c2d(ss(A, B, C, D), Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sys_dt);

% Continuous-time plant with full state output (required by Simulink)
sys_x = ss(A, B, eye(2), 0);

% =========================================================================
% 2. MPCTOOLS SELECTOR MATRICES
% =========================================================================
Cy = eye(2);      % Measurement:          full state feedback assumed

% Both states must be regulated to zero: Cz and Cc are 2x2 identity matrices.
% This differs from output tracking, where only the scalar output y was controlled.
Cz = eye(2);
Dz = [0 ; 0];     % Must be 2x1 to match the 2 rows of Cz
Cc = eye(2);
Dc = Dz;

% =========================================================================
% 3. HORIZONS
% =========================================================================
% Same unstable plant as in the output tracking case.
% Hp and Hu must remain above the feasibility threshold.
Hp = 40;   % Prediction horizon [steps]
Hu = 40;   % Control horizon    [steps]
Hw = 1;    % Performance window start

z_blk = 1;
u_blk = 1;

% =========================================================================
% 4. COST FUNCTION WEIGHTS
% =========================================================================
% Cz has 2 rows, so Q must be a 2x2 matrix.
% The large weight on x1 (position) forces the controller to aggressively
% squash the position error, meeting the 0.4 s settling time requirement.
Q = diag([1700, 1]);   % Heavy penalty on x1, light penalty on x2
R = 1;                 % Penalty on control effort

cmode = 0;   % State feedback mode

% =========================================================================
% 5. CONSTRAINT DEFINITION
% =========================================================================
% [A] State constraints (unconstrained)
%     Cc is 2x2, so z_max and z_min must be 2x1 column vectors.
z_max = [ inf ;  inf];
z_min = [-inf ; -inf];

% [B] Input magnitude constraints
%     Wider saturation limits (+/- 30) compared to the tracking case:
%     larger authority is needed to catch the instability from a perturbed state.
u_max = [ 30];
u_min = [-30];

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
t_sim = 0.5;   % Total simulation time [s]

disp('Running simulation...');
out = sim('MPC_scheme_StateRegulation');
disp('Simulation complete.');

% =========================================================================
% 8. RESULTS
% =========================================================================
ts_target = 0.4;   % Target settling time [s]

% Euclidean norm of the state vector: ||x||_2 = sqrt(x1^2 + x2^2)
% Single scalar metric for how close the full state is to the origin.
norm_x = sqrt( out.x.signals.values(:,1).^2 + out.x.signals.values(:,2).^2 );

figure('Name', 'MPC — Full State Regulation', 'NumberTitle', 'off')

% --- State norm ---
subplot(1, 2, 1)
plot(out.x.time, norm_x, 'b', 'LineWidth', 1.5)
hold on; grid on; zoom on

yline( 1e-4, '--r', 'LineWidth', 0.8)
yline(-1e-4, '--r', 'LineWidth', 0.8)

xline(ts_target * 0.95, ':r', 'LineWidth', 1.2)
xline(ts_target * 1.05, ':r', 'LineWidth', 1.2)
ylim([-0.05, max(norm_x) * 1.1])

xlabel('t [s]'), ylabel('||x(t)||_2')
title('State Regulation (Euclidean Norm)')

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
% The asymmetric Q matrix (diag([1700, 1])) is the key tuning choice here:
% heavily penalizing x1 while tolerating larger x2 transients allows the
% controller to meet the 0.4 s settling time with the available actuator
% authority (+/- 30). Reducing the x1 weight causes the response to slow
% down; increasing it beyond a threshold yields diminishing returns as the
% actuator saturates.
% =========================================================================