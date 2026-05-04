% ------------------------------------------------------------------------
% PROBLEM 4: Receding Horizon with Input and State Constraints
% ------------------------------------------------------------------------
% OBJECTIVE:
% Simulate the Receding Horizon controller ensuring that NEITHER the actuator 
% limits NOR the physical position/velocity limits are ever violated.
% This introduces dynamic constraints that must be recalculated at each step.
% ------------------------------------------------------------------------

clear all
close all
clc

% =========================================================================
% 1. SYSTEM DEFINITION & WEIGHTS
% =========================================================================
A = [1 0.0488; 
     0 0.9512];
B = [0.0012; 
     0.0488];
x0 = [-0.8; 0];
Ts = 0.05;

Q = diag([100, 1]);
R = 1;

% =========================================================================
% 2. PREDICTION MATRICES & QUADRATIC FORMULATION
% =========================================================================
A_cal = [A; A^2; A^3];
B_cal = [[B zeros(2,1) zeros(2,1)]; 
         [A*B B zeros(2,1)]; 
         [(A^2)*B A*B B]];

Q_cal = blkdiag(Q, Q, Q);
R_cal = blkdiag(R, R, R);

H = 2 * (B_cal' * Q_cal * B_cal + R_cal);
H = (H + H') / 2; 
F = 2 * A_cal' * Q_cal * B_cal;

% =========================================================================
% 3. CONSTRAINT DEFINITION (The Static Parts)
% =========================================================================
% --- INPUT CONSTRAINTS (Gu and hu) ---
% Static because the physical motor limits never change.
u_sat = 0.6;
Gu = [ eye(3); 
      -eye(3)];
hu = u_sat * ones(6,1);

% --- STATE CONSTRAINTS (Gx and initial hx) ---
% Maximum allowed values for [Position; Velocity].
x_max = [0.01; 0.35];

% Gx is simply B_cal, because B_cal maps the inputs U to the future states.
Gx = B_cal;

% Master matrix G. This coefficient matrix will NOT change during simulation.
G = [Gu; Gx];

% =========================================================================
% 4. RECEDING HORIZON SIMULATION SETUP
% =========================================================================
x_traj(:,1) = x0;
x_k = x0;
sim_steps = 500;
options = optimoptions('quadprog', 'Display', 'off');

% =========================================================================
% 5. THE RECEDING HORIZON LOOP
% =========================================================================
for kk = 1:sim_steps
    
    % --- DYNAMIC CONSTRAINT UPDATE (The "Cliff" Analogy) ---
    % The controller needs to know the REMAINING SPACE before hitting the boundary.
    % Remaining Space (hx) = Boundary (x_max) - Autonomous State Evolution.
    % Since the system moves every step, this remaining space must be 
    % continuously recalculated to know how hard to brake.
    hx = repmat(x_max, 3, 1) - A_cal * x_k;
    
    % Stack static 'hu' and dynamic 'hx' to form the complete limit vector 'h'
    h = [hu; hx];
    
    % --- PLAN: Solve QP ---
    U = quadprog(H, x_k'*F, G, h, [], [], [], [], [], options);
    
    % --- SIMULATE REAL PLANT ---
    x_traj(:, kk+1) = A*x_k + B*U(1);
    u_traj(kk) = U(1);
    
    % --- UPDATE MEASUREMENT ---
    x_k = x_traj(:, kk+1);
end

% =========================================================================
% 6. AUTOMATIC EVALUATION OF ZERO REGULATION TIME (t_reg)
% =========================================================================
norm_x = sqrt(x_traj(1,:).^2 + x_traj(2,:).^2);
last_exceed_idx = find(norm_x > 1e-4, 1, 'last');
t_reg = last_exceed_idx * Ts;

fprintf('\n---> Constrained Zero Regulation Time (t_reg): %.4f [s] <---\n\n', t_reg);

% =========================================================================
% 7. PLOT RESULTS
% =========================================================================
figure(1)
subplot(311)
stairs([0:sim_steps]*Ts, x_traj(1,:)), grid on, hold on
yline(0.01, '-r', 'x_{max} limit', 'LineWidth', 1.5) 
ylabel('Position (x_1)'), xlabel('t (s)')

subplot(312)
stairs([0:sim_steps]*Ts, x_traj(2,:)), grid on, hold on
yline(0.35, '-r', 'x_{max} limit', 'LineWidth', 1.5) 
ylabel('Velocity (x_2)'), xlabel('t (s)')

subplot(313)
stairs([0:sim_steps]*Ts, norm_x), grid on, hold on
yline(1e-4, ':r', 'LineWidth', 1.5)
yline(-1e-4, ':r', 'LineWidth', 1.5)
ylabel('Norm ||x||_2'), xlabel('t (s)')

figure(2)
stairs([0:sim_steps-1]*Ts, u_traj), grid on, hold on
yline(0.6, '-r', 'u_{max} limit', 'LineWidth', 1.5)
yline(-0.6, '-r', 'u_{min} limit', 'LineWidth', 1.5)
ylabel('Control Effort (u)'), xlabel('t (s)')