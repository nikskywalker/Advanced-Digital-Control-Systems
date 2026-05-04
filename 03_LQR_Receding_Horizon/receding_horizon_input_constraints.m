% ------------------------------------------------------------------------
% PROBLEM 3: Receding Horizon Control with Input Constraints
% ------------------------------------------------------------------------
% OBJECTIVE:
% Simulate the Receding Horizon controller for a 2D system.
% The algorithm computes an optimal 3-step sequence at each iteration,
% but applies ONLY the first move to the physical system (closing the loop).
% ------------------------------------------------------------------------

clear all
close all
clc

% =========================================================================
% 1. SYSTEM DEFINITION
% =========================================================================
A = [1 0.0488; 
     0 0.9512];         % 2x2 matrix
B = [0.0012; 
     0.0488];           % 2x1 column vector
x0 = [-0.8; 0];         % Initial state vector [Position; Velocity]
Ts = 0.05;              % Sampling time

% =========================================================================
% 2. COST FUNCTION WEIGHTS
% =========================================================================
Q = diag([100, 1]);     % Weighting position error much more than velocity
R = 1;                  % Control effort weight

% =========================================================================
% 3. PREDICTION MATRICES (The Controller's "Imagination")
% =========================================================================
% A_cal and B_cal are used by the algorithm to look 3 steps into the future.
A_cal = [A; 
         A^2; 
         A^3];

B_cal = [[B        zeros(2,1) zeros(2,1)]; 
         [A*B      B          zeros(2,1)]; 
         [(A^2)*B  A*B        B         ]];

Q_cal = blkdiag(Q, Q, Q);
R_cal = blkdiag(R, R, R);

% =========================================================================
% 4. QUADRATIC FORMULATION (H and F)
% =========================================================================
H = 2 * (B_cal' * Q_cal * B_cal + R_cal);
H = (H + H') / 2; % Force exact symmetry for quadprog
F = 2 * A_cal' * Q_cal * B_cal;

% =========================================================================
% 5. INPUT CONSTRAINT DEFINITION
% =========================================================================
u_sat = 0.6; % The physical limit of the actuator

G = [ eye(3); 
     -eye(3)];
h = u_sat * ones(6,1);

% =========================================================================
% 6. RECEDING HORIZON SIMULATION SETUP
% =========================================================================
x_k = x0;             % CURRENT MEASURED STATE (The "GPS Blue Dot")
x_traj(:,1) = x0;     % History log for plotting
sim_steps = 500;      % 500 steps * 0.05s = 25 seconds of simulation

% =========================================================================
% 7. THE RECEDING HORIZON LOOP
% =========================================================================
options = optimoptions('quadprog', 'Display', 'off');

for kk = 1:sim_steps
    
    % --- PLAN: Solve QP ---
    % Find the optimal 3-step U sequence honoring G*U <= h.
    U = quadprog(H, x_k'*F, G, h, [], [], [], [], [], options);
    
    % --- SIMULATE REAL PLANT (The Physical Step) ---
    % We apply ONLY the first computed move: U(1).
    x_traj(:, kk+1) = A*x_k + B*U(1);
    u_traj(kk) = U(1);
    
    % --- UPDATE MEASUREMENT ---
    % The physical system has moved. We update the current state measurement
    % so the next loop plans from the new correct position.
    x_k = x_traj(:, kk+1);
end

% =========================================================================
% 8. AUTOMATIC EVALUATION OF ZERO REGULATION TIME (t_reg)
% =========================================================================
norm_x = sqrt(x_traj(1,:).^2 + x_traj(2,:).^2);
last_exceed_idx = find(norm_x > 1e-4, 1, 'last');
t_reg = last_exceed_idx * Ts;

fprintf('\n---> Zero Regulation Time (t_reg): %.4f [s] <---\n\n', t_reg);

% =========================================================================
% 9. PLOT RESULTS
% =========================================================================
figure(1)
subplot(311)
stairs([0:sim_steps]*Ts, x_traj(1,:)), grid on, hold on
ylabel('Position (x_1)'), xlabel('t (s)')

subplot(312)
stairs([0:sim_steps]*Ts, x_traj(2,:)), grid on, hold on
ylabel('Velocity (x_2)'), xlabel('t (s)')

subplot(313)
stairs([0:sim_steps]*Ts, norm_x), grid on, hold on
yline(1e-4, ':r'), yline(-1e-4, ':r')
ylabel('Norm ||x||_2'), xlabel('t (s)')

figure(2)
stairs([0:sim_steps-1]*Ts, u_traj), grid on, hold on
yline(0.6, '-r'), yline(-0.6, '-r')
ylabel('Control Effort (u)'), xlabel('t (s)')