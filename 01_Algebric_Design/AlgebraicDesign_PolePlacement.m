% ------------------------------------------------------------------------
% Algebraic Design of 1-DOF Digital Controllers (Pole Placement)
% ------------------------------------------------------------------------
% OBJECTIVE:
% Design discrete-time controllers C(z) via the Diophantine equation and
% Sylvester matrix, progressively covering: basic pole placement, plant
% pole cancellations, algebraic constraints, full design from continuous
% plants, steady-state ramp requirements, and unstable plant design.
% ------------------------------------------------------------------------
clear all
close all
clc

%%
% =========================================================================
% 1. BASIC POLE PLACEMENT (No Cancellations)
% =========================================================================
% Plant: G(z) = (z + 1.2) / ((z - 0.5)(z + 0.9))
% Target: assign 4 closed-loop poles; include integral action in C(z).
% -------------------------------------------------------------------------
clear all; close all; clc

z = tf('z', 1);

B = [1 1.2];
A = conv([1 -0.5], [1 +0.9]);
G = zpk(tf(B, A, 1));

% Desired closed-loop poles (all well inside the unit circle)
Am = poly([0.2+0.2i, 0.2-0.2i, 0.1+0.1i, 0.1-0.1i]);

% Diophantine equation: A_dioph * R1 + B_dioph * S = Am
% Adding integral action: R(z) = (z-1) * R1(z)  ->  A_dioph = (z-1)*A(z)
A_dioph = conv([1 -1], A);
B_dioph = B;

% Sylvester matrix (5x5: deg(Am) = 4, unknowns = R1[2] + S[3])
M_s = [ 1,     0,     0,   0,   0 ;
       -0.6,   1,     1,   0,   0 ;
       -0.85, -0.6,   1.2, 1,   0 ;
        0.45, -0.85,  0,   1.2, 1 ;
        0,     0.45,  0,   0,   1.2];

theta = M_s \ Am';

R1 = theta(1:2)';
S  = theta(3:5)';
R  = conv([1 -1], R1);
C  = zpk(tf(S, R, 1));

disp('--- [1] Basic Pole Placement ---');
disp('Controller C(z):'); disp(C)
disp('Closed-loop poles (magnitudes):'); disp(abs(pole(minreal(C*G/(1+C*G), 1e-3))))

%%
% =========================================================================
% 2. POLE PLACEMENT WITH PLANT POLE CANCELLATION
% =========================================================================
% Same plant. The z-plane map reveals which poles lie safely inside the
% stability cardioid and can therefore be cancelled by the controller.
%   A+(z): stable pole at z = +0.5  -> inside cardioid, safe to cancel
%   A-(z): pole at z = -0.9         -> near the boundary, keep it
% Cancellation reduces the Diophantine equation order (3 poles instead of 4).
% -------------------------------------------------------------------------
clear all; close all; clc

z = tf('z', 1);

A = conv([1 -0.5], [1 0.9]);
B = [1 1.2];
G = zpk(tf(B, A, 1));

% --- Z-plane analysis ---
figure('Name', '[2] Z-plane: Plant Poles/Zeros', 'NumberTitle', 'off')
pzmap(G, 'r')
axis('equal')
hold on
zgrid
title('Z-plane: Plant Poles and Zeros')

A_plus  = [1 -0.5];   % Cancelled: becomes a ZERO of S(z)
A_minus = [1 +0.9];   % Retained
B_plus  = 1;
B_minus = B;

% Reduced target polynomial (3 poles)
Am = poly([0.2+0.2i, 0.2-0.2i, 0.2]);

% Reduced Diophantine equation
A_dioph = conv([1 -1], A_minus);
B_dioph = B_minus;

M = [ 1,    0,    0,   0 ;
     -0.1,  1,    1,   0 ;
     -0.9, -0.1,  1.2, 1 ;
      0,   -0.9,  0,   1.2 ];

theta = M \ Am';

R1 = theta(1:2)';
S1 = theta(3:4)';

R = conv([1 -1], R1);
S = conv(A_plus, S1);   % A+ re-enters as zeros of C(z)
C = zpk(tf(S, R, 1));

disp('--- [2] Pole Placement with Cancellation ---');
disp('Controller C(z) (notice the zero at z = 0.5):'); disp(C)
disp('Closed-loop poles (magnitudes):'); disp(abs(pole(minreal(C*G/(1+C*G), 1e-3))))

%%
% =========================================================================
% 3. OVER-CONSTRAINED DESIGN (Algebraic Constraint)
% =========================================================================
% Same plant and factorization. Target polynomial has 4 poles (degree 4),
% but the unknowns (R1, S1) provide 6 degrees of freedom.
% The system is underdetermined: a 6th algebraic equation must be added
% to the Sylvester matrix to obtain a unique solution.
% -------------------------------------------------------------------------
clear all; close all; clc

z = tf('z', 1);

A = conv([1 -0.5], [1 0.9]);
B = [1 1.2];
G = zpk(tf(B, A, 1));

% --- Z-plane analysis ---
figure('Name', '[3] Z-plane: Plant Poles/Zeros', 'NumberTitle', 'off')
pzmap(G, 'r')
axis('equal')
hold on
zgrid
title('Z-plane: Plant Poles and Zeros')

A_plus  = [1 -0.5];
A_minus = [1 +0.9];
B_minus = B;

Am = poly([0.2+0.2i, 0.2-0.2i, 0.1+0.1i, 0.1-0.1i]);

A_dioph = conv([1 -1], A_minus);
B_dioph = B_minus;

% 6x6 Sylvester matrix: 5 polynomial rows + 1 algebraic constraint row
M = [ 1,    0,    0,    0,    0,    0 ;
     -0.1,  1,    0,    1,    0,    0 ;
     -0.9, -0.1,  1,    1.2,  1,    0 ;
      0,   -0.9, -0.1,  0,    1.2,  1 ;
      0,    0,   -0.9,  0,    0,    1.2 ;
      3.8,  3.8,  3.8, -2.2, -2.2, -2.2];

gamma = [Am'; 0];

theta = M \ gamma;

R1 = theta(1:3)';
S1 = theta(4:6)';
R  = conv([1 -1], R1);
S  = conv(A_plus, S1);
C  = zpk(tf(S, R, 1));

disp('--- [3] Over-Constrained Design ---');
disp('Controller C(z):'); disp(C)
disp('Closed-loop poles (magnitudes):'); disp(abs(pole(minreal(C*G/(1+C*G), 1e-3))))

%%
% =========================================================================
% 4. COMPLETE DESIGN FROM CONTINUOUS PLANT
% =========================================================================
% Plant: G(s) = 1 / (s(s+1))   [integrating, stable]
% Requirements: overshoot <= 10%, settling time (2%) <= 5.6 s
% Workflow: specs -> zeta, wn -> dominant poles in s-plane -> map to z-plane
%           -> z-plane analysis -> Diophantine equation -> C(z)
% -------------------------------------------------------------------------
clear all; close all; clc

s  = tf('s');
Ts = 0.15;

G_cont = 1 / (s * (s + 1));
G      = zpk(c2d(G_cont, Ts, 'zoh'));

[z_G, p_G, k_G] = zpkdata(G, 'v');
B = k_G * poly(z_G);
A = poly(p_G);

% Transient specifications -> zeta and wn
s_hat = 0.10;
ts_2  = 5.6;
zeta  = abs(log(s_hat)) / sqrt(pi^2 + (log(s_hat))^2);
wn    = 4 / (zeta * ts_2);

% Dominant poles: s-plane -> z-plane
p1c = -zeta*wn + 1i*wn*sqrt(1 - zeta^2);
p2c = conj(p1c);
p3c = -5 * zeta * wn;

p1d = exp(p1c * Ts);
p2d = exp(p2c * Ts);
p3d = exp(p3c * Ts);

Am = poly([p1d, p2d, p3d]);

% --- Z-plane analysis ---
figure('Name', '[4] Z-plane: Plant Poles/Zeros and Stability Cardioid', 'NumberTitle', 'off')
pzmap(G, 'r')
axis('equal')
hold on
zgrid(zeta, wn)
title('Z-plane: Plant Poles/Zeros and Stability Cardioid')

% Factorization
A_plus  = [1, -p_G(2)];
A_minus = [1, -p_G(1)];
B_minus = B;

A_dioph = A_minus;
B_dioph = B_minus;

% Steady-state constraint: |y_d1_inf| <= 0.85
y_d1_inf    = 0.85;
A_plus_at_1 = polyval(A_plus, 1);

MS = zeros(5, 5);
MS(1:4, 1) = [A_dioph(:); 0;    0   ];
MS(1:4, 2) = [0; A_dioph(:);    0   ];
MS(1:4, 3) = [0;    0;    A_dioph(:)];
MS(1:4, 4) = [0; B_dioph(:);    0   ];
MS(1:4, 5) = [0;    0;    B_dioph(:)];
MS(5, :)   = [1, 1, 1, -y_d1_inf*A_plus_at_1, -y_d1_inf*A_plus_at_1];

Gamma = [Am(:); 0];
theta = MS \ Gamma;

R       = theta(1:3)';
S_prime = theta(4:5)';
S       = conv(A_plus, S_prime);
C       = zpk(tf(S, R, Ts));

disp('--- [4] Complete Design from Continuous Plant ---');
disp('Controller C(z):'); disp(C)

%%
% =========================================================================
% 5. DESIGN WITH STEADY-STATE RAMP CONSTRAINT
% =========================================================================
% Plant: G(z) = (z+1.1) / ((z-1)(z-0.4))   [already has an integrator]
% Requirement: |y_d1_inf| <= 0.1 for d1(k) = k*ε(k)  (ramp disturbance)
% A ramp disturbance requires ℓ2 = 1 (pole at z=1 in C(z)).
% -------------------------------------------------------------------------
clear all; close all; clc

Ts = 1;
z  = tf('z', Ts);

G = (z + 1.1) / ((z - 1) * (z - 0.4));

[z_G, p_G, k_G] = zpkdata(G, 'v');
[B, A] = tfdata(G, 'v');

A_plus  = [1 -0.4];
A_minus = [1 -1];
B_plus  = 1;
B_minus = B;

A_dioph = conv([1 -1], [1 -1]);
B_dioph = B;

Am = poly([0.09, 0.10, 0.11, 0.12]);

Kc      = 10;
zeta_ss = 0.6;

M_S = [ [A_dioph, 0, 0, -Kc    ]', ...
        [0, A_dioph, 0, -Kc    ]', ...
        [0, 0, A_dioph, -Kc    ]', ...
        [B_dioph, 0, 0, zeta_ss]', ...
        [0, B_dioph, 0, zeta_ss]', ...
        [0, 0, B_dioph, zeta_ss]' ];

gamma = [Am, 0]';
theta = M_S \ gamma;

R2 = theta(1:3)';
S1 = theta(4:6)';
S  = conv(A_plus, S1);
R  = conv([1 -1], R2);
C  = zpk(tf(S, R, Ts));

L = C * G;
W = minreal(L / (1 + L), 1e-3);

disp('--- [5] Design with Steady-State Ramp Constraint ---');
disp('Controller C(z):'); disp(C)
disp('Closed-loop poles (magnitudes):'); disp(abs(pole(W)))

%%
% =========================================================================
% 6. DESIGN FOR AN UNSTABLE CONTINUOUS PLANT
% =========================================================================
% Plant: G(s) = 40 / (s^2 + 4s - 10)   [open-loop UNSTABLE: one RHP pole]
% Requirements: |y_d2_inf| <= 5e-4, |y_d1_inf| <= 5e-5, overshoot <= 25%,
%               settling time (1%) <= 2.5 s, rise time <= 0.9 s
% Ts = 5 ms
% Dependencies: Simulink model 'AlgebraicDesign_UnstablePlant_sim.slx'
% -------------------------------------------------------------------------
clear all; close all; clc

s  = tf('s');
Ts = 5e-3;

G_cont = 40 / (s^2 + 4*s - 10);
G      = zpk(c2d(G_cont, Ts, 'zoh'));

[z_G, p_G, k_G] = zpkdata(G, 'v');
[B, A] = tfdata(G, 'v');

Kg = dcgain(G);

% Transient specs -> zeta and wn
s_hat = 0.25;
ts_1  = 2.5;
tr    = 0.9;

zeta = abs(log(s_hat)) / sqrt(pi^2 + (log(s_hat))^2);
wn1  = 4.6 / (ts_1 * zeta);
wn2  = (0.12*zeta + 0.6) / tr;
wn   = max(wn1, wn2);

% --- Z-plane analysis ---
% The unstable pole (|p| > 1, outside the unit circle) is clearly visible
% outside the stability region. It CANNOT be cancelled under any circumstances.

figure('Name', '[6] Z-plane: Unstable Plant', 'NumberTitle', 'off')
pzmap(G, 'r')
axis('equal')
hold on
zgrid(zeta, wn)
title('Z-plane: Unstable Plant — Poles/Zeros and Stability Cardioid')

% Dominant s-plane poles -> z-plane
p1c = -zeta*wn + 1i*wn*sqrt(1 - zeta^2);
p2c = conj(p1c);
p3c = -35 * zeta * wn;
p4c = p3c;

p1d = exp(p1c * Ts);
p2d = exp(p2c * Ts);
p3d = exp(p3c * Ts);
p4d = exp(p4c * Ts);

Am = poly([p1d, p2d, p3d, p4d]);

% Factorization
% A-(z) retains the unstable pole: cancelling it would cause internal
% instability regardless of the closed-loop transfer function appearance.
A_plus  = [1, -p_G(2)];   % Stable pole: cancel
A_minus = [1, -p_G(1)];   % Unstable pole: MUST be retained
B_plus  = 1;
B_minus = B;

% Integral action required by steady-state specs (ℓ2 = 1)
A_dioph = conv([1 -1], A_minus);
B_dioph = B_minus;

% 6x6 Sylvester matrix
% The value 0.0283 is the steady-state gain constraint derived from the
% combined requirements on |y_d2_inf| and |y_d1_inf|.
M_S = [ [A_dioph, 0, 0, -1     ]', ...
        [0, A_dioph, 0, -1     ]', ...
        [0, 0, A_dioph, -1     ]', ...
        [B_dioph, 0, 0, 0.0283 ]', ...
        [0, B_dioph, 0, 0.0283 ]', ...
        [0, 0, B_dioph, 0.0283 ]' ];

gamma = [Am, 0]';
theta = M_S \ gamma;

R2 = theta(1:3)';
S1 = theta(4:6)';
S  = conv(S1, A_plus);
R  = conv([1 -1], conv(B_plus, R2));
C  = zpk(tf(S, R, Ts));

% Validation: closed-loop poles must match the roots of Am
L = C * G;
W = minreal(L / (1 + L), 1e-3);

disp('--- [6] Design for Unstable Continuous Plant ---');
disp('Controller C(z):'); disp(C)
disp('Closed-loop poles (should match Am roots):');
disp(pole(W))

% =========================================================================
% 6b. SIMULINK VALIDATION
% =========================================================================
% Closed-loop response with reference and disturbances active simultaneously.
% rho    -> step reference amplitude
% delta1 -> plant input disturbance amplitude
% delta2 -> plant output disturbance amplitude
% -------------------------------------------------------------------------
rho    = 1;
delta1 = 1;
delta2 = 1;
t_sim  = 5;

disp('Running Simulink validation...')
out = sim('AlgebraicDesign_UnstablePlant_sim');
disp('Simulation complete.')

figure('Name', '[6] Closed-Loop Step Response', 'NumberTitle', 'off')
plot(out.y.time, out.y.signals.values, 'b', 'LineWidth', 1.5)
grid on; zoom on; hold on
xlabel('t [s]'), ylabel('y(t)')
title('Closed-Loop Step Response — Unstable Plant')

% Overshoot limit (25%)
yline(1.25, 'r', 'LineWidth', 0.8)

% Settling time band (1%, ts <= 2.5 s)
yline(1 * 0.99, 'g', 'LineWidth', 0.8)
yline(1 * 1.01, 'g', 'LineWidth', 0.8)
xline(ts_1, 'g', 'LineWidth', 0.8)

% Rise time reference (tr <= 0.9 s)
xline(tr, 'k', 'LineWidth', 0.8)
yline(1, '--k', 'LineWidth', 0.8)

% =========================================================================
% NOTE
% =========================================================================
% The steady-state requirements determine the minimum number of integrators
% in C(z) (controller type ℓ2). The transient specs map to zeta and wn,
% which define the dominant closed-loop poles in the s-plane, then mapped
% to the z-plane via z = exp(s*Ts).
% The z-plane pzmap with zgrid overlay is a critical analysis step: it
% shows which plant poles lie inside the stability cardioid (safe to cancel)
% and which must be retained in A-(z). For unstable plants, the unstable
% pole must always be retained — cancelling it creates a hidden unstable
% mode regardless of the closed-loop transfer function appearance.
% =========================================================================