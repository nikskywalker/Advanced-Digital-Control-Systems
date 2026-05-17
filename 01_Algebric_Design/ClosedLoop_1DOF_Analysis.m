% ------------------------------------------------------------------------
% 1-DOF Discrete-Time Control System: Closed-Loop Analysis
% ------------------------------------------------------------------------
% OBJECTIVE:
% Analyze the internal stability, tracking error response, and steady-state
% behavior of a 1-DOF discrete-time feedback control system in the presence
% of reference signals and input/output disturbances.
% ------------------------------------------------------------------------
clear all
close all
clc

% =========================================================================
% 1. SYSTEM DEFINITION
% =========================================================================
Ts = 1;
z  = tf('z', Ts);

% Controller C(z): includes integral action (pole at z = 1)
C = (0.007 * (z + 0.6)) / (z - 1);

% Plant G(z)
G = (z + 0.5) / ((z - 0.7) * (z - 0.2));

% Open-loop transfer function L(z) = C(z) * G(z)
% Internal stability requires NO unstable pole-zero cancellations in L(z).
% Visual check: C has poles at {1}, zeros at {-0.6}.
%               G has poles at {0.7, 0.2}, zeros at {-0.5}.
%               No cancellations occur -> internal stability check passes.
L = zpk(series(C, G));

% =========================================================================
% 2. INTERNAL STABILITY ANALYSIS
% =========================================================================
% Closed-loop complementary sensitivity W(z) = L(z) / (1 + L(z))
% All poles of W(z) must satisfy |p| < 1 for the system to be stable.
W = zpk(minreal(L / (1 + L), 1e-3));

disp('--- Closed-Loop Poles ---');
disp('Magnitudes |p| (must all be < 1 for stability):');
disp(abs(pole(W)));

% =========================================================================
% 3. TRACKING ERROR RESPONSE e(k)
% =========================================================================
% Inputs (Z-domain):
%   r(k)  = 0.1 * step  ->  R(z)  = 0.1 * z/(z-1)
%   d1(k) = 0.2 * step  ->  D1(z) = 0.2 * z/(z-1)  (plant input disturbance)
R  = 0.1 * (z / (z - 1));
D1 = 0.2 * (z / (z - 1));

% Transfer functions from each input to output y
W_r  = zpk(minreal(L / (1 + L), 1e-3));       % r  -> y  (complementary sensitivity)
W_d1 = zpk(minreal(G / (1 + L), 1e-3));       % d1 -> y

% Total output Y(z) by superposition
Y = zpk(minreal(W_r * R + W_d1 * D1, 1e-3));

% Total error E(z) = R(z) - Y(z)
E = zpk(minreal(R - Y, 1e-3));

% Partial Fraction Expansion for manual z-antitransformation
disp('--- PFE Residues for Error e(k) ---');
[num, den] = tfdata(E, 'v');
[res, pol, k] = residuez(num, den)

% =========================================================================
% 4. STEADY-STATE ANALYSIS (FINAL VALUE THEOREM)
% =========================================================================
% Additional output disturbance d2(k) = step  ->  D2(z) = z/(z-1)
D2 = z / (z - 1);

% Sensitivity function W2(z): transfer from d2 to y
W_d2 = zpk(minreal(1 / (1 + L), 1e-3));       % d2 -> y

% Total output Y(z) with all three inputs active
Y_tot = zpk(minreal(W_r * R + W_d1 * D1 + W_d2 * D2, 1e-3));

% Steady-state value via Final Value Theorem:
%   y_inf = lim(z->1) [ (z-1)/z * Y(z) ]
% For a stable closed-loop and step inputs, this equals the DC gain at z = 1.
disp('--- Steady-State Output y_inf ---');
y_inf = dcgain(Y_tot)

% =========================================================================
% NOTE
% =========================================================================
% The integral action in C(z) (pole at z = 1) ensures zero steady-state
% error to step references and rejection of step output disturbances d2.
% The plant input disturbance d1 is attenuated but not fully rejected,
% as its path to the output does not pass through the integrator.
% =========================================================================