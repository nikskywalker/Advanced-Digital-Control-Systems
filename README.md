# Advanced Digital Control Systems

This repository contains a collection of MATLAB and Simulink implementations covering advanced control strategies for linear systems, progressing from classical state-space methods to modern constrained optimization techniques.

## Repository Overview

The repository is structured as a progressive path in digital control:

1. **Algebraic Design**: Closed-loop analysis and algebraic controller design via the Diophantine equation, covering pole placement, plant pole cancellations, steady-state constraints, and unstable plants.
2. **LQR Fundamentals & Tracking**: Infinite-horizon Linear Quadratic Regulation, including augmented state-space models for integral action and steady-state tracking.
3. **LQR Receding Horizon**: Finite-horizon optimal control and the transition towards Predictive Control, featuring Quadratic Programming (QP) to handle physical constraints.
4. **Model Predictive Control (MPC)**: Closed-loop MPC controllers with hard constraint enforcement on inputs and outputs, applied to open-loop unstable plants.

## Repository Structure

* `/01_Algebraic_Design`: Closed-loop 1-DOF analysis and algebraic controller design (Diophantine equation, Sylvester matrix).
* `/02_LQR_Fundamentals_and_Tracking`: Zero regulation and integral tracking with Simulink schemes.
* `/03_LQR_Receding_Horizon`: Finite-horizon optimization and input/state saturation handling.
* `/04_Model_Predictive_Control`: Constrained output tracking and full state regulation via MPCtools.

## Requirements

* MATLAB (including Control System Toolbox and Optimization Toolbox).
* Simulink.
* MPCtools toolbox (required for the MPC module).
