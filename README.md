# Advanced Digital Control Systems

This repository contains a collection of MATLAB and Simulink implementations covering advanced control strategies for linear systems, progressing from classical state-space methods to modern constrained optimization techniques.

## Repository Overview

The repository is structured as a progressive path in digital control:

1. **Pole Placement**: (Upcoming) State-feedback design via pole allocation.
2. **LQR Fundamentals & Tracking**: Infinite-horizon Linear Quadratic Regulation, including augmented state-space models for integral action and steady-state tracking.
3. **LQR Receding Horizon**: Finite-horizon optimal control and the transition towards Predictive Control, featuring Quadratic Programming (QP) to handle physical constraints.
4. **Model Predictive Control (MPC)**: Closed-loop MPC controllers with hard constraint enforcement on inputs and outputs, applied to open-loop unstable plants.

## Repository Structure

* `/02_LQR_Fundamentals_and_Tracking`: Zero regulation and integral tracking with Simulink schemes.
* `/03_LQR_Receding_Horizon`: Finite-horizon optimization and input/state saturation handling.
* `/04_Model_Predictive_Control`: Constrained output tracking and full state regulation via MPCtools.

## Requirements

* MATLAB (including Control System Toolbox and Optimization Toolbox).
* Simulink.
* MPCtools toolbox (required for the MPC module).
