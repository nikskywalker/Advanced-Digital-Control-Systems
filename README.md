# Advanced Digital Control Systems

This repository contains a collection of MATLAB and Simulink implementations developed for the **Digital Control Technologies and Architectures (DiCTAr)** course. The project explores advanced control strategies for linear systems, transitioning from classical state-space methods to modern constrained optimization techniques.

## Project Overview

The repository is structured to follow a progressive learning path in digital control:

1. **Pole Placement**: (Upcoming) State-feedback design via pole allocation.
2. **LQR Fundamentals & Tracking**: Infinite-horizon Linear Quadratic Regulation, including augmented state-space models for integral action and steady-state tracking.
3. **LQR Receding Horizon**: Finite-horizon optimal control and the transition towards Predictive Control, featuring Quadratic Programming (QP) to handle physical constraints.
4. **Model Predictive Control (MPC)**: (Upcoming) Advanced predictive strategies with explicit constraint handling.

## Key Features

* **Optimal Control:** Implementation of Discrete Algebraic Riccati Equations (DARE) and quadratic cost function minimization[cite: 8].
* **Constraint Handling:** Use of `quadprog` to manage hard physical limits on actuators and states[cite: 10, 11].
* **Hybrid Workflows:** Seamless integration between MATLAB scripting for controller synthesis and Simulink for non-linear plant simulation[cite: 8].
* **Receding Horizon Principle:** Closing the loop by solving optimization problems at each sampling instant[cite: 11, 12].

## Repository Structure

Each directory contains specialized scripts and detailed documentation:
* `/02_LQR_Fundamentals_and_Tracking`: Focuses on zero regulation and integral tracking with Simulink schemes.
* `/03_LQR_Receding_Horizon`: Focuses on finite-horizon optimization and input/state saturation handling.

## Requirements

* MATLAB (including Control System Toolbox and Optimization Toolbox).
* Simulink.
