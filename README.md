# Attitude and Orbit Control System (AOCS) for Space Station OS: System Model, Controls & Filters

This repository contains the development of the Attitude and Orbit Control System (AOCS) for the Space Station OS subsystem. The work focuses on designing and simulating foundational components of the AOCS, ensuring precise control of the space station's attitude and orbital dynamics.

## Objectives
1. **System Modeling**: Develop mathematical models of the space station's 6DOF rotational and translational dynamics, accounting for external forces and environmental factors.
2. **Control System Design**: Implement stabilization and tracking non-linear optimal controllers
3. **State Estimation and Sensor Fusion**: Integrate sensor inputs using the Unscented Kalman Filter (UKF) for accurate state estimation.

## Current Focus
- Building body dynamics models using Newton-Euler equations.
  - Rotational dynamics with quaternions/DCM to avoid singularities.
  - Translational dynamics incorporating forces like gravity gradients and solar radiation pressure.
- Designing linear and non--linear controllers for attitude stabilization and tracking.
- Prioritizing system modeling as the foundation for further development.

## Build and Run
```bash
mkdir build && cd build
cmake ..
make
./iss_dynamics

# To vusulaise the dynamics
python ../scripts/vis_data.py
```

## Model with constant Torque input and untuned linear PD controller from Single Gimballed CMGs
![Constant Torque Input](assets/const_inp_T.png)

![PD Controller Input](assets/PD_untuned.png)

### More details regarding equations:
[Resources and Equations](resources/)

## Future Work
Subsequent phases will expand on nonlinear and adaptive control strategies, fault-tolerant mechanisms, and advanced functionalities aligned with real-world space station requirements.
