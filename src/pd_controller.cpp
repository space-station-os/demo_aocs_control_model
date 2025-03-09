#include "demo_aocs_control_model/pd_controller.hpp"

pd_controller::pd_controller() {
    // Initialize proportional gains (kp) for each axis
    kp << 1, 1, 1;
    
    // Initialize derivative gains (kd) for each axis
    kd << 0.2, 0.2, 0.2;

    // Set the current state, setpoint, and previous error to zero vectors
    curSt.setZero();
    setPt.setZero();
    prev_err.setZero();
}

Eigen::Vector3d pd_controller::update(const Eigen::Vector3d& current_state, const Eigen::Vector3d& setpoint) {
    // Update internal state with the provided inputs
    curSt = current_state;
    setPt = setpoint;

    // Compute error between desired state and current state
    error = setPt - curSt;

    // Calculate derivative of error by subtracting the previous error and dividing by dt
    // The control output is a sum of the proportional term and the derivative term
    Eigen::Vector3d out = kp.cwiseProduct(error) + kd.cwiseProduct((error - prev_err) / dt);

    // Update previous error for the next iteration
    prev_err = error;

    return out;
}
