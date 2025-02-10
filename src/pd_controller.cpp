#include "pd_controller.hpp"

pd_controller::pd_controller() {
    kp << 1, 1, 1;
    kd << 0.2, 0.2, 0.2;

    curSt.setZero();
    setPt.setZero();
    prev_err.setZero();
}

Eigen::Vector3d pd_controller::update(const Eigen::Vector3d& current_state, const Eigen::Vector3d& setpoint) {
    curSt = current_state;
    setPt = setpoint;

    error = setPt - curSt;

    Eigen::Vector3d out = kp.cwiseProduct(error) + kd.cwiseProduct((error - prev_err) / dt);

    prev_err = error;

    return out;
}
