#ifndef _PD_CONTROLLER_H
#define _PD_CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>

/**
 * @brief A Proportional-Derivative (PD) controller class.
 *
 * This class implements a simple PD controller that computes a control output
 * based on the current state and a desired setpoint. The control law uses constant
 * proportional and derivative gains.
 */
class pd_controller {
public:
    /**
     * @brief PD controller constructor.
     *
     * Initializes the proportional (kp) and derivative (kd) gains as well as
     * setting the current state, setpoint, and previous error vectors to zero.
     */
    pd_controller();

    /**
     * @brief Updates the PD controller output.
     *
     * This method computes the control output using the proportional-derivative (PD)
     * control law. It calculates the current error between the setpoint and current state,
     * then computes the output based on the proportional and derivative gains.
     *
     * @param current_state The current state (attitude angles or other relevant measure).
     * @param setpoint The desired state.
     * @return Eigen::Vector3d The computed control output.
     */
    Eigen::Vector3d update(const Eigen::Vector3d& current_state, const Eigen::Vector3d& setpoint);

private:
    const double dt = 0.1;  ///< Time step used for derivative computation.

    Eigen::Vector3d curSt;    ///< Current state vector.
    Eigen::Vector3d setPt;    ///< Desired setpoint vector.
    Eigen::Vector3d kp;       ///< Proportional gain vector.
    Eigen::Vector3d kd;       ///< Derivative gain vector.

    Eigen::Vector3d error;    ///< Current error vector (setpoint - current state).
    Eigen::Vector3d prev_err; ///< Previous error vector for derivative calculation.
};

#endif //_PD_CONTROLLER_H
