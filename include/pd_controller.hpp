#ifndef _PD_CONTROLLER_H
#define _PD_CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>

class pd_controller {
public:
    pd_controller();

    Eigen::Vector3d update(const Eigen::Vector3d& current_state, const Eigen::Vector3d& setpoint);

private:
    const double dt = 0.1;

    Eigen::Vector3d curSt;
    Eigen::Vector3d setPt;
    Eigen::Vector3d kp;
    Eigen::Vector3d kd;

    Eigen::Vector3d error;
    Eigen::Vector3d prev_err;
};

#endif //_PD_CONTROLLER_H
