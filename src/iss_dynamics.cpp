#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include "demo_aocs_control_model/pd_controller.hpp"
#include "demo_aocs_control_model/iss_dynamics.hpp"
#include "demo_aocs_control_model/L_p_func.h"

// Global vectors representing the current attitude state and the setpoint
Eigen::Vector3d curState;
Eigen::Vector3d setPt;

/**
 * @brief Structure representing the spacecraft's attitude state.
 *
 * This structure holds the angular velocity, Euler angles, and control 
 * surface deflections (deltas) for the spacecraft.
 */
struct state
{
    double p, q, r;          ///< Angular velocity components (rad/s)
    double phi, theta, psi;  ///< Euler angles (rad)
    Eigen::VectorXd deltas;  ///< Control surface deflections
};

/**
 * @brief Structure representing the time derivative of the attitude state.
 *
 * This structure holds the time derivatives of the angular velocity, Euler 
 * angles, and deflections.
 */
struct dotstate
{
    double dp, dq, dr;        ///< Derivatives of angular velocities
    double dphi, dtheta, dpsi;  ///< Derivatives of Euler angles
    Eigen::VectorXd ddeltas;   ///< Derivatives of control surface deflections
};

/**
 * @brief Structure containing spacecraft inertial properties and environmental parameters.
 *
 * This structure holds the moments of inertia, gravitational parameters, orbit 
 * radius, and external torque information.
 */
struct ssParm
{
    double Ixx, Iyy, Izz;    ///< Moments of inertia about principal axes
    double Ixy, Ixz, Iyz;    ///< Products of inertia
    double mu;               ///< Gravitational parameter (m^3/s^2)
    double rOrbit;           ///< Nominal orbit radius (m)
    Eigen::Vector3d T_cmg;   ///< Control moment gyro torque vector
};

/**
 * @brief Constructs and returns the inertia matrix from the parameters.
 *
 * @param par The spacecraft parameters containing inertia properties.
 * @return Eigen::Matrix<double,3,3> The inertia matrix.
 */
Eigen::Matrix<double,3,3> iMat(const ssParm& par)
{
    Eigen::Matrix<double,3,3> Ib;
    Ib << par.Ixx, par.Ixy, par.Ixz,
          par.Ixy, par.Iyy, par.Iyz,
          par.Ixz, par.Iyz, par.Izz;
    
    return Ib;
}

/**
 * @brief Computes the vector h based on control surface deflections.
 *
 * This function calculates a weighted sum of unit vectors defined by each 
 * control surface deflection.
 *
 * @param delta Vector of control surface deflections.
 * @return Eigen::Vector3d The computed h vector.
 */
Eigen::Vector3d compute_h(const Eigen::VectorXd& delta) {
    // Beta is a fixed angle in radians (54.73 degrees)
    constexpr double beta = 54.73 * M_PI / 180.0;

    // Extract individual deflections from the vector
    double delta_1 = delta(0);
    double delta_2 = delta(1);
    double delta_3 = delta(2);
    double delta_4 = delta(3);

    // Compute unit vectors for each control surface based on deflection angles
    Eigen::Vector3d r1, r2, r3, r4;
    r1 << -std::cos(beta) * std::sin(delta_1),
           std::cos(delta_1),
           std::sin(beta) * std::sin(delta_1);

    r2 << -std::cos(delta_2),
          -std::cos(beta) * std::sin(delta_2),
           std::sin(beta) * std::sin(delta_2);

    r3 <<  std::cos(beta) * std::sin(delta_3),
          -std::cos(delta_3),
           std::sin(beta) * std::sin(delta_3);

    r4 <<  std::cos(delta_4),
           std::cos(beta) * std::sin(delta_4),
           std::sin(beta) * std::sin(delta_4);

    // Sum the contributions and apply a scaling factor
    Eigen::Vector3d h = 10 * (r1 + r2 + r3 + r4);

    return h;
}

/**
 * @brief Computes the Jacobian matrix for the control surfaces.
 *
 * This function calls an external function L_p_func to compute the Jacobian 
 * matrix for the control surface deflections.
 *
 * @param delta Vector of control surface deflections.
 * @return Eigen::Matrix<double,4,3> The Jacobian matrix.
 */
Eigen::Matrix<double,4,3> jacobian(Eigen::VectorXd delta) {
    Eigen::Matrix<double,4,3> jacob(4,3);
    double out[12];
    L_p_func(delta(0), delta(1), delta(2), delta(3), out);

    jacob = Eigen::Map<Eigen::Matrix<double,4,3>>(out);
    return jacob;
}

/**
 * @brief Computes the gravity gradient torque.
 *
 * This function calculates the gravity gradient torque acting on the spacecraft
 * based on its attitude state and inertia properties.
 *
 * @param x The current attitude state.
 * @param par The spacecraft parameters including inertia and gravitational parameters.
 * @return Eigen::Vector3d The gravity gradient torque vector.
 */
Eigen::Vector3d gravityGradT(const state& x, const ssParm& par)
{
    // Mean motion calculated from gravitational parameter and orbit radius
    double n = std::sqrt(par.mu / std::pow(par.rOrbit, 3));
    double sp = std::sin(x.phi),   cp = std::cos(x.phi);
    double st = std::sin(x.theta), ct = std::cos(x.theta);

    // Torque components computed based on differences in moments of inertia
    double T1 = (par.Iyy - par.Izz) * st * ct;
    double T2 = (par.Izz - par.Ixx) * sp * cp;
    double T3 = (par.Ixx - par.Iyy) * sp * st;

    return 3.0 * n * n * Eigen::Vector3d(T1, T2, T3);
}

/**
 * @brief Computes the Euler kinematics transformation matrix.
 *
 * This matrix relates the angular velocity vector to the time derivatives 
 * of the Euler angles.
 *
 * @param phi Roll angle (rad).
 * @param theta Pitch angle (rad).
 * @return Eigen::Matrix<double,3,3> The Euler kinematics matrix.
 */
Eigen::Matrix<double,3,3> eulerKinMat(double phi, double theta)
{
    Eigen::Matrix<double,3,3> M;
    double sp = std::sin(phi), cp = std::cos(phi);
    double st = std::sin(theta), ct = std::cos(theta);
    double tt = st / ct;

    M << 1.0,     sp * tt,    cp * tt,
         0.0,     cp,         -sp,
         0.0, sp / ct,   cp / ct;

    return M;
}

/**
 * @brief Computes the time derivative of the attitude state.
 *
 * This function combines the dynamics of angular velocity, Euler angles, 
 * and control surface deflections. It calculates angular acceleration, 
 * Euler angle rates, and deflection rates.
 *
 * @param x The current attitude state.
 * @param par The spacecraft and environmental parameters.
 * @return Eigen::VectorXd A vector representing the state derivative.
 */
Eigen::VectorXd compute(const state& x, const ssParm& par)
{
    // Angular velocity vector
    Eigen::Vector3d w(x.p, x.q, x.r);
    
    // Inertia matrix and its inverse
    Eigen::Matrix<double,3,3> Ib = iMat(par);
    Eigen::Matrix<double,3,3> IbInv = Ib.inverse();

    // Compute external torques: gravity gradient and control moment gyro torque
    Eigen::Vector3d T_ext = gravityGradT(x, par);
    Eigen::Vector3d T_cmg = par.T_cmg;
    
    // Compute cross-product term for angular momentum dynamics
    Eigen::Vector3d crossTerm = w.cross(Ib * w);
    
    // Compute angular acceleration using Euler's equations for rotational motion
    Eigen::Vector3d w_dot = IbInv * (T_ext + T_cmg - crossTerm);

    // Compute Euler angle rates from angular velocity
    Eigen::Matrix<double,3,3> M = eulerKinMat(x.phi, x.theta);
    Eigen::Vector3d euler_dot = M * w;

    int nCmg = (int)x.deltas.size();

    // Compute Jacobian for the control surfaces and deflection derivatives
    Eigen::Matrix<double,4,3> jacob = jacobian(x.deltas);
    Eigen::Vector3d h = compute_h(x.deltas);
    // Calculate deflection rates based on control torques and cross-coupling effects
    Eigen::VectorXd delta_dots = jacob * (-(T_cmg + w.cross(h)));
    // Eigen::VectorXd delta_dots = Eigen::VectorXd::Zero(nCmg); // Alternative approach

    // Combine all derivatives into a single state derivative vector
    Eigen::VectorXd xDot(6 + nCmg);
    xDot(0) = w_dot(0);
    xDot(1) = w_dot(1);
    xDot(2) = w_dot(2);
    xDot(3) = euler_dot(0);
    xDot(4) = euler_dot(1);
    xDot(5) = euler_dot(2);
    for (int i = 0; i < nCmg; ++i)
        xDot(6 + i) = delta_dots(i);

    return xDot;
}

/**
 * @brief Advances the attitude state one time step using the Runge-Kutta 4th order method.
 *
 * This function performs numerical integration of the state using the RK4 scheme.
 *
 * @param x The current attitude state.
 * @param par The spacecraft and environmental parameters.
 * @param dt The time step for integration (s).
 * @return state The updated attitude state after one time step.
 */
state rk4Step(const state& x, const ssParm& par, double dt)
{
    // Lambda function to compute state derivative
    auto f = [&](const state& s) { return compute(s, par); };

    // Lambda to convert state to a vector form for RK4 integration
    auto stateToVec = [&](const state& s) {
        int nCmg = (int)s.deltas.size();
        Eigen::VectorXd v(6 + nCmg);
        v << s.p, s.q, s.r, s.phi, s.theta, s.psi, s.deltas;
        return v;
    };

    // Lambda to convert a vector back to the state structure
    auto vecToState = [&](const Eigen::VectorXd& v, const state& ref) {
        state s;
        s.p     = v(0);
        s.q     = v(1);
        s.r     = v(2);
        s.phi   = v(3);
        s.theta = v(4);
        s.psi   = v(5);
        int nCmg = (int)ref.deltas.size();
        s.deltas = v.segment(6, nCmg);
        return s;
    };

    // Convert current state to vector form
    Eigen::VectorXd xVec = stateToVec(x);

    // Compute RK4 slopes
    Eigen::VectorXd k1 = f(x);
    Eigen::VectorXd x2 = xVec + 0.5 * dt * k1;
    state s2 = vecToState(x2, x);
    Eigen::VectorXd k2 = f(s2);

    Eigen::VectorXd x3 = xVec + 0.5 * dt * k2;
    state s3 = vecToState(x3, x);
    Eigen::VectorXd k3 = f(s3);

    Eigen::VectorXd x4 = xVec + dt * k3;
    state s4 = vecToState(x4, x);
    Eigen::VectorXd k4 = f(s4);

    // Combine slopes to update the state vector
    Eigen::VectorXd xNext = xVec + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    return vecToState(xNext, x);
}

/**
 * @brief Main function for the attitude control simulation.
 *
 * This function sets up the initial state, parameters, and controller, and 
 * then runs a simulation loop using RK4 integration. Simulation data is 
 * saved to a CSV file.
 *
 * @return int Exit status code.
 */
int main()
{
    // Create an instance of the PD controller
    pd_controller controller;

    // Initialize the attitude state (angular velocities, Euler angles, and deltas)
    state x0;
    x0.p = 0.0;
    x0.q = 0.001;
    x0.r = 0.0;
    x0.phi = 0.0;
    x0.theta = 0.0;
    x0.psi = 0.0;
    x0.deltas = Eigen::VectorXd::Zero(4);

    // Set up the spacecraft and environmental parameters
    ssParm par;
    par.Ixx = 10.0;
    par.Iyy = 12.0;
    par.Izz = 9.0;
    par.Ixy = 0.0;
    par.Ixz = 0.0;
    par.Iyz = 0.0;
    par.mu = 3.986004418e14;
    par.rOrbit = 7.0e6;
    par.T_cmg << 1, 1, 0;

    // Integration settings
    double dt = 0.1;
    int steps = 7000;
    state x = x0;

    // Open CSV file for output
    std::ofstream file("ss_data.csv");
    file << "time,p,q,r,phi,theta,psi,delta1,delta2,delta3,delta4\n";

    // Simulation loop
    for (int i = 0; i <= steps; ++i)
    {
        double t = i * dt;
        // Write current time and state values to the CSV file
        file << t << "," 
             << x.p << "," << x.q << "," << x.r << ","
             << x.phi << "," << x.theta << "," << x.psi << ","
             << x.deltas(0) << "," << x.deltas(1) << "," << x.deltas(2) << "," << x.deltas(3) << "\n";

        // Update current attitude state and setpoint for controller
        curState << x.phi, x.theta, x.psi;
        setPt << 0.1, 0.2, 0.0;
        // Update the control torque using the PD controller
        par.T_cmg = controller.update(curState, setPt);
        
        // Propagate the state using RK4 integration
        if (i < steps)
            x = rk4Step(x, par, dt);
    }

    // Close the output file
    file.close();
    return 0;
}
