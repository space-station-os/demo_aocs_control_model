#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include "pd_controller.hpp"
#include "iss_dynamics.hpp"
#include "L_p_func.h"

struct state
{
    double p, q, r;
    double phi, theta, psi;
    Eigen::VectorXd deltas;
};

struct dotstate
{
    double dp, dq, dr;
    double dphi, dtheta, dpsi;
    Eigen::VectorXd ddeltas;
};

struct ssParm
{
    double Ixx, Iyy, Izz;
    double Ixy, Ixz, Iyz;
    double mu;
    double rOrbit;
    Eigen::Vector3d T_cmg;
};

Eigen::Matrix3d iMat(const ssParm& par)
{
    Eigen::Matrix3d Ib;
    Ib << par.Ixx, par.Ixy, par.Ixz,
          par.Ixy, par.Iyy, par.Iyz,
          par.Ixz, par.Iyz, par.Izz;
    
    return Ib;
}

Eigen::MatrixXd jacobian(Eigen::VectorXd delta) {
    Eigen::MatrixXd jacob(4,3);
    double out[12];
    L_p_func(delta(0),delta(1),delta(2),delta(3),out);

    jacob = Eigen::Map<Eigen::Matrix<double,4,3>>(out);
    return jacob;
}

Eigen::Vector3d gravityGradT(const state& x, const ssParm& par)
{
    double n = std::sqrt(par.mu / std::pow(par.rOrbit, 3));
    double sp = std::sin(x.phi),   cp = std::cos(x.phi);
    double st = std::sin(x.theta), ct = std::cos(x.theta);

    double T1 = (par.Iyy - par.Izz) * st * ct;
    double T2 = (par.Izz - par.Ixx) * sp * cp;
    double T3 = (par.Ixx - par.Iyy) * sp * st;

    return 3.0 * n * n * Eigen::Vector3d(T1, T2, T3);
}

Eigen::Matrix3d eulerKinMat(double phi, double theta)
{
    Eigen::Matrix3d M;
    double sp = std::sin(phi), cp = std::cos(phi);
    double st = std::sin(theta), ct = std::cos(theta);
    double tt = st / ct;

    M << 1.0,     sp * tt,    cp * tt,
         0.0,     cp,         -sp,
         0.0, sp / ct,   cp / ct;

    return M;
}

Eigen::VectorXd compute(const state& x, const ssParm& par)
{
    Eigen::Vector3d w(x.p, x.q, x.r);
    Eigen::Matrix3d Ib = iMat(par);
    Eigen::Matrix3d IbInv = Ib.inverse();

    Eigen::Vector3d T_ext = gravityGradT(x, par);
    Eigen::Vector3d T_cmg = par.T_cmg;
    Eigen::Vector3d crossTerm = w.cross(Ib * w);
    Eigen::Vector3d w_dot = IbInv * (T_ext + T_cmg - crossTerm);

    Eigen::Matrix3d M = eulerKinMat(x.phi, x.theta);
    Eigen::Vector3d euler_dot = M * w;

    int nCmg = (int)x.deltas.size();

    // TODO: Add Jacobian overall calculation
    jacobian(x.deltas);
    Eigen::VectorXd delta_dots = Eigen::VectorXd::Zero(nCmg);

    Eigen::VectorXd xDot(6 + nCmg);
    xDot(0) = w_dot(0);
    xDot(1) = w_dot(1);
    xDot(2) = w_dot(2);
    xDot(3) = euler_dot(0);
    xDot(4) = euler_dot(1);
    xDot(5) = euler_dot(2);
    for(int i = 0; i < nCmg; ++i) xDot(6 + i) = delta_dots(i);

    return xDot;
}

state rk4Step(const state& x, const ssParm& par, double dt)
{
    auto f = [&](const state& s){ return compute(s, par); };

    auto stateToVec = [&](const state& s){
        int nCmg = (int)s.deltas.size();
        // std::cout << s.deltas.size();
        Eigen::VectorXd v(6 + nCmg);
        v << s.p, s.q, s.r, s.phi, s.theta, s.psi, s.deltas;
        return v;
    };

    auto vecToState = [&](const Eigen::VectorXd& v, const state& ref){
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

    Eigen::VectorXd xVec = stateToVec(x);

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

    Eigen::VectorXd xNext = xVec + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

    return vecToState(xNext, x);
}

int main()
{
    state x0;
    x0.p = 0.0;
    x0.q = 0.001;
    x0.r = 0.0;
    x0.phi = 0.0;
    x0.theta = 0.0;
    x0.psi = 0.0;
    x0.deltas = Eigen::VectorXd::Zero(4);

    ssParm par;
    par.Ixx = 10.0;
    par.Iyy = 12.0;
    par.Izz = 9.0;
    par.Ixy = 0.0;
    par.Ixz = 0.0;
    par.Iyz = 0.0;
    par.mu = 3.986004418e14;
    par.rOrbit = 7.0e6;
    par.T_cmg = Eigen::Vector3d::Zero();

    double dt = 0.1;
    int steps = 10000;
    state x = x0;

    std::ofstream file("ss_data.csv");
    file << "time,p,q,r,phi,theta,psi,delta1,delta2,delta3,delta4\n";

    for(int i = 0; i <= steps; ++i)
    {
        double t = i * dt;
        file << t << "," 
             << x.p << "," << x.q << "," << x.r << ","
             << x.phi << "," << x.theta << "," << x.psi << ","
             << x.deltas(0) << "," << x.deltas(1) << x.deltas(2) << x.deltas(3) << "\n";

        if(i < steps) x = rk4Step(x, par, dt);
    }

    file.close();
    return 0;
}
