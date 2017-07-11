#ifndef UTILS_H
#define UTILS_H
#include <cppad/cppad.hpp>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

struct WayPoints {
  std::vector<double> x;
  std::vector<double> y;

  void ToEigenVector(Eigen::VectorXd& vec_x, Eigen::VectorXd& vec_y,
                     int start_pos = 0);
};

struct Vehicle{
    double x;
    double y;
    double psi;
    double v;
    double steer;
    double throttle;
};

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

double polyslope(Eigen::VectorXd coeffs, double x);
CppAD::AD<double> polyslope(Eigen::VectorXd coeffs, CppAD::AD<double> x);

void rotate(const double& in_x, const double& in_y, double& out_x,
            double& out_y, const double& theta);

void GlobalToLocal(const double& veh_x, const double& veh_y,
                   const double& veh_psi, const double& in_x,
                   const double& in_y, double& out_x, double& out_y);

void GlobalToLocal(const double& veh_x, const double& veh_y,
                   const double& veh_psi, const std::vector<double>& in_x,
                   const std::vector<double>& in_y, std::vector<double>& out_x,
                   std::vector<double>& out_y);

void LocalToGlobal(const double& veh_x, const double& veh_y,
                   const double& veh_psi, const double& in_x,
                   const double& in_y, double& out_x, double& out_y);

void LocalToGlobal(const double& veh_x, const double& veh_y,
                   const double& veh_psi, const std::vector<double>& in_x,
                   const std::vector<double>& in_y, std::vector<double>& out_x,
                   std::vector<double>& out_y);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);
#endif  // UTILS_H
