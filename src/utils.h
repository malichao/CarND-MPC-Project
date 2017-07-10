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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

double polyslope(Eigen::VectorXd coeffs, double x);
CppAD::AD<double> polyslope(Eigen::VectorXd coeffs, CppAD::AD<double> x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);
#endif  // UTILS_H
