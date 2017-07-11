#include "utils.h"
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evaluate a polynomial slope.
double polyslope(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i - 1);
  }
  return result;
}

// Evaluate a polynomial slope.
CppAD::AD<double> polyslope(Eigen::VectorXd coeffs, CppAD::AD<double> x) {
  CppAD::AD<double> result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i - 1);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

void rotate(const double &in_x, const double &in_y, double &out_x,
            double &out_y, const double &theta) {
  double s = sin(theta);
  double c = cos(theta);

  out_x = in_x * c - in_y * s;
  out_y = in_x * s + in_y * c;
}

void GlobalToLocal(const double &veh_x, const double &veh_y,
                   const double &veh_psi, const double &in_x,
                   const double &in_y, double &out_x, double &out_y) {
  double delta_x = in_x - veh_x;
  double delta_y = in_y - veh_y;
  rotate(delta_x, delta_y, out_x, out_y, -veh_psi);
}

void GlobalToLocal(const double &veh_x, const double &veh_y,
                   const double &veh_psi, const std::vector<double> &in_x,
                   const std::vector<double> &in_y, std::vector<double> &out_x,
                   std::vector<double> &out_y) {
  out_x.resize(in_x.size());
  out_y.resize(in_x.size());
  assert(in_x.size() == in_y.size() && in_x.size() == out_x.size() &&
         out_x.size() == out_y.size());
  for (int i = 0; i < in_x.size(); i++) {
    GlobalToLocal(veh_x, veh_y, veh_psi, in_x[i], in_y[i], out_x[i], out_y[i]);
  }
}

void LocalToGlobal(const double &veh_x, const double &veh_y,
                   const double &veh_psi, const double &in_x,
                   const double &in_y, double &out_x, double &out_y) {
  double out_x_, out_y_;
  rotate(in_x, in_y, out_x_, out_y_, veh_psi);
  out_x = out_x_ + veh_x;
  out_y = out_y_ + veh_y;
}

void LocalToGlobal(const double &veh_x, const double &veh_y,
                   const double &veh_psi, const std::vector<double> &in_x,
                   const std::vector<double> &in_y, std::vector<double> &out_x,
                   std::vector<double> &out_y) {
  out_x.resize(in_x.size());
  out_y.resize(in_x.size());
  assert(in_x.size() == in_y.size() && in_x.size() == out_x.size() &&
         out_x.size() == out_y.size());
  for (int i = 0; i < in_x.size(); i++) {
    LocalToGlobal(veh_x, veh_y, veh_psi, in_x[i], in_y[i], out_x[i], out_y[i]);
  }
}

void ProcessData(MPC &mpc, const WayPoints &waypoints, const Vehicle &veh) {
  WayPoints waypoints_local;
  Eigen::VectorXd ptsx_local(waypoints.x.size());
  Eigen::VectorXd ptsy_local(waypoints.x.size());
  Eigen::VectorXd state(6);
  GlobalToLocal(veh.x, veh.y, veh.psi, waypoints.x, waypoints.y,
                waypoints_local.x, waypoints_local.y);
  mpc.SetReference(waypoints_local);
  waypoints_local.ToEigenVector(ptsx_local, ptsy_local);
  auto coeffs = polyfit(ptsx_local, ptsy_local, 3);
  double cte = polyeval(coeffs, 0);
  double epsi = 0 - atan(polyslope(coeffs, 0));
  state << 0, 0, 0, veh.v, cte, epsi;
  mpc.Solve(state, coeffs);
  //  std::cout << "Steer, throttle = " << veh.steer << "," << veh.throttle <<
  //  "\n";
}

typedef std::vector<std::vector<int>> ImageArray;

