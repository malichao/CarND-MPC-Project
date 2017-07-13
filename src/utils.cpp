#include "utils.h"
#include <chrono>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2ms(double x) { return x * 0.447; }
double ms2mph(double x) { return x / 0.447; }

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evaluate a polynomial.
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x) {
  CppAD::AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

// Evaluate a polynomial slope.
double polyslope(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * pow(x, i - 1);
  }
  return result;
}

// Evaluate a polynomial slope.
CppAD::AD<double> polyslope(Eigen::VectorXd coeffs, CppAD::AD<double> x) {
  CppAD::AD<double> result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * CppAD::pow(x, i - 1);
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
  for (size_t i = 0; i < in_x.size(); i++) {
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
  for (size_t i = 0; i < in_x.size(); i++) {
    LocalToGlobal(veh_x, veh_y, veh_psi, in_x[i], in_y[i], out_x[i], out_y[i]);
  }
}

WayPoints PolyToWaypoints(const Eigen::VectorXd &coeffs, const double &x_start,
                          const double &x_end, const double &spacing) {
  WayPoints reference;
  for (double x = x_start; x < x_end; x += spacing) {
    reference.x.push_back(x);
    reference.y.push_back(polyeval(coeffs, x));
  }
  return reference;
}

void ProcessData(MPC &mpc, const WayPoints &waypoints, const Vehicle &veh,
                 MPCConfig &config) {
  // Step 1 Convert global waypoints into local frame
  WayPoints waypoints_local;
  Eigen::VectorXd ptsx_local(waypoints.x.size());
  Eigen::VectorXd ptsy_local(waypoints.x.size());
  Eigen::VectorXd state(6);
  GlobalToLocal(veh.X(), veh.Y(), veh.Psi(), waypoints.x, waypoints.y,
                waypoints_local.x, waypoints_local.y);
  waypoints_local.ToEigenVector(ptsx_local, ptsy_local);

  // Step 2 Fit the waypoints with 3rd order polynomial
  auto coeffs = polyfit(ptsx_local, ptsy_local, 3);
  double x_start = 0;
  double x_end = ptsx_local[ptsx_local.size() - 1] - ptsx_local[0];
  mpc.SetReference(PolyToWaypoints(coeffs, x_start, x_end, 2.0));
  const WayPoints &reference = mpc.Reference();

  // Step 3 Calculate the reference velocit based on road curvature
  double deviation = 0;
  for (int i = 1; i < reference.x.size(); i++) {
    deviation += fabs(atan2(reference.y[i] - reference.y[i - 1],
                            reference.x[i] - reference.x[i - 1]));
  }
  deviation /= reference.x.size();
  const double dev_max = 0.6, dev_min = 0.1;
  deviation = std::max(std::min(deviation, dev_max), dev_min);
  double v_max = config.vx_max, v_min = config.vx_min;
  double slope = (v_max - v_min) / (dev_max - dev_min);
  double v_ref = v_max - deviation * slope;
  v_ref = std::max(std::min(v_ref, v_max), v_min);
  config.ref_v = mph2ms(v_ref);
  //  printf("D %.3f V %.1f ", deviation, v_ref);

  //  double p1_x = 0;
  //  double p1_y = polyeval(coeffs, p1_x);
  //  double p2_x = 20;
  //  double p2_y = polyeval(coeffs, p2_x);
  //  double p3_x = 40;
  //  double p3_y = polyeval(coeffs, p2_x);
  //  double curv = Curvature(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y);
  //  double radius = 1 / fabs(curv);
  //  double acc_y_max = 5;
  //  double v_max = sqrt(acc_y_max / fabs(curv));
  //  v_max = std::max(std::min(v_max, 67.0), 29.0);
  //  config.ref_v = v_max;
  //  printf("C%.3f R %.1f V%.1f\n", curv, radius, ms2mph(v_max));

  // Step 4 Solve the optimization problem
  Vehicle veh_local(veh);
  veh_local.X() = 0;
  veh_local.Y() = 0;
  veh_local.Psi() = 0;
  veh_local.Cte() = polyeval(coeffs, 0);
  veh_local.Epsi() = 0 - atan(polyslope(coeffs, 0));
  //  state << 0, 0, 0, veh.v, cte, epsi;
  mpc.Solve(veh_local, coeffs);

  //  std::cout << "Steer, throttle = " << veh.steer << "," << veh.throttle <<
  //  "\n";
}

double Now() {
  auto now = std::chrono::system_clock::now().time_since_epoch();
  return now.count() / 1e9;
}

double WrapHeading(const double &heading) {
  double x = cos(heading);
  double y = sin(heading);
  return atan2(y, x);
}

CppAD::AD<double> WrapHeading(const CppAD::AD<double> &heading) {
  CppAD::AD<double> x = CppAD::cos(heading);
  CppAD::AD<double> y = CppAD::sin(heading);
  return CppAD::atan2(y, x);
}

double Distance(const double &x1, const double &y1, const double &x2,
                const double &y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Area(const double &a_x, const double &a_y, const double &b_x,
            const double &b_y, const double &c_x, const double &c_y) {
  double area = (b_x - a_x) * (c_y - a_y) - (b_y - a_y) * (c_x - a_x);
  return area;
}

double Curvature(const double &a_x, const double &a_y, const double &b_x,
                 const double &b_y, const double &c_x, const double &c_y) {
  double dist1 = Distance(a_x, a_y, b_x, b_y);
  double dist2 = Distance(b_x, b_y, c_x, c_y);
  double dist3 = Distance(c_x, c_y, a_x, a_y);
  double curv =
      4 * Area(a_x, a_y, b_x, b_y, c_x, c_y) / (dist1 * dist2 * dist3);
  return curv;
}
