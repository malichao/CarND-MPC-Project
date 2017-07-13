#ifndef UTILS_H
#define UTILS_H
#include <cppad/cppad.hpp>
#include <vector>
#include "MPC.h"
#include "vehicle.h"
#include "waypoints.h"

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double mph2ms(double x);
double ms2mph(double x);

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

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

double WrapHeading(const double& heading);
CppAD::AD<double> WrapHeading(const CppAD::AD<double>& heading);

// Get current time
double Now();

double Distance(const double& x1, const double& y1, const double& x2,
                const double& y2);

double Area(const double& a_x, const double& a_y, const double& b_x,
            const double& b_y, const double& c_x, const double& c_y);

double Curvature(const double& a_x, const double& a_y, const double& b_x,
                 const double& b_y, const double& c_x, const double& c_y);

// Linearly sample the polynomial line, unit in meter
WayPoints PolyToWaypoints(const Eigen::VectorXd& coeffs, const double& x_start,
                          const double& x_end, const double& spacing = 1.0);

void ProcessData(MPC& mpc, const WayPoints& waypoints, const Vehicle& veh,
                 MPCConfig& config);
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);
#endif  // UTILS_H
