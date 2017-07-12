#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "utils.h"

using CppAD::AD;
using namespace std;

class FG_eval {
 public:
  Eigen::VectorXd coeffs;
  const MPCConfig* config_m;
  // Coefficients of the fitted polynomial.
  FG_eval(Eigen::VectorXd coeffs, const MPCConfig& config) {
    this->coeffs = coeffs;
    config_m = &config;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    const size_t& x_start = config_m->x_start;
    const size_t& y_start = config_m->y_start;
    const size_t& psi_start = config_m->psi_start;
    const size_t& v_start = config_m->v_start;
    const size_t& cte_start = config_m->cte_start;
    const size_t& epsi_start = config_m->epsi_start;
    const size_t& delta_start = config_m->delta_start;
    const size_t& a_start = config_m->a_start;
    const size_t& N = config_m->N;
    const double dt = config_m->dt;
    const double ref_v = config_m->ref_v;
    const double Lf = config_m->Lf;

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += config_m->cte_w * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += config_m->epsi_w * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += config_m->v_w * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += config_m->delta_w * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += config_m->acc_w * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += config_m->delta_dot_w *
               CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += config_m->acc_dot_w *
               CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Setup Constraints

    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (size_t t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = polyeval(coeffs, x0);
      AD<double> psides0 = CppAD::atan(polyslope(coeffs, x0));

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition
//

MPC::MPC(const MPCConfig& config) { config_m = &config; }
MPC::~MPC() {}

void MPC::SetConfig(const MPCConfig& config) { config_m = &config; }

vector<double> MPC::Solve(const Vehicle& veh, Eigen::VectorXd coeffs) {
  const size_t& x_start = config_m->x_start;
  const size_t& y_start = config_m->y_start;
  const size_t& psi_start = config_m->psi_start;
  const size_t& v_start = config_m->v_start;
  const size_t& cte_start = config_m->cte_start;
  const size_t& epsi_start = config_m->epsi_start;
  const size_t& delta_start = config_m->delta_start;
  const size_t& a_start = config_m->a_start;
  const size_t& N = config_m->N;

  typedef CPPAD_TESTVECTOR(double) Dvector;

  const double& x = veh.X();
  const double& y = veh.Y();
  const double& psi = veh.Psi();
  const double& v = veh.V();
  const double& cte = veh.Cte();
  const double& epsi = veh.Epsi();
  const double& delta = veh.Steer();
  const double& acc = veh.Throttle();

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  vars[delta_start] = delta;
  vars[a_start] = acc;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = config_m->default_min;
    vars_upperbound[i] = config_m->default_max;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = config_m->delta_min;
    vars_upperbound[i] = config_m->delta_max;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = config_m->acc_min;
    vars_upperbound[i] = config_m->acc_max;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs, *config_m);

  // options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  //
  // Check some of the solution values
  //
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  if (!ok) std::cout << "Solution not found\n";
  //  auto cost = solution.obj_value;
  //  std::cout << "Cost " << cost << std::endl;
  steering_m = solution.x[delta_start];
  throttle_m = solution.x[a_start];
  prediction_m.x.resize(N);
  prediction_m.y.resize(N);
  prediction_m.psi.resize(N);
  for (size_t i = 0; i < N; i++) {
    prediction_m.x[i] = solution.x[x_start + i];
    prediction_m.y[i] = solution.x[y_start + i];
    prediction_m.psi[i] = solution.x[psi_start + i];
  }
  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};
}

void MPC::SetReference(const WayPoints& ref) { reference_m = ref; }

const double MPC::Steer() { return steering_m; }

const double MPC::Throttle() { return throttle_m; }

const WayPoints& MPC::Prediction() const { return prediction_m; }

const WayPoints& MPC::Reference() const { return reference_m; }

//
// Helper functions to fit and evaluate polynomials.
//
