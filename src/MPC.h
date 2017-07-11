#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "utils.h"

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  const double Steer();
  const double Throttle();
  const WayPoints& Predictions();

 private:
  double steering_m;
  double throttle_m;
  WayPoints waypoints_m;
};

#endif /* MPC_H */
