#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "mpc_config.h"
#include "vehicle.h"
#include "waypoints.h"

class MPC {
 public:
  MPC(const MPCConfig& config);

  virtual ~MPC();

  void SetConfig(const MPCConfig& config);

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(const Vehicle& veh, Eigen::VectorXd coeffs);

  void SetReference(const WayPoints& ref);

  const double Steer();

  const double Acc();

  const double Cost();

  // Predicted vehicle trajectory in vehicle's LOCAL coordinate
  const WayPoints& Prediction() const;

  // Reference trajectory in vehicle's LOCAL coordinate
  const WayPoints& Reference() const;

 private:
  double steering_m;
  double acc_m;
  double cost_m;
  WayPoints prediction_m, reference_m;
  const MPCConfig* config_m;
};

#endif /* MPC_H */
