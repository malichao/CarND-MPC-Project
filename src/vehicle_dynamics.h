#ifndef VEHICLE_DYNAMICS_H
#define VEHICLE_DYNAMICS_H
#include "Eigen-3.3/Eigen/Core"

class VehicleDynamics {
 public:
  VehicleDynamics();
  void drive(const double dt);
  Eigen::VectorXd Derivatives(Eigen::VectorXd& state, double input);
  double x, y, psi, vx, vy, yawr, steer;
};

#endif  // VEHICLE_DYNAMICS_H
