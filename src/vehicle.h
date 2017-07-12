#ifndef VEHICLE_H
#define VEHICLE_H
#include "Eigen-3.3/Eigen/Core"
class Vehicle {
 public:
  Vehicle();

  void Drive(const double dt);
  double& X();
  double& Y();
  double& Psi();
  double& V();
  double& Steer();
  double& Throttle();
  double& Cte();
  double& Epsi();

  const double& X()const ;
  const double& Y()const ;
  const double& Psi()const ;
  const double& V()const ;
  const double& Steer()const ;
  const double& Throttle()const ;
  const double& Cte()const ;
  const double& Epsi()const ;

private:
  Eigen::VectorXd state_m;
};

#endif  // VEHICLE_H
