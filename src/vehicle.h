#ifndef VEHICLE_H
#define VEHICLE_H
#include "Eigen-3.3/Eigen/Core"
class Vehicle {
 public:
  Vehicle();
  Vehicle(const double lf);

  void Drive(const double dt);

  void Drive2(const double dt);

  double& X();
  double& Y();
  double& Psi();
  double& V();
  double& Steer();
  double& Acc();
  double& Cte();
  double& Epsi();

  const double& X()const ;
  const double& Y()const ;
  const double& Psi()const ;
  const double& V()const ;
  const double& Steer()const ;
  const double& Acc()const ;
  const double& Cte()const ;
  const double& Epsi()const ;

private:
  Eigen::VectorXd state_m;
  double Lf_m;  //Wheelbase
};

#endif  // VEHICLE_H
