#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
 public:
  Vehicle();

  void Drive(const double dt);

  double x;
  double y;
  double psi;
  double v;
  double steer;
  double throttle;
};

#endif  // VEHICLE_H
