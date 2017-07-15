#include "vehicle_dynamics.h"

VehicleDynamics::VehicleDynamics() {}

// void VehicleDynamics::drive(const double dt) {
//  Eigen::VectorXd state(5);
//  state << x, y, psi, vy, yawr;
//  Eigen::VectorXd k1(5);
//  k1 = Derivatives(state, steer);
//  state = state + k1 * dt;
//  x = state[0];
//  y = state[1];
//  psi = state[2];
//  vy = state[3];
//  yawr = state[4];
//}

void VehicleDynamics::drive(const double dt) {
  Eigen::VectorXd state(5);
  state << x, y, psi, vy, yawr;

  Eigen::VectorXd k1 = Derivatives(state, steer);

  Eigen::VectorXd state1 = state + k1 / 2;
  Eigen::VectorXd k2 = Derivatives(state1, steer);

  Eigen::VectorXd state2 = state + k2 / 2;
  Eigen::VectorXd k3 = Derivatives(state2, steer);

  Eigen::VectorXd state3 = state + k3;
  Eigen::VectorXd k4 = Derivatives(state3, steer);

  Eigen::VectorXd ks = k1 + k2 * 2 + k3 * 3 + k4;

  state = state + ks * dt / 6.0;
  x = state[0];
  y = state[1];
  psi = state[2];
  vy = state[3];
  yawr = state[4];
}

// state: x,y,theta,vy,r
Eigen::VectorXd VehicleDynamics::Derivatives(Eigen::VectorXd &state,
                                             double input) {
  //  double x = state[0];
  //  double y = state[1];
  double psi = state[2];
  double vy = state[3];
  double r = state[4];

  double steer = input;
  double m = 1700;
  double lf = 2.67;
  double lr = 1.5;
  double cf = 500;  // TireCoefFront
  double cr = 500;

  double Izz = m * 1.8;  // 1/12.0 * m * (Length*Length+Width*Width);

  double A, B, C, D, E, F;
  double cos_f = cos(steer);
  double cos_psi = cos(psi);
  double sin_psi = sin(psi);

  A = -(cf * cos_f + cr) / (m * vx);
  B = (-lf * cf * cos_f + lr * cr) / (m * vx) - vx;
  C = (-lf * cf * cos_f + lr * cr) / (Izz * vx);
  D = -(lf * lf * cf * cos_f + cr * lr * lr) / (Izz * vx);
  E = cf * cos_f / m;
  F = lf * cf * cos_f / Izz;

  double x_dot = vx * cos_psi - vy * sin_psi;
  double y_dot = vx * sin_psi + vy * cos_psi;
  double psi_dot = r;
  double vy_dot = A * vy + C * r + E * steer;
  double r_dot = B * vy + D * r + F * steer;
  Eigen::VectorXd der(5);
  der << x_dot, y_dot, psi_dot, -vy_dot, r_dot;
  return der;
}
