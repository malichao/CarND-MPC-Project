#include "vehicle.h"

Vehicle::Vehicle() : state_m(8), Lf_m(2.67) {}
Vehicle::Vehicle(const double lf) : state_m(8), Lf_m(lf) {}

void Vehicle::Drive(const double dt) {
  X() = X() + V() * cos(Psi()) * dt;
  Y() = Y() + V() * sin(Psi()) * dt;
  Psi() = Psi() + V() / Lf_m * Steer() * dt;
  V() = V() + Acc() * dt;
}

// void Vehicle::Drive2(const double dt) {
//  double steer = Steer();
//  double m = 1600;
//  double lf = Lf_m;
//  double lr = Lf_m;
//  double cf = 1.5E5;  // TireCoefFront
//  double cr = 4E4;
//  double tr = 0.35;    // Tire radius
//  double Izz = m * 2;  // 1/12.0 * m * (Length*Length+Width*Width);
//  double v = V();
//  double psi = Psi();
//  //  v = std::max(v, 0.01);

//  double cos_f = cos(steer);

//  double a11 = -(cf * cos_f + cr) / (m * v);
//  double a12 = (-lf * cf * cos_f + lr * cr) / (m * v) - v;
//  double a21 = (-lf * cf * cos_f + lr * cr) / (Izz * v);
//  double a22 = -(lf * lf * cf * cos_f + lr * lr * cr) / (Izz * v);
//  double b11 = cf * cos_f / m;
//  double b12 = lf * cf * cos_f / Izz;

//  double vy_dot = a11 * vy + a21 * r + b11 * steer;
//  double r_dot = a12 * vy + a22 * r + b12 * steer;
//  double x_dot = v * cos(psi) - vy * sin(psi);
//  double y_dot = v * sin(psi) + vy * cos(psi);
//  double psi_dot = r;

//  X() = X() + x_dot * dt;
//  Y() = Y() + y_dot * dt;
//  Psi() = Psi() + psi_dot * dt;
//  V() = V() + Acc() * dt;
//  vy_m = vy_dot * dt;
//}

#include <iostream>

void Vehicle::Drive2(const double dt) {
  double steer = Steer();
  double m = 1500;
  double lf = Lf_m;
  double lr = Lf_m;
  double cf = 1000;  // TireCoefFront
  double cr = 1000;
  double tr = 0.35;  // Tire radius

  double Izz = m * 1.8;  // 1/12.0 * m * (Length*Length+Width*Width);
  double vx = V();
  vx = std::max(vx, .1);

  double A, B, C, D, E, F;
  double cos_f = cos(steer);
  double psi = Psi();
  double coefcos_f = cf * cos_f;
  double yawr = vx / lf * steer;

  A = -(coefcos_f + cr) / (m * vx);
  B = (-lf * coefcos_f + lr * cr) / (m * vx) - vx;
  C = (-lf * coefcos_f + lr * cr) / (Izz * vx);
  D = -(lf * lf * cos_f + cr * lr * lr) / (Izz * vx);
  E = coefcos_f / m;
  F = lf * coefcos_f / Izz;

  //  vy = 0;

  double vy_dot = A * vy + C * tr + E * steer;
  double r_dot = B * vy + D * tr + F * steer;

  printf("str %.3f yawr %.4f vy_dot %.3f vy %.3f r_dot %.4f\n", steer, yawr,
         vy_dot, vy, r_dot);

  vy -= vy_dot * dt;
  yawr -= r_dot * dt;

  double x_dot = vx * cos(psi) - vy * sin(psi);
  double y_dot = vx * sin(psi) + vy * cos(psi);
  double theta_dot = yawr;

  X() += x_dot * dt;
  Y() += y_dot * dt;
  Psi() += theta_dot * dt;
}

// void Vehicle::Drive2(const double dt) {
//  double steer = Steer();
//  double m = 1600;
//  double lf = Lf_m;
//  double lr = Lf_m;
//  double cf = 1.5E5;  // TireCoefFront
//  double cr = 4E4;
//  double tr = 0.35;    // Tire radius
//  double Izz = m * 2;  // 1/12.0 * m * (Length*Length+Width*Width);
//  double psi = Psi();
//  double v = V();
//  double vx = v * cos(psi);
//  double vy = v * sin(psi);
//  double yawr = v / lf * steer * dt;
//  double r = 0.05;
//  double p[] = {m, lf, lr, cf, cr, r};
//  double dx[] = {0, 0, 0};
//  double x[] = {vx, vy, yawr};
//  double y[] = {0, 0, 0};
//  double u[] = {0.001, 0.001, 0, 0, steer};

//  ComputeDx(dx, x, u, p);
//  ComputeY(y, x, u, p);

//  vx += dx[1] * dt;
//  yawr += dx[2] * dt;

//  double xd = vx * cos(psi) - vy * sin(psi);
//  double yd = vx * sin(psi) + vy * cos(psi);

//  X() = X() + xd * dt;
//  Y() = Y() + yd * dt;
//  Psi() = Psi() + yawr * dt;
//  V() = V() + Acc() * dt;
//}

/* State equations. */
void Vehicle::ComputeDx(double *dx, double *x, double *u, double *p) {
  /* Retrieve model parameters. */
  double *m, *a, *b, *Cx, *Cy, *CA;
  m = &p[0];  /* Vehicle m.                    */
  a = &p[1];  /* Distance from front axle to COG. */
  b = &p[2];  /* Distance from rear axle to COG.  */
  Cx = &p[3]; /* Longitudinal tire stiffness.     */
  Cy = &p[4]; /* Lateral tire stiffness.          */
  CA = &p[5]; /* Air resistance coefficient.      */
  /* x[0]: Longitudinal vehicle velocity. */
  /* x[1]: Lateral vehicle velocity. */
  /* x[2]: Yaw rate. */
  dx[0] =
      x[1] * x[2] +
      1 / m[0] * (Cx[0] * (u[0] + u[1]) * cos(u[4]) -
                  2 * Cy[0] * (u[4] - (x[1] + a[0] * x[2]) / x[0]) * sin(u[4]) +
                  Cx[0] * (u[2] + u[3]) - CA[0] * pow(x[0], 2));
  dx[1] =
      -x[0] * x[2] +
      1 / m[0] * (Cx[0] * (u[0] + u[1]) * sin(u[4]) +
                  2 * Cy[0] * (u[4] - (x[1] + a[0] * x[2]) / x[0]) * cos(u[4]) +
                  2 * Cy[0] * (b[0] * x[2] - x[1]) / x[0]);
  dx[2] =
      1 / (pow(((a[0] + b[0]) / 2), 2) * m[0]) *
      (a[0] * (Cx[0] * (u[0] + u[1]) * sin(u[4]) +
               2 * Cy[0] * (u[4] - (x[1] + a[0] * x[2]) / x[0]) * cos(u[4])) -
       2 * b[0] * Cy[0] * (b[0] * x[2] - x[1]) / x[0]);
}

/* Output equations. */
void Vehicle::ComputeY(double *y, double *x, double *u, double *p) {
  /* Retrieve model parameters. */
  double *m = &p[0];  /* Vehicle m.                    */
  double *a = &p[1];  /* Distance from front axle to COG. */
  double *b = &p[2];  /* Distance from rear axle to COG.  */
  double *Cx = &p[3]; /* Longitudinal tire stiffness.     */
  double *Cy = &p[4]; /* Lateral tire stiffness.          */
  /* y[0]: Longitudinal vehicle velocity. */
  /* y[1]: Lateral vehicle acceleration. */
  /* y[2]: Yaw rate. */
  y[0] = x[0];
  y[1] =
      1 / m[0] * (Cx[0] * (u[0] + u[1]) * sin(u[4]) +
                  2 * Cy[0] * (u[4] - (x[1] + a[0] * x[2]) / x[0]) * cos(u[4]) +
                  2 * Cy[0] * (b[0] * x[2] - x[1]) / x[0]);
  y[2] = x[2];
}

double &Vehicle::X() { return state_m[0]; }

double &Vehicle::Y() { return state_m[1]; }

double &Vehicle::Psi() { return state_m[2]; }

double &Vehicle::V() { return state_m[3]; }

double &Vehicle::Steer() { return state_m[4]; }

double &Vehicle::Acc() { return state_m[5]; }

double &Vehicle::Cte() { return state_m[6]; }

double &Vehicle::Epsi() { return state_m[7]; }

const double &Vehicle::X() const { return state_m[0]; }

const double &Vehicle::Y() const { return state_m[1]; }

const double &Vehicle::Psi() const { return state_m[2]; }

const double &Vehicle::V() const { return state_m[3]; }

const double &Vehicle::Steer() const { return state_m[4]; }

const double &Vehicle::Acc() const { return state_m[5]; }

const double &Vehicle::Cte() const { return state_m[6]; }

const double &Vehicle::Epsi() const { return state_m[7]; }
