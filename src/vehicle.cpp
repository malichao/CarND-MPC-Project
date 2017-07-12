#include "vehicle.h"

Vehicle::Vehicle() : state_m(8) {}

void Vehicle::Drive(const double dt) {
  //    x = x + v * cos(psi) * dt;
  //    y = y + v * sin(psi) * dt;
  //    psi = psi + v / Lf * delta * dt;
  //    v = v + a * dt;
}

double &Vehicle::X() { return state_m[0]; }

double &Vehicle::Y() { return state_m[1]; }

double &Vehicle::Psi() { return state_m[2]; }

double &Vehicle::V() { return state_m[3]; }

double &Vehicle::Steer() { return state_m[4]; }

double &Vehicle::Throttle() { return state_m[5]; }

double &Vehicle::Cte() { return state_m[6]; }

double &Vehicle::Epsi() { return state_m[7]; }

const double &Vehicle::X() const { return state_m[0]; }

const double &Vehicle::Y() const { return state_m[1]; }

const double &Vehicle::Psi() const { return state_m[2]; }

const double &Vehicle::V() const { return state_m[3]; }

const double &Vehicle::Steer() const { return state_m[4]; }

const double &Vehicle::Throttle() const { return state_m[5]; }

const double &Vehicle::Cte() const { return state_m[6]; }

const double &Vehicle::Epsi() const { return state_m[7]; }
