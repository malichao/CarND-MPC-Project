#include "vehicle.h"

Vehicle::Vehicle() : state_m(8), Lf_m(2.67) {}
Vehicle::Vehicle(const double lf) : state_m(8), Lf_m(lf) {}

void Vehicle::Drive(const double dt) {
  X() = X() + V() * cos(Psi()) * dt;
  Y() = Y() + V() * sin(Psi()) * dt;
  Psi() = Psi() + V() / Lf_m * tan(Steer()) * dt;
  V() = V() + Acc() * dt;
}

void Vehicle::Drive2(const double dt) {
  X() = X() + V() * cos(Psi()) * dt;
  Y() = Y() + V() * sin(Psi()) * dt;
  Psi() = Psi();
  V() = V() + Acc() * dt;
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
