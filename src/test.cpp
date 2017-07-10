#include <fstream>
#include "MPC.h"
#include "matplotlibcpp.h"
#include "utils.h"
namespace plt = matplotlibcpp;

void TestDrawCenterPath(WayPoints& waypoints) {
  std::string file = "../lake_track_waypoints.csv";
  std::ifstream sin(file);
  std::string line;
  // Discard the first line
  std::getline(sin, line);
  while (std::getline(sin, line)) {
    //    std::cout << line << "\n";
    waypoints.x.push_back(std::stod(line.substr(0, line.find(',') - 1)));
    waypoints.y.push_back(std::stod(line.substr(line.find(',') + 1)));
  }
  plt::plot(waypoints.x, waypoints.y, "r");
  plt::show();
}

void TestMPC(WayPoints& waypoints) {}

int main() {
  // ...
  WayPoints waypoints;
  MPC mpc;
  Eigen::VectorXd x(6);
  Eigen::VectorXd y(6);
  waypoints.ToEigenVector(x, y);
  TestDrawCenterPath(waypoints);
}
