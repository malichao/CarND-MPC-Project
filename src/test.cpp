#include <fstream>
#include "MPC.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

struct WayPoints {
  std::vector<double> x;
  std::vector<double> y;
};

void TestDrawCenterPath(WayPoints& way_points) {
  std::string file = "../lake_track_waypoints.csv";
  std::ifstream sin(file);
  std::string line;
  // Discard the first line
  std::getline(sin, line);
  while (std::getline(sin, line)) {
    //    std::cout << line << "\n";
    way_points.x.push_back(std::stod(line.substr(0, line.find(',') - 1)));
    way_points.y.push_back(std::stod(line.substr(line.find(',') + 1)));
  }
  plt::plot(way_points.x, way_points.y, "r");
  plt::show();
}

int main() {
  // ...
  WayPoints way_point;
  TestDrawCenterPath(way_point);
}
