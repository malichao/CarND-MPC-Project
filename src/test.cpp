#include <fstream>
#include "MPC.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void TestDrawCenterPath() {
  std::vector<double> in_x;
  std::vector<double> in_y;
  std::string file = "../lake_track_waypoints.csv";
  std::ifstream sin(file);
  std::string line;
  // Discard the first line
  std::getline(sin, line);
  while (std::getline(sin, line)) {
    //    std::cout << line << "\n";
    std::string x, y;
    in_x.push_back(std::stod(line.substr(0, line.find(',') - 1)));
    in_y.push_back(std::stod(line.substr(line.find(',') + 1)));
  }
  plt::plot(in_x, in_y, "r");
  plt::show();
}

int main() {
  // ...
  TestDrawCenterPath();
}
