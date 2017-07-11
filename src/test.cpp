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
  //  plt::plot(waypoints.x, waypoints.y, "r--");
  //  plt::show();
}

void TestMPC(WayPoints& waypoints) {
  int test_size = 8;
  int test_offset = 0;
  MPC mpc;
  Vehicle veh;
  Eigen::VectorXd ptsx(test_size);
  Eigen::VectorXd ptsy(test_size);
  waypoints.ToEigenVector(ptsx, ptsy, test_offset);
  auto coeffs = polyfit(ptsx, ptsy, 3);
  veh.x = ptsx[0];
  veh.y = ptsy[0];
  veh.psi = 0;
  veh.v = 10;
  std::vector<double> x_vals, y_vals;
  WayPoints future_path;
  int test_iterations = 20;
  for (size_t i = 0; i < test_iterations; i++) {
    std::cout << "Iteration " << i << "\n";
    ProcessData(mpc, waypoints, future_path, veh);
    x_vals.push_back(veh.x);
    y_vals.push_back(veh.y);
    veh.x = mpc.Predictions().x[1];
    veh.y = mpc.Predictions().y[1];
    //    veh.psi = mpc.Predictions().psi[1];
    std::cout << "x = " << veh.x << "\n"
              << "y = " << veh.y << "\n"
              << "psi = " << veh.psi << "\n"
              << "steer = " << veh.steer << "\n"
              << "throttle = " << veh.throttle << "\n";
  }

  // Plot values
  // NOTE: feel free to play around with this.
  // It's useful for debugging!
  //  plt::subplot(3, 1, 1);
  //  plt::title("CTE");
  //  plt::plot(cte_vals);
  //  plt::grid(true);
  //  plt::subplot(3, 1, 2);
  //  plt::title("Delta (Radians)");
  //  plt::plot(delta_vals);
  //  plt::grid(true);
  //  plt::subplot(3, 1, 3);
  //  plt::title("Velocity");
  //  plt::plot(v_vals);
  //  plt::grid(true);
  //  plt::show();

  std::vector<double> orig_x(waypoints.x.begin() + test_offset,
                             waypoints.x.begin() + test_offset + test_size);
  std::vector<double> orig_y(waypoints.y.begin() + test_offset,
                             waypoints.y.begin() + test_offset + test_size);

  std::vector<double> poly_x;
  std::vector<double> poly_y;
  //  for (int i = 0; i < ptsx.size(); i++) {
  //    poly_x.push_back(ptsx[i]);
  //    poly_y.push_back(polyeval(coeffs, ptsx[i]));
  //  }
  double step = (ptsx[ptsx.size() - 1] - ptsx[0]) / 20;
  for (double x = ptsx[0], i = 0; i < 20; i++) {
    poly_x.push_back(x);
    poly_y.push_back(polyeval(coeffs, x));
    x += step;
  }

  plt::plot(orig_x, orig_y, "r--");
  plt::plot(poly_x, poly_y, "r");
  plt::plot(x_vals, y_vals, "b");
  plt::grid(true);
}

int main() {
  // ...
  WayPoints waypoints;
  TestDrawCenterPath(waypoints);
  TestMPC(waypoints);
  plt::show();
}
