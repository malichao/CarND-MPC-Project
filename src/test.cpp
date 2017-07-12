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
  size_t test_size = waypoints.x.size();
  MPC mpc;
  Vehicle veh;
  Eigen::VectorXd ptsx(test_size);
  Eigen::VectorXd ptsy(test_size);
  waypoints.ToEigenVector(ptsx, ptsy);
  veh.x = ptsx[0];
  veh.y = ptsy[0];
  //  veh.psi = -atan(polyslope(coeffs,veh.x));
  //  veh.psi =
  //  atan2(polyeval(coeffs,waypoints.x[1])-polyeval(coeffs,waypoints.x[0]),
  //          waypoints.x[1]-waypoints.x[0]);
  //  double x0 = waypoints.x[0];
  //  veh.psi = tan(coeffs[1]);
  veh.psi =
      atan2(waypoints.y[1] - waypoints.y[0], waypoints.x[1] - waypoints.x[0]);

  //  auto coeffs = polyfit(ptsx, ptsy, 3);
  //  printf("coef:[%.3f,%.3f,%.3f,%.3f]\n",coeffs[0],coeffs[1],coeffs[2],coeffs[3]);
  //  printf("heading: %.2f\n",atan(polyslope(coeffs,veh.x)));
  //  printf("heading:
  //  %.2f\n",atan2(waypoints.y[1]-waypoints.y[0],waypoints.x[1]-waypoints.x[0]));
  //  printf("heading:
  //  %.2f\n",atan2(polyeval(coeffs,waypoints.x[1])-polyeval(coeffs,waypoints.x[0]),
  //                                 waypoints.x[1]-waypoints.x[0]));
  veh.v = 40;
  std::vector<double> x_vals, y_vals;
  WayPoints future_path;
  int test_iterations = 1;
  for (size_t i = 0; i < test_iterations; i++) {
    std::cout << "Iteration " << i << "\n";
    ProcessData(mpc, waypoints, veh);
    x_vals.push_back(veh.x);
    y_vals.push_back(veh.y);
    LocalToGlobal(veh.x, veh.y, veh.psi, mpc.Prediction().x, mpc.Prediction().y,
                  future_path.x, future_path.y);
    veh.steer = mpc.Steer();
    veh.throttle = mpc.Throttle();
    veh.x = future_path.x[1];
    veh.y = future_path.y[1];
    veh.psi += mpc.Prediction().psi[1];
    std::cout << "x = " << veh.x << "\n"
              << "y = " << veh.y << "\n"
              << "v = " << veh.v << "\n"
              << "psi = " << veh.psi << "\n"
              << "steer = " << veh.steer << "\n"
              << "throttle = " << veh.throttle << "\n";
  }

  //  plt::plot(orig_x, orig_y, "r--");
  plt::plot(mpc.Reference().x, mpc.Reference().y, "r--");
  plt::plot(mpc.Prediction().x, mpc.Prediction().y, "b");
  plt::grid(true);
}

int main() {
  // ...
  WayPoints waypoints;
  TestDrawCenterPath(waypoints);
  size_t test_offset = 60;
  size_t test_size = 6;
  WayPoints test_waypoints;
  test_waypoints.x = std::vector<double>(&waypoints.x[test_offset],
                                         &waypoints.x[test_offset + test_size]);
  test_waypoints.y = std::vector<double>(&waypoints.y[test_offset],
                                         &waypoints.y[test_offset + test_size]);
  TestMPC(test_waypoints);
  plt::show();
}
