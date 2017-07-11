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
  size_t test_size =waypoints.x.size();
  MPC mpc;
  Vehicle veh;
  Eigen::VectorXd ptsx(test_size);
  Eigen::VectorXd ptsy(test_size);
  waypoints.ToEigenVector(ptsx, ptsy);
  auto coeffs = polyfit(ptsx, ptsy, 3);
  double steer_offset = polyslope(coeffs,veh.x);
  veh.x = ptsx[0];
  veh.y = ptsy[0];
  veh.psi = polyslope(coeffs,veh.x);
//  veh.psi=atan2(waypoints.y[1]-waypoints.y[0],waypoints.x[1]-waypoints.x[0]);
  veh.v = 10;
  std::vector<double> x_vals, y_vals;
  WayPoints future_path;
  int test_iterations = 50;
  for (size_t i = 0; i < test_iterations; i++) {
    std::cout << "Iteration " << i << "\n";
    ProcessData(mpc, waypoints,veh);
    x_vals.push_back(veh.x);
    y_vals.push_back(veh.y);
    LocalToGlobal(veh.x, veh.y, veh.psi, mpc.Prediction().x, mpc.Prediction().y,
                  future_path.x, future_path.y);
    veh.steer = mpc.Steer();
    veh.throttle = mpc.Throttle();
    veh.x = future_path.x[1];
    veh.y = future_path.y[1];
    veh.psi +=mpc.Prediction().psi[1];
//    std::cout << "x = " << veh.x << "\n"
//              << "y = " << veh.y << "\n"
//              << "v = " << veh.v << "\n"
//              << "psi = " << veh.psi << "\n"
//              << "steer = " << veh.steer << "\n"
//              << "throttle = " << veh.throttle << "\n";
  }

  std::vector<double> orig_x(waypoints.x.begin(),
                             waypoints.x.begin() + test_size);
  std::vector<double> orig_y(waypoints.y.begin(),
                             waypoints.y.begin() + test_size);

  std::vector<double> poly_x;
  std::vector<double> poly_y;
  double step = (ptsx[ptsx.size() - 1] - ptsx[0]) / 20;
  for (double x = ptsx[0], i = 0; i < 20; i++) {
    poly_x.push_back(x);
    poly_y.push_back(polyeval(coeffs, x));
    x += step;
  }

//  plt::plot(orig_x, orig_y, "r--");
//  plt::plot(mpc.Reference().x, mpc.Reference().y, "r");
//  plt::plot(mpc.Prediction().x, mpc.Prediction().y, "b");
  plt::plot(poly_x, poly_y, "r");
  plt::plot(x_vals, y_vals, "b");
  plt::grid(true);
}

int main() {
  // ...
  WayPoints waypoints;
  TestDrawCenterPath(waypoints);
  size_t test_offset =0;
  size_t test_size = 6;
  WayPoints test_waypoints;
  test_waypoints.x=std::vector<double>(&waypoints.x[test_offset],
                                      &waypoints.x[test_offset + test_size]);
  test_waypoints.y=std::vector<double>(&waypoints.y[test_offset],
                                      &waypoints.y[test_offset + test_size]);
  TestMPC(test_waypoints);
  plt::show();
}
