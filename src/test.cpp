#include <fstream>
#include "MPC.h"
#include "matplotlibcpp.h"
#include "path_smoother.h"
#include "utils.h"
namespace plt = matplotlibcpp;

void TestDrawCenterPath(WayPoints& waypoints) {
  std::string file = "../lake_track_waypoints.csv";
  std::ifstream sin(file);
  std::string line;
  // Discard the first line
  std::getline(sin, line);
  int i = 0;
  WayPoints temp;
  while (std::getline(sin, line)) {
    //    std::cout << line << "\n";
    waypoints.x.push_back(std::stod(line.substr(0, line.find(',') - 1)));
    waypoints.y.push_back(std::stod(line.substr(line.find(',') + 1)));
    if (i++ % 10 == 0) {
      temp.x.push_back(waypoints.x.back());
      temp.y.push_back(waypoints.y.back());
    }
  }
  plt::plot(waypoints.x, waypoints.y, "r--");
  plt::plot(temp.x, temp.y, "g*");
}

void TestSmooth(WayPoints& waypoints) {
  PathSmoother smoother;
  smoother.smooth(waypoints, waypoints);
  plt::plot(waypoints.x, waypoints.y, "b--");
}

void TestMPC(WayPoints& waypoints) {
  size_t test_size = waypoints.x.size();
  MPCConfig mpc_config;
  MPC mpc(mpc_config);
  Vehicle veh;
  Eigen::VectorXd ptsx(test_size);
  Eigen::VectorXd ptsy(test_size);
  waypoints.ToEigenVector(ptsx, ptsy);
  veh.X() = ptsx[0];
  veh.Y() = ptsy[0];
  //  veh.Psi() = -atan(polyslope(coeffs,veh.X()));
  //  veh.Psi() =
  //  atan2(polyeval(coeffs,waypoints.x[1])-polyeval(coeffs,waypoints.x[0]),
  //          waypoints.x[1]-waypoints.x[0]);
  //  double x0 = waypoints.x[0];
  //  veh.Psi() = tan(coeffs[1]);
  veh.Psi() =
      atan2(waypoints.y[1] - waypoints.y[0], waypoints.x[1] - waypoints.x[0]);

  //  auto coeffs = polyfit(ptsx, ptsy, 3);
  //  printf("coef:[%.3f,%.3f,%.3f,%.3f]\n",coeffs[0],coeffs[1],coeffs[2],coeffs[3]);
  //  printf("heading: %.2f\n",atan(polyslope(coeffs,veh.X())));
  //  printf("heading:
  //  %.2f\n",atan2(waypoints.y[1]-waypoints.y[0],waypoints.x[1]-waypoints.x[0]));
  //  printf("heading:
  //  %.2f\n",atan2(polyeval(coeffs,waypoints.x[1])-polyeval(coeffs,waypoints.x[0]),
  //                                 waypoints.x[1]-waypoints.x[0]));
  veh.V() = 70;
  std::vector<double> x_vals, y_vals;
  WayPoints future_path;
  size_t test_iterations = 1;
  for (size_t i = 0; i < test_iterations; i++) {
    std::cout << "Iteration " << i << "\n";
    ProcessData(mpc, waypoints, veh, mpc_config);
    x_vals.push_back(veh.X());
    y_vals.push_back(veh.Y());
    LocalToGlobal(veh.X(), veh.Y(), veh.Psi(), mpc.Prediction().x,
                  mpc.Prediction().y, future_path.x, future_path.y);
    veh.Steer() = mpc.Steer();
    veh.Acc() = mpc.Acc();
    veh.X() = future_path.x[1];
    veh.Y() = future_path.y[1];
    veh.Psi() += mpc.Prediction().psi[1];
    std::cout << "x = " << veh.X() << "\n"
              << "y = " << veh.Y() << "\n"
              << "v = " << veh.V() << "\n"
              << "psi = " << veh.Psi() << "\n"
              << "steer = " << veh.Steer() << "\n"
              << "acc = " << veh.Acc() << "\n";
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
  TestSmooth(waypoints);
  //  size_t test_offset = 51;
  //  size_t test_size = 6;
  //  WayPoints test_waypoints;
  //  test_waypoints.x = std::vector<double>(&waypoints.x[test_offset],
  //                                         &waypoints.x[test_offset +
  //                                         test_size]);
  //  test_waypoints.y = std::vector<double>(&waypoints.y[test_offset],
  //                                         &waypoints.y[test_offset +
  //                                         test_size]);
  //  TestMPC(test_waypoints);
  plt::show();
}
