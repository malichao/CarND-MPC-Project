#include <fstream>
#include "MPC.h"
#include "matplotlibcpp.h"
#include "path_smoother.h"
#include "utils.h"
#include "vehicle_dynamics.h"
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
  //  plt::plot(waypoints.x, waypoints.y, "r--");
  //  plt::plot(temp.x, temp.y, "g*");
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
  //  plt::plot(mpc.Reference().x, mpc.Reference().y, "r--");
  //  plt::plot(mpc.Prediction().x, mpc.Prediction().y, "b");
  plt::named_plot("Reference", mpc.Reference().x, mpc.Reference().y, "r--");
  plt::named_plot("MPC Prediction", mpc.Prediction().x, mpc.Prediction().y,
                  "b");
  plt::legend();
  plt::grid(true);
}

void TestVehicle() {
  VehicleDynamics veh_dyn;
  Vehicle veh;
  double test_v = 20;
  double test_steer = deg2rad(0.45);
  veh.V() = test_v;
  veh.Steer() = test_steer;
  veh_dyn.vx = test_v;
  veh_dyn.steer = test_steer;
  std::vector<double> x, y;
  std::vector<double> x1, y1;
  for (int i = 0; i < 50; i++) {
    x.push_back(veh.X());
    y.push_back(veh.Y());
    x1.push_back(veh_dyn.x);
    y1.push_back(veh_dyn.y);
    veh.Drive(0.1);
    veh_dyn.drive(0.1);
  }
  printf("yawr %.3f | yawr %.3f vy %.5f \n", veh.V() / 2.67 * veh.Steer(),
         veh_dyn.yawr, veh_dyn.vy);
  plt::ylim(-200, 200);
  plt::plot(x, y, "r");
  plt::plot(x1, y1, "b");
  plt::grid(true);
  plt::show();
}

int main() {
  TestVehicle();

  //  WayPoints waypoints;
  //  TestDrawCenterPath(waypoints);
  //  //  TestSmooth(waypoints);
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
  //  plt::show();
}
