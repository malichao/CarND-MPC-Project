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
  Eigen::VectorXd ptsx(test_size);
  Eigen::VectorXd ptsy(test_size);
  waypoints.ToEigenVector(ptsx, ptsy, test_offset);
  auto coeffs = polyfit(ptsx, ptsy, 3);
  //  double x = ptsx[0];
  //  double y = ptsy[0];
  //  double psi = atan2(ptsy[1] - ptsy[0], ptsx[1] - ptsx[0]);
  //  double v = 10;
  //  double cte = polyeval(coeffs, x) - y;
  //  double epsi = psi - atan(polyslope(coeffs, x));
  double x_global = ptsx[0];
  double y_global = ptsy[0];
  double psi_offset = atan2(ptsy[1] - ptsy[0], ptsx[1] - ptsx[0]);
  double x = 0;
  double y = 0;
  double psi = 0;
  double v = 10;
  double cte = polyeval(coeffs, x) - y;
  double epsi = psi_offset - atan(polyslope(coeffs, x));

  //  std::cout << "Initial state\n"
  //            << "x = " << x << "\n"
  //            << "y = " << y << "\n"
  //            << "psi = " << psi << "\n"
  //            << "v = " << v << "\n"
  //            << "cte = " << cte << "\n"
  //            << "epsi = " << epsi << "\n";

  Eigen::VectorXd state(6);
  state << x, y, psi, v, cte, epsi;

  std::vector<double> x_vals = {x_global};
  std::vector<double> y_vals = {y_global};
  std::vector<double> psi_vals = {state[2]};
  std::vector<double> v_vals = {state[3]};
  std::vector<double> cte_vals = {state[4]};
  std::vector<double> epsi_vals = {state[5]};
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};

  int test_iterations = 20;
  WayPoints waypoints_local;
  Eigen::VectorXd ptsx_local(test_size);
  Eigen::VectorXd ptsy_local(test_size);
  for (size_t i = 0; i < test_iterations; i++) {
    std::cout << "Iteration " << i << std::endl;
    GlobalToLocal(x_global, y_global, psi_offset + state[2], waypoints.x,
                  waypoints.y, waypoints_local.x, waypoints_local.y);
    waypoints_local.ToEigenVector(ptsx_local, ptsy_local, test_offset);
    auto coeffs = polyfit(ptsx_local, ptsy_local, 3);
    auto vars = mpc.Solve(state, coeffs);

    LocalToGlobal(x_global, y_global, psi_offset + state[2], vars[0], vars[1],
                  x_global, y_global);
    x_vals.push_back(x_global);
    y_vals.push_back(y_global);
    psi_vals.push_back(vars[2]);
    v_vals.push_back(vars[3]);
    cte_vals.push_back(vars[4]);
    epsi_vals.push_back(vars[5]);

    delta_vals.push_back(vars[6]);
    a_vals.push_back(vars[7]);

    state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
    //    std::cout << "x = " << x_global << std::endl;
    //    std::cout << "y = " << y_global << std::endl;
    //    std::cout << "psi = " << vars[2] << std::endl;
    //    std::cout << "v = " << vars[3] << std::endl;
    //    std::cout << "cte = " << vars[4] << std::endl;
    //    std::cout << "epsi = " << vars[5] << std::endl;
    //    std::cout << "delta = " << vars[6] << std::endl;
    //    std::cout << "a = " << vars[7] << std::endl;
    //    std::cout << std::endl;
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
