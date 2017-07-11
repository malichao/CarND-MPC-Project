#ifndef WAYPOINTS_H
#define WAYPOINTS_H
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

class WayPoints {
 public:
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> psi;

  void ToEigenVector(Eigen::VectorXd& vec_x, Eigen::VectorXd& vec_y,
                     int start_pos = 0);
};

#endif  // WAYPOINTS_H
