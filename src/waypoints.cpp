#include "waypoints.h"

void WayPoints::ToEigenVector(Eigen::VectorXd &vec_x, Eigen::VectorXd &vec_y,
                              int start_pos) {
  assert(vec_x.size() == vec_y.size());
  for (int i = 0; i < vec_x.size() && i + start_pos < x.size(); i++) {
    vec_x[i] = x[i + start_pos];
    vec_y[i] = y[i + start_pos];
  }
}
