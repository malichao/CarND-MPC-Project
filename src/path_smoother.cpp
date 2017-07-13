#include "path_smoother.h"

PathSmoother::PathSmoother() {}

void PathSmoother::smooth(const WayPoints& in, WayPoints& out) {
  assert(in.x.size() == in.y.size());
  if (in.x.size() <= 2) {
    out = in;
    return;
  }

  WayPoints in_;
  WayPoints out_;

  for (auto p : in.x) {
    for (int i = 0; i < up_sample_m; i++) {
      in_.x.push_back(p);
    }
  }
  for (auto p : in.y) {
    for (int i = 0; i < up_sample_m; i++) {
      in_.y.push_back(p);
    }
  }

  out_ = in_;

  double change = tolerance_m;
  int iteration = 0;
  while (change >= tolerance_m) {
    change = 0;
    for (size_t i = 1; i < out_.x.size() - 1; i++) {
      double delta_x, delta_y;

      delta_x = (in_.x[i] - out_.x[i]) * weight_data_m;
      delta_y = (in_.y[i] - out_.y[i]) * weight_data_m;

      double new_x, new_y;
      new_x = out_.x[i] + delta_x;
      new_y = out_.y[i] + delta_y;
      out_.x[i] = new_x;
      out_.y[i] = new_y;

      change += delta_x * delta_x + delta_y * delta_y;

      delta_x =
          (out_.x[i - 1] + out_.x[i + 1] - out_.x[i] * 2) * weight_smooth_m;

      delta_y =
          (out_.y[i - 1] + out_.y[i + 1] - out_.y[i] * 2) * weight_smooth_m;

      new_x = out_.x[i] + delta_x;
      new_y = out_.y[i] + delta_y;
      out_.x[i] = new_x;
      out_.y[i] = new_y;
      change += delta_x * delta_x + delta_y * delta_y;
    }
    if (iteration++ > max_iteration_m) break;
  }

  out = out_;
}

void PathSmoother::SetTolerance(double value) {
  tolerance_m = value;
  tolerance_m = tolerance_m < 0 ? 0 : tolerance_m;
}

void PathSmoother::SetDataWeight(double value) {
  weight_data_m = value;
  weight_data_m = weight_data_m < 0 ? 0 : weight_data_m;
  weight_data_m = weight_data_m >= 5.0 ? 4.99999999 : weight_data_m;
}

void PathSmoother::SetSmoothWeight(double value) {
  weight_smooth_m = value;
  weight_smooth_m = weight_smooth_m < 0 ? 0 : weight_smooth_m;
  weight_smooth_m = weight_smooth_m >= 5.0 ? 4.99999999 : weight_smooth_m;
}

void PathSmoother::SetUpSample(int value) {
  up_sample_m = value;
  up_sample_m = up_sample_m < 1 ? 1 : up_sample_m;
}

void PathSmoother::SetMaxIteration(int value) {
  max_iteration_m = value;
  max_iteration_m = max_iteration_m < 1 ? 1 : max_iteration_m;
}
