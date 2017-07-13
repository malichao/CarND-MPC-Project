#ifndef PATHS_MOOTHER_H
#define PATHS_MOOTHER_H
#include "waypoints.h"

class PathSmoother {
 public:
  PathSmoother();
  void smooth(const WayPoints& in, WayPoints& out);

  void SetTolerance(double value);

  void SetDataWeight(double value);

  void SetSmoothWeight(double value);

  void SetUpSample(int value);

  void SetMaxIteration(int value);

 private:
  double tolerance_m = 0.1;
  double weight_data_m = 0.45;
  double weight_smooth_m = 0.455;
  int up_sample_m = 2;
  int max_iteration_m = 100;
};

#endif  // PATHS_MOOTHER_H
