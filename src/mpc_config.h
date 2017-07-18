#ifndef MPC_CONFIG_H
#define MPC_CONFIG_H
#include <math.h>
#include <string>
class MPCConfig {
 public:
  MPCConfig();
  MPCConfig(const std::string file);

  void ReadConfig(const std::string file);
  void WriteConfig(const std::string file);

  size_t N = 40;
  double dt = 0.05;
  double Lf = 2.67;   // Wheelbase
  double ref_v = 40;  // mph

  // ...
  size_t x_start = 0;
  size_t y_start = x_start + N;
  size_t psi_start = y_start + N;
  size_t v_start = psi_start + N;
  size_t cte_start = v_start + N;
  size_t epsi_start = cte_start + N;
  size_t delta_start = epsi_start + N;
  size_t a_start = delta_start + N - 1;

  // Weights for the cost function
  double cte_w = 2.50;
  double epsi_w = 300.0;
  double v_w = 1.0;
  double delta_w = 5000.0;
  double acc_w = 1.0;
  double delta_dot_w = 15000.0;
  double acc_dot_w = 10.0;

  double default_max = 1.0e19;
  double default_min = -1.0e19;
  double delta_max = 25.0 / 180.0 * M_PI;
  double delta_min = -25.0 / 180.0 * M_PI;
  double acc_max = 3.0;
  double acc_min = -0.50;
  double vx_max = 100;
  double vx_min = 65;
};

#endif  // MPC_CONFIG_H
