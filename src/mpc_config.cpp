#include "mpc_config.h"
#include <fstream>
#include "json.hpp"
using json = nlohmann::json;

MPCConfig::MPCConfig() {
  x_start = 0;
  y_start = x_start + N;
  psi_start = y_start + N;
  v_start = psi_start + N;
  cte_start = v_start + N;
  epsi_start = cte_start + N;
  delta_start = epsi_start + N;
  a_start = delta_start + N - 1;
}

void MPCConfig::ReadConfig(const std::string file) {
  std::ifstream in(file);
  json j;
  in >> j;

  N = j["N"];
  dt = j["dt"];
  Lf = j["Lf"];        // Wheelbase
  ref_v = j["ref_v"];  // mph

  // ...
  x_start = 0;
  y_start = x_start + N;
  psi_start = y_start + N;
  v_start = psi_start + N;
  cte_start = v_start + N;
  epsi_start = cte_start + N;
  delta_start = epsi_start + N;
  a_start = delta_start + N - 1;

  // Weights for the cost function
  cte_w = j["cte_w"];
  epsi_w = j["epsi_w"];
  v_w = j["v_w"];
  delta_w = j["delta_w"];
  acc_w = j["acc_w"];
  delta_dot_w = j["delta_dot_w"];
  acc_dot_w = j["acc_dot_w"];

  default_max = j["default_max"];
  default_min = j["default_min"];
  delta_max = j["delta_max"];
  delta_min = j["delta_min"];
  acc_max = j["acc_max"];
  acc_min = j["acc_min"];
}

void MPCConfig::WriteConfig(const std::string file) {
  std::ofstream out(file);
  json j;

  j["N"] = N;
  j["dt"] = dt;
  j["Lf"] = Lf;        // Wheelbase
  j["ref_v"] = ref_v;  // mph

  // Weights for the cost function
  j["cte_w"] = cte_w;
  j["epsi_w"] = epsi_w;
  j["v_w"] = v_w;
  j["delta_w"] = delta_w;
  j["acc_w"] = acc_w;
  j["delta_dot_w"] = delta_dot_w;
  j["acc_dot_w"] = acc_dot_w;

  j["default_max"] = default_max;
  j["default_min"] = default_min;
  j["delta_max"] = delta_max;
  j["delta_min"] = delta_min;
  j["acc_max"] = acc_max;
  j["acc_min"] = acc_min;

  out << std::setw(2) << j << std::endl;
}
