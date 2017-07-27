#include "mpc_config.h"
#include <fstream>
#include "json.hpp"
#include "utils.h"
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

MPCConfig::MPCConfig(const std::string file) { ReadConfig(file); }

// MMa 2017-07-22 We should convert unit as soon as possible
void MPCConfig::ReadConfig(const std::string file) {
  try {
    std::cout << " Loading configurations from [" << file << "]\n";
    std::ifstream in(file);
    json j;
    in >> j;

    N = j["N"];
    dt = j["dt"];
    Lf = j["Lf"];                // Wheelbase
    ref_v = mph2ms(j["ref_v"]);  // mph

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
    cte_w = j["w_cte"];
    epsi_w = j["w_epsi"];
    v_w = j["w_v"];
    delta_w = j["w_delta"];
    acc_w = j["w_acc"];
    delta_dot_w = j["w_delta_dot"];
    acc_dot_w = j["w_acc_dot"];

    default_max = j["r_default_max"];
    default_min = j["r_default_min"];
    delta_max = j["r_delta_max"];
    delta_min = j["r_delta_min"];
    acc_max = j["r_acc_max"];
    acc_min = j["r_acc_min"];
    vx_max = mph2ms(j["r_vx_max"]);
    vx_min = mph2ms(j["r_vx_min"]);

    adaptive_speed = j["adaptive_speed"];

    std::cout << "------ Configuration ------\n";
    std::cout << std::setw(2) << j << "\n";
  } catch (...) {
    std::cout << "Warning! Could not load configurations.\n"
              << "Using default settings.\n"
              << "------ Configuration ------\n";
    std::cout << "cte_w          " << cte_w << "\n"
              << "epsi_w         " << epsi_w << "\n"
              << "v_w            " << v_w << "\n"
              << "delta_w        " << delta_w << "\n"
              << "acc_w          " << acc_w << "\n"
              << "delta_dot_w    " << delta_dot_w << "\n"
              << "acc_dot_w      " << delta_dot_w << "\n"

              << "default_max    " << default_max << "\n"
              << "default_min    " << default_min << "\n"
              << "delta_max      " << delta_max << "\n"
              << "delta_min      " << delta_min << "\n"
              << "acc_max        " << acc_max << "\n"
              << "acc_min        " << acc_min << "\n"
              << "vx_max         " << vx_max << "\n"  // mph
              << "vx_min         " << vx_min << "\n"  // mph
              << "adaptive_speed " << adaptive_speed << "\n"
              << "ref_v          " << ref_v << "\n";  // mph
    ref_v = mph2ms(ref_v);
    vx_max = mph2ms(vx_max);
    vx_min = mph2ms(vx_min);
  }
}

void MPCConfig::WriteConfig(const std::string file) {
  std::ofstream out(file);
  json j;

  j["N"] = N;
  j["dt"] = dt;
  j["Lf"] = Lf;                // Wheelbase
  j["ref_v"] = ms2mph(ref_v);  // mph

  // Weights for the cost function
  j["w_cte"] = cte_w;
  j["w_epsi"] = epsi_w;
  j["w_v"] = v_w;
  j["w_delta"] = delta_w;
  j["w_acc"] = acc_w;
  j["w_delta_dot"] = delta_dot_w;
  j["w_acc_dot"] = acc_dot_w;

  j["r_default_max"] = default_max;
  j["r_default_min"] = default_min;
  j["r_delta_max"] = delta_max;
  j["r_delta_min"] = delta_min;
  j["r_acc_max"] = acc_max;
  j["r_acc_min"] = acc_min;
  j["r_vx_max"] = ms2mph(vx_max);
  j["r_vx_min"] = ms2mph(vx_min);

  j["adaptive_speed"] = adaptive_speed;

  out << std::setw(2) << j << std::endl;
}
