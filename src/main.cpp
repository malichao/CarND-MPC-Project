#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "mpc_config.h"
#include "path_smoother.h"
#include "utils.h"

// for convenience
using json = nlohmann::json;
using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double ToSimSteer(const double steer) { return rad2deg(-steer) / 25.0; }
double ToMPCSteer(const double steer) { return -steer; }

/* --- Sample telemetry format from simulator ---
[
  "telemetry",
  {
    "psi": 3.73521,
    "psi_unity": 4.118772,
    "ptsx": [
      -32.16173,
      -43.49173,
      -61.09,
      -78.29172,
      -93.05002,
      -107.7717
    ],
    "ptsy": [
      113.361,
      105.941,
      92.88499,
      78.73102,
      65.34102,
      50.57938
    ],
    "speed": 8.139975,
    "steering_angle": -0.001243055,
    "throttle": 1,
    "x": -42.01076,
    "y": 107.7916
  }
]
*/

double last_t = Now();
Vehicle veh;
int main(int argc, char** argv) {
  uWS::Hub h;

  // MPC is initialized here!
  std::string config_file = "../config/test.cfg";
  if (argc == 2) {
    config_file = argv[1];
  }
  MPCConfig mpc_config(config_file);
  //  mpc_config.WriteConfig("../config/test.cfg");
  MPC mpc(mpc_config);

  h.onMessage([&mpc, &mpc_config](uWS::WebSocket<uWS::SERVER> ws, char* data,
                                  size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message
    // event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        //        cout << setw(2) << j << endl;
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          //          double t1 = WrapHeading(j[1]["psi"]), t2 =
          //          j[1]["psi_unity"];
          //          printf("psi,psi_u [%.2f,%.2f]\n", t1, t2);
          WayPoints waypoints{j[1]["ptsx"], j[1]["ptsy"]};
          //          PathSmoother smoother;
          //          smoother.smooth(waypoints, waypoints);

          double psi = j[1]["psi"];
          psi = WrapHeading(psi);

          double v = mph2ms(j[1]["speed"]);
          double dt = Now() - last_t;
          double acc = (v - veh.V()) / dt;
          last_t = Now();

          veh.X() = j[1]["x"];
          veh.Y() = j[1]["y"];
          veh.V() = v;
          veh.Psi() = psi;
          veh.Acc() = acc;
          veh.Steer() = ToMPCSteer(j[1]["steering_angle"]);

          // Propagate the vehicle state by some time to compensate the latency
          veh.Drive(dt);

          ProcessData(mpc, waypoints, veh, mpc_config);
          veh.Steer() = mpc.Steer();
          veh.Acc() = mpc.Acc();
          double throttle = j[1]["throttle"];
          //          if (mpc.Acc() > 0.2)
          //            throttle += 0.1;
          //          else if (mpc.Acc() < -0.2)
          //            throttle -= 0.1;
          throttle = mpc.Acc();

          json msgJson;
          // Just want to see how it looks like in the second lapse.
          //          if (ms2mph(veh.V()) > 98) {
          //            disable_vis = true;
          //          }
          msgJson["steering_angle"] = ToSimSteer(veh.Steer());
          msgJson["throttle"] = throttle;
          //          printf("A%.2f T%.1f\n", mpc.Acc(), throttle);

          printf("Cost %.1f Vref %.1f Steer %.1f Freq %.1f\n", mpc.Cost(),
                 ms2mph(veh.V()), rad2deg(veh.Steer()), 1.0 / dt);

          // Display the MPC predicted trajectory
          msgJson["mpc_x"] = mpc.Prediction().x;
          msgJson["mpc_y"] = mpc.Prediction().y;

          // Display the waypoints/reference line
          msgJson["next_x"] = mpc.Reference().x;
          msgJson["next_y"] = mpc.Reference().y;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char* message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
