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
#include "utils.h"

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

void ProcessData(MPC& mpc,const WayPoints& waypoints, WayPoints& future_path,
                 Vehicle& veh) {
    WayPoints waypoints_local;
    Eigen::VectorXd ptsx_local(waypoints.x.size());
    Eigen::VectorXd ptsy_local(waypoints.x.size());
    Eigen::VectorXd state(6);
    GlobalToLocal(veh.x, veh.y, veh.psi, waypoints.x,
                  waypoints.y, waypoints_local.x, waypoints_local.y);
    waypoints_local.ToEigenVector(ptsx_local, ptsy_local);
    auto coeffs = polyfit(ptsx_local, ptsy_local, 3);
    double cte = polyeval(coeffs, 0);
    double epsi = 0 - atan(polyslope(coeffs, 0));
    state<<0,0,0,veh.v,cte,epsi;
    mpc.Solve(state, coeffs);
    LocalToGlobal(veh.x, veh.y, veh.psi, mpc.Predictions().x, mpc.Predictions().y,
                  future_path.x, future_path.y);
    veh.steer=mpc.Steer();
    veh.throttle=mpc.Throttle();
    std::cout<<"Steer, throttle = "<<veh.steer<<","<<veh.throttle<<"\n";
}

double ToSimSteer(const double steer){
    return steer/(25.0/180.0*M_PI);
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
//    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          //          vector<double> ptsx = j[1]["ptsx"];
          //          vector<double> ptsy = j[1]["ptsy"];
          //          double px = j[1]["x"];
          //          double py = j[1]["y"];
          //          double psi = j[1]["psi"];
          //          double v = j[1]["speed"];

          WayPoints waypoints{j[1]["ptsx"],j[1]["ptsy"]};
          WayPoints future_path;
//          waypoints.x = ;
//          waypoints.y = j[1]["ptsxy"];
          Vehicle veh;
          veh.x = j[1]["x"];
          veh.y = j[1]["y"];
          veh.psi = j[1]["psi"];
          veh.v = j[1]["speed"];
          ProcessData(mpc,waypoints,future_path,veh);
           for(int i=0;i<future_path.x.size();i++){
               printf("(%.1f,%.1f) ",future_path.x[i],future_path.y[i]);
           }
           std::cout<<"\n";
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          // steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25]
          // instead of [-1, 1].

//          std::cout<<"Steer, throttle sim = "<<ToSimSteer(veh.steer)<<","
//                    <<veh.throttle<<"\n";
          msgJson["steering_angle"] = 0;//ToSimSteer(veh.steer);
          msgJson["throttle"] = 0;//veh.throttle;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = future_path.x;
          msgJson["mpc_y"] = future_path.y;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = waypoints.x;
          msgJson["next_y"] = waypoints.y;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
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
