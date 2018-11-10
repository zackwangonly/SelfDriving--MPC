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

// for convenience
using json = nlohmann::json;

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // The states of the vehicle
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          // The input of the vehicle
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // To Calculate steering angle and throttle using MPC, we need the optimization function return
          //an input due to the state of the vehicle now and the whole trajectory of the road
          //First we need to transfer the global coordinate to the vehicle coordinate
          //ref: https://en.wikipedia.org/wiki/Rotation_of_axes
          VectorXd ptsx_car(ptsx.size());
          VectorXd ptsy_car(ptsy.size());
          for(unsigned int i = 0; i < ptsx.size(); i++){
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            ptsx_car[i] = x * cos(psi) + y * sin(psi);
            ptsy_car[i] = - x * sin(psi) + y * cos(psi);
          }
          // Now the pysx is the the x position of the way points under the coordinate of the vehicle
          //Now the ptsy is the y position of the way points under the coordinate of the vehicle

          // Get the poly based on vehicle coordinate , this is a poly gor road trajectory
          // Here we choose 3 order trajectory, because it can satisfy most roads
          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

          // The next step is to get the state of the vehicle

          // After the transition of the coordinate, because the origin of the new coordinate is
          // position of the vehicle, we should adjust the position of the state
          // The new state
          double state_x = 0.0;
          double state_y = 0.0;
          double state_psi = 0.0;
          double state_v = v;
          double state_cte = polyeval(coeffs, state_x) - state_y;
          double state_epsi = state_psi - atan(coeffs[1]);

          // Define a state of the vehicle
          VectorXd state(6);
          //length of the front of the vehicle to the G center of the vehicle
          const double Lf = 2.67;
          // Define a latency time
          const double time_latency = 0.1;

          //Here we consider the latency, update the state, try to get the state after the latency
          double latency_v = state_v + a * time_latency;
          double latency_psi = state_psi + 0.5*(state_v+latency_v) / Lf * (-delta) * time_latency;
          double latency_x = state_x + 0.5*(state_v+latency_v)*cos(0.5*(state_psi+latency_psi))*time_latency;
          double latency_y = state_y + 0.5*(state_v+latency_v)*sin(0.5*(state_psi+latency_psi))*time_latency;
          double latency_cte = polyeval(coeffs,latency_x)-latency_y;
          double latency_epsi = latency_psi - atan(coeffs[1]);

          state << latency_x, latency_y, latency_psi, latency_v, latency_cte, latency_epsi;

          // Now we have the state and the poly of the road
          // We can get the input using optimization
          // in the vars, the first and second value are the control signal
          auto vars = mpc.Solve(state, coeffs);
          double steer_value = vars[0];
          double throttle_value = vars[1];



          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/(deg2rad(25) * Lf);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          // From the vars[2], the value are the predict x,y, so we want to display them
          for (unsigned int i = 2; i < vars.size(); i += 2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          // Here we display next 100 points, sample 25 points
          for (double i = 0.0; i < 100.0; i += 4.0){
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
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
                         char *message, size_t length) {
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
