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
          //ptx, ptsy contain waypoints of ideal path
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          //mph to m/s
          v *= 0.4407;

          /*
           * Transform car x,y to be the origin (0,0). Then rotate all
           * waypoints of "ideal" path given by simulator relative to this
           * new origin. This has the effect of making psi, the car's
           * orientation w.r.t lane center, zero. Simulator returns waypoints
           * in "map" coordinates.
           */
          for (size_t i = 0; i < ptsx.size(); i++) {
            double shiftx = ptsx[i] - px;
            double shifty = ptsy[i] - py;
            ptsx[i] = shiftx * cos(-psi) - shifty * sin(-psi);
            ptsy[i] = shiftx * sin(-psi) + shifty * cos(-psi);
          }

          /*
           * Fit a 3rd degree polynomial to the transformed waypoints.
           */
          Eigen::Map<Eigen::VectorXd> ptsxtransformed(&ptsx[0], ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsytransformed(&ptsy[0], ptsy.size());
          Eigen::VectorXd coeffs = polyfit(ptsxtransformed, ptsytransformed, 3);

          /*
           * Compute approximate CTE as the y-distance between
           * car's current position at 0,0 and the fitted curve. This can be
           * done by calling polyeval at x=0. Then compute Error in Orientation
           * (EPSI) as arctan of derivative of ideal path polynomial evaluated
           * at x=0 i.e., if poly is c0 + c1 * x + c2 * x^2 + c3 * x^3 then,
           * d/dx(poly) = c1 + 2 * c2 * x + 3 * c3 * x^2 which at x=0 due to
           * our earlier transformation means:
           * => epsi = psi - atan(c1) or epsi = -atan(c1), given that we also
           * transformed psi = 0.
           */
          double cte = polyeval(coeffs, 0.0);
          double epsi = -atan(coeffs[1]);

          /*
           * Build state and use MPC solve to get the new state vector
           * that optimizes for all of our criteria using the IPOPT solver.
           */
          Eigen::VectorXd state(ptsx.size());
          state << 0, 0, 0, v, cte, epsi;
          auto vars = mpc.Solve(state, coeffs);

          /*
           * Setup idealx, idealy values using data from the ideal path
           * polynomial fit - this line gets printed in yellow in the simulator.
           */
          vector<double> idealx, idealy;
          double ptspace = 2.0;
          double numpts = 10;
          for (int i = 0; i < numpts; i++) {
            idealx.push_back(i * ptspace);
            idealy.push_back(polyeval(coeffs, i * ptspace));
          }

          /*
           * Setup mpc_x_vals, mpc_y_vals values using vars from mpc solve
           * - this stuff gets printed in yellow in the simulator. Pay attention
           * to how vars is setup the x,y values alternate i.e. odd -> x,
           * even -> y. This line gets printed in green in the simulator.
           */
          vector<double> mpcx, mpcy;
          for (size_t i = 2; i < vars.size(); i++) {
            if (i % 2 == 0)
              mpcx.push_back(vars[i]);
            else
              mpcy.push_back(vars[i]);
          }

          /*
           * Finally, send the throttle, steering, new_*, mpc_* over to the
           * simulator.
           */

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          double steer_value = -vars[0] / deg2rad(25);
          double throttle_value = vars[1];
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpcx;
          msgJson["mpc_y"] = mpcy;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = idealx;
          msgJson["next_y"] = idealy;

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
