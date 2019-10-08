#include <cmath>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid_steering;
  PID pid_throttle;

  std::vector<double> p = {0.241871, 0.0095814, 19.7321};
//  std::vector<double> p = {0.241869, 0.0095816, 19.7320};

  int max_iterations = 1800;
  int iterations_to_ignore = 300;

  std::vector<double> dp = {0.00466, 0.00045, 0.092};
  double tol = 0.003;

//  std::vector<double> dp = {0.1, 0.005, 6};
  double best_err = 0.755628;

  std::vector<double> best_p = p;
  // pid_steering.Init(0.202, 0.0037602, 5.05);
  // pid_steering.Init(0.202, -0.0001602, 9.9505);
  // pid_steering.Init(0.202, -0.0001602, 5.05);
//  pid_steering.Init(0.2, 0.006, 15.0);
//  pid_steering.Init(0.2, 0.006, 10.0);
  pid_throttle.Init(0, 0, 10);

  bool run_optimizer = false;
  int num_iterations = 0;
  int epoch = 0;
  int factor_to_optimize = 0;
  double total_err = 0.0;
  int branch_num = 0;
  bool increment_factor = true;
  bool abort_early = false;
  float max_throttle_val = 0.75;
  float throttle_val = max_throttle_val / 2;
  float prev_err = 0;

  h.onMessage([
                  &pid_steering,
//                  &pid_throttle,
                  &total_err,
                  &num_iterations,
                  &max_iterations,
                  &run_optimizer,
                  &p,
                  &dp,
                  &tol,
                  &factor_to_optimize,
                  &epoch,
                  &best_p,
                  &best_err,
                  &branch_num,
                  &increment_factor,
                  &iterations_to_ignore,
                  &abort_early,
                  &max_throttle_val,
                  &throttle_val,
                  &prev_err
              ](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (!s.empty()) {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          double cte_steer = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());

          if (run_optimizer) {
            if (num_iterations == 0) {
              if (increment_factor) {
                p[factor_to_optimize] += dp[factor_to_optimize];
                branch_num = 0;
                total_err = 0;
                increment_factor = false;
                std::cout << "optimizing " << factor_to_optimize << std::endl;
              }

              std::cout << "trying {" << p[0] << ", " << p[1] << ", " << p[2] << "}" << std::endl;
              pid_steering.Init(p[0], p[1], p[2]);
            }

            if (
                (num_iterations > iterations_to_ignore && speed < max_throttle_val * 100 / 2) ||
                (num_iterations > 100 && abs(cte_steer) > 4.5)
                ) {
              abort_early = true;
            }
            // abort
            if (num_iterations == max_iterations || abort_early) {
              total_err = total_err / (float) (num_iterations - iterations_to_ignore);
              if (abort_early) {
                std::cout << "aborted early" << std::endl;
                total_err = 1000;
                abort_early = false;
              }
              // reset simulator
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

              std::cout << "error " << total_err << std::endl;

              if (total_err < best_err || best_err == 0) {
                best_err = total_err;
                dp[factor_to_optimize] *= 1.1;
                // move on to the next factor
                increment_factor = true;
                factor_to_optimize += 1;
                total_err = 0;
                best_p = p;
              } else {
                if (branch_num == 0) {
                  branch_num = 1;
                  p[factor_to_optimize] -= 2 * dp[factor_to_optimize];
                } else {
                  branch_num = 0;
                  p[factor_to_optimize] += dp[factor_to_optimize];
                  dp[factor_to_optimize] *= 0.9;

                  // move on to the next factor
                  increment_factor = true;
                  factor_to_optimize += 1;
                  total_err = 0;
                }
              }

              // reset iterations
              num_iterations = 0;

              // if we looped through all the factors then increase epoch
              if (factor_to_optimize > 2) {
                std::cout << "epoch " << epoch << std::endl;
                std::cout << "best error " << best_err << std::endl;
                std::cout << "best p {" << best_p[0] << ", " << best_p[1] << ", " << best_p[2] << "}" << std::endl;
                std::cout << "sum dp " << dp[0] + dp[1] + dp[2] << std::endl;

                factor_to_optimize = 0;
                increment_factor = true;
                epoch++;

                if (dp[0] + dp[1] + dp[2] < tol) {
                  run_optimizer = false;
                  reset_msg = "42[\"reset\",{}]";
                  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                }
              }
            } else {
              // ignore the initial wobble
              if (num_iterations >= iterations_to_ignore) {
                total_err += pow(pid_steering.TotalError(), 2);
              }
              num_iterations++;
            }
          } else if (!pid_steering.initialized) {
            pid_steering.Init(p[0], p[1], p[2]);
          }


          pid_steering.UpdateError(cte_steer);
          double steer_value = pid_steering.TotalError();

          // very feeble attempt to reduce throttle if the car is drifting too wide
          if (abs(cte_steer) > prev_err && prev_err > 0) {
            throttle_val -= 0.009;
            if (throttle_val < max_throttle_val / 2) {
              throttle_val = max_throttle_val / 2;
            }
          } else {
            throttle_val += 0.01;
            if (throttle_val > max_throttle_val) {
              throttle_val = max_throttle_val;
            }
          }

          prev_err = abs(cte_steer);

          // j[1] is the data JSON object
//          std::cout << j[1] << std::endl;
//          double angle = std::stod(j[1]["steering_angle"].get<string>());
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
//          std::cout << "angle " << angle << std::endl;
//          double cte_speed = speed - 100;
//          double steer_value = pid_steering.TotalError();

          if (steer_value > abs(1.0f)) {
            if (steer_value < 0) {
              steer_value = fmax(steer_value, -1);
            } else {
              steer_value = fmin(steer_value, 1);
            }
          }
          pid_steering.UpdateError(cte_steer);

//          double max_throttle_value = pid_throttle.TotalError();
//          pid_throttle.UpdateError(cte_speed);

          // DEBUG
//          std::cout << "CTE: " << cte_steer << " Steering Value: " << steer_value
//                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_val;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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