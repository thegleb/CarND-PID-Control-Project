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

  std::vector<double> p = {0.241871, 0.0095814, 19.7321};
  std::vector<double> best_p = p;

  int max_iterations = 1800;
  int num_iterations_to_ignore = 300;

  std::vector<double> dp = {0.00466, 0.00045, 0.092};
  double max_sum_dp = 0.003;
  double best_err = 0.755628;

  // enable or disable the optimizer
  bool run_optimizer = false;

  // counters
  int num_iterations = 0;
  int epoch = 0;
  int factor_to_optimize = 0;

  // total error (sum of the error squared for each iteration of the twiddle, divided by number of iterations)
  double total_err = 0.0;

  // are we adding 1 unit or subtracting (twiddle algo specific)
  int branch_num = 0;

  // flag to determine when to move on to the next factor in the twiddle algo
  bool increment_factor = true;

  // flag to determine whether we should bail on the twiddle algo early
  bool abort_early = false;

  // max throttle; ~1.0 is about 100mph, so 0.75 is about 75mph top speed
  float max_throttle_val = 0.75;

  // set initial throttle value to half of max throttle (see "very weak attempt" section below)
  float throttle_val = max_throttle_val / 2;
  // also used by "very weak attempt" code
  double prev_err = 0;

  // wow this is a lot of captured variables
  h.onMessage([
                  &pid_steering,
                  &total_err,
                  &num_iterations,
                  &max_iterations,
                  &run_optimizer,
                  &p,
                  &dp,
                  &max_sum_dp,
                  &factor_to_optimize,
                  &epoch,
                  &best_p,
                  &best_err,
                  &branch_num,
                  &increment_factor,
                  &num_iterations_to_ignore,
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
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          std::cout << "angle " << angle << std::endl;

          // this twiddle looks like its full of demons but it actually seems to work ok
          // it is extra possessed because the entire logic here happens within a message loop
          // and within this loop we need to count out some # of iterations, perform some logic,
          // then hit a couple branches depending on the results of the logic, all while
          // the message loop continues ticking
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

            // if we have at least passed the initial few iterations but either:
            // - hit something that causes speed to drop
            // - go off-track (abs(cte) > 4.5)
            // then we should abort the cycle early
            if (
                (num_iterations > num_iterations_to_ignore && speed < max_throttle_val * 100 / 2) ||
                (num_iterations > 100 && abs(cte_steer) > 4.5)
                ) {
              abort_early = true;
            }

            // end state of the cycle - either we hit maximum number of iterations or we used the escape hatch above
            if (num_iterations == max_iterations || abort_early) {
              total_err = total_err / (float) (num_iterations - num_iterations_to_ignore);
              if (abort_early) {
                std::cout << "aborted early" << std::endl;
                // set error to something high because we probably don't want to use these factors anyway
                total_err = 1000;
                abort_early = false;
              }

              // reset simulator so we can start over
              // this places the car back at the starting line
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

              // output error value for this cycle
              std::cout << "error " << total_err << std::endl;

              // twiddle algorithm:
              // - for each multiplier (Tp, Ti, and Td), adjust it up by 1 unit or down by 1 unit
              // -- if this change improved error, then increase the unit for this factor by 10%
              // -- if it did NOT improve error, then decrease the unit for this factor by 10%
              // - repeat until the sum of the units is below some threshold (or some other arbitrary goal post)
              if (total_err < best_err || best_err == 0) {
                best_err = total_err;
                dp[factor_to_optimize] *= 1.1;
                // move on to the next factor
                increment_factor = true;
                factor_to_optimize += 1;
                total_err = 0;
                best_p = p;
              } else {
                // branch_num basically determines whether we added 1 unit or subtracted 1 unit in this iteration
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

              // if we looped through all the factors then increase "epoch"
              if (factor_to_optimize > 2) {
                std::cout << "epoch " << epoch << std::endl;
                std::cout << "best error " << best_err << std::endl;
                std::cout << "best p {" << best_p[0] << ", " << best_p[1] << ", " << best_p[2] << "}" << std::endl;
                std::cout << "sum dp " << dp[0] + dp[1] + dp[2] << std::endl;

                factor_to_optimize = 0;
                increment_factor = true;
                epoch++;

                if (dp[0] + dp[1] + dp[2] < max_sum_dp) {
                  run_optimizer = false;
                  reset_msg = "42[\"reset\",{}]";
                  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                }
              }
            } else {
              // ignore the initial wobble
              if (num_iterations >= num_iterations_to_ignore) {
                total_err += pow(pid_steering.TotalError(), 2);
              }
              num_iterations++;
            }
          } else if (!pid_steering.initialized) {
            pid_steering.Init(p[0], p[1], p[2]);
          }

          // main PID logic for steering
          pid_steering.UpdateError(cte_steer);
          double steer_value = pid_steering.TotalError();

          // very weak attempt to reduce throttle if the car is drifting too wide
          if (abs(cte_steer) > prev_err && prev_err > 0) {
            // if the error is increasing, then cut throttle
            throttle_val -= 0.009;

            // make sure we are always at least at half throttle
            if (throttle_val < max_throttle_val / 2) {
              throttle_val = max_throttle_val / 2;
            }
          } else {
            // if the error is decreasing, add throttle
            throttle_val += 0.01;

            // don't exceed max throttle (calm down, Vin Diesel)
            if (throttle_val > max_throttle_val) {
              throttle_val = max_throttle_val;
            }
          }
          prev_err = abs(cte_steer);

          // clamp the steer_value to a range between -1 and 1
          if (steer_value > abs(1.0f)) {
            if (steer_value < 0) {
              steer_value = fmax(steer_value, -1);
            } else {
              steer_value = fmin(steer_value, 1);
            }
          }
          pid_steering.UpdateError(cte_steer);

          // DEBUG
          std::cout << "CTE: " << cte_steer << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_val;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
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