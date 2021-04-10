#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "behavior_stm.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

std::pair<double, double> Frenet2Cartesian(double s, double d, const tk::spline &s_x, const tk::spline &s_y)
{
  auto tr_x = s_x(s);
  auto tr_y = s_y(s);

  // Calculate direction vector at point s by using the spline derivative
  auto dx = s_x.deriv(1, s);
  auto dy = s_y.deriv(1, s);

  // normal to s curve, d coordinate unit vector
  auto der_mag = sqrt(dx * dx + dy * dy);
  auto n_d_x = dy / der_mag;
  auto n_d_y = -dx / der_mag;

  return std::pair<double, double>(tr_x + n_d_x * d, tr_y + n_d_y * d);
}

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Add starting point again to close the last interval
  map_waypoints_s.push_back(max_s);
  map_waypoints_x.push_back(map_waypoints_x[0]);
  map_waypoints_y.push_back(map_waypoints_y[0]);

  // Create spline of the road (center of the road, d = 0)
  tk::spline road_spline_x_s;
  tk::spline road_spline_y_s;

  road_spline_x_s.set_points(map_waypoints_s, map_waypoints_x);
  road_spline_y_s.set_points(map_waypoints_s, map_waypoints_y);

  // Since in this solution we approximate the car speed
  // simply as ds/dt, the approximation will be better for
  // smaller relative d. Hence, we put the waypoint line at the
  // center of the second track.
  double center_line_offset = 6;
  map_waypoints_x.clear();
  map_waypoints_y.clear();
  for (auto s_waypoint: map_waypoints_s)
  {
    auto new_pair = Frenet2Cartesian(s_waypoint, center_line_offset, road_spline_x_s, road_spline_y_s);
    map_waypoints_x.push_back(new_pair.first);
    map_waypoints_y.push_back(new_pair.second);
  }

  road_spline_x_s.set_points(map_waypoints_s, map_waypoints_x);
  road_spline_y_s.set_points(map_waypoints_s, map_waypoints_y);

  // Create state machine, which will make general decision on the movement of the car.
  constexpr int trajectory_length = 30;

  // behavior_stm_used: represents the realised trajectory of the car.
  // behavior_stm_used: represents predicted behavior some time in the future (defined by trajectory_length)
  // where the car does not make new decisions but only projects previous one into the future.
  BehaviorStm behavior_stm_used(max_s, center_line_offset);
  BehaviorStm behavior_stm_new = behavior_stm_used;
  bool initialized = false;

  h.onMessage([&road_spline_x_s, &road_spline_y_s, &behavior_stm_new, &behavior_stm_used,
                      &initialized, &max_s, &center_line_offset]
                      (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode)
              {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2')
                {
                  auto s = hasData(data);

                  if (s != "")
                  {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry")
                    {
                      // j[1] is the data JSON object

                      // Main car's localization Data
                      double car_x = j[1]["x"];
                      double car_y = j[1]["y"];
                      double car_s = j[1]["s"];
                      double car_d = j[1]["d"];
                      double car_yaw = j[1]["yaw"];
                      double car_speed = j[1]["speed"];

                      // Previous path data given to the Planner
                      auto previous_path_x = j[1]["previous_path_x"];
                      auto previous_path_y = j[1]["previous_path_y"];
                      // Previous path's end s and d values
                      double end_path_s = j[1]["end_path_s"];
                      double end_path_d = j[1]["end_path_d"];

                      // Sensor Fusion Data, a list of all other cars on the same side
                      // of the road. Represents [ id, x, y, vx, vy, s, d] f the car.
                      auto sensor_fusion = j[1]["sensor_fusion"];

                      json msgJson;

                      vector<double> next_x_vals;
                      vector<double> next_y_vals;

                      /**
                       * TODO: define a path made up of (x,y) points that the car will visit
                       *   sequentially every .02 seconds
                       */

                      int path_size = previous_path_x.size();

                      if (!initialized)
                      {
                        behavior_stm_used.SetModel(car_s, 0, car_d - center_line_offset,
                                                   std::floor(car_d - center_line_offset));
                        initialized = true;
                      }

                      // Iterate the model until the state space coordinates exactly
                      // match the first "unspent" (x,y) data from the previous set.
                      // This will be the point from which we can continue our prediction.
                      // This is necessary since it is not known in advance how many points the
                      // simulator will use from the previous iteration.
                      if (path_size > 0)
                      {
                        auto behavior_stm_temp = behavior_stm_used;
                        int counter = 0;

                        double p_x = previous_path_x[0];
                        double p_y = previous_path_y[0];

                        while (counter < trajectory_length)
                        {
                          auto x_y_predicted = Frenet2Cartesian(behavior_stm_temp.GetS(),
                                                                behavior_stm_temp.GetD(),
                                                                road_spline_x_s, road_spline_y_s);

                          double x_pred = x_y_predicted.first;
                          double y_pred = x_y_predicted.second;

                          // For some reason, the previously calculated values are not exactly the
                          // same when returned from the simulator. There is a small difference,
                          // so we consider that position is found if it's in  the vicinity of the
                          // previous calculation (0.01 -> 1 cm).
                          if (abs(x_pred - p_x) < 0.01 and abs(y_pred - p_y) < 0.01)
                          {
                            behavior_stm_used = behavior_stm_temp;
                            break;
                          }

                          behavior_stm_temp.Cycle(sensor_fusion);
                          ++counter;
                        }

                        // If the previous point is not found, we have undefined behavior of the model!
                        if (counter >= trajectory_length)
                        {
                          std::cout << "Undefined behavior, previous point not found!" << std::endl;
                        }
                      }

                      // Calculate one single cycle on actual data, and make decisions
                      behavior_stm_used.Cycle(sensor_fusion, false);

                      // Copy the realised model to the prediction model
                      behavior_stm_new = behavior_stm_used;

                      // Project our model certain time into future (for simulator use).
                      // During this time no actual sensor data is used!
                      for (size_t i = 0; i < trajectory_length; ++i)
                      {
                        behavior_stm_new.Cycle(sensor_fusion);
                        auto s_current = behavior_stm_new.GetS();
                        auto d_current = behavior_stm_new.GetD();
                        auto x_y = Frenet2Cartesian(s_current, d_current, road_spline_x_s, road_spline_y_s);

                        next_x_vals.push_back(x_y.first);
                        next_y_vals.push_back(x_y.second);
                      }

                      msgJson["next_x"] = next_x_vals;
                      msgJson["next_y"] = next_y_vals;
                      auto msg = "42[\"control\"," + msgJson.dump() + "]";

                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }  // end "telemetry" if
                  } else
                  {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
                }  // end websocket if
              }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                 {
                   std::cout << "Connected!!!" << std::endl;
                 });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
                    {
                      ws.close();
                      std::cout << "Disconnected" << std::endl;
                    });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  } else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}