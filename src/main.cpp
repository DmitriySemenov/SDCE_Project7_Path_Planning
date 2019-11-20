#define _CRT_SECURE_NO_WARNINGS
#ifdef _WIN32
#pragma comment(lib, "Ws2_32.lib")
#endif
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm> 
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "constants.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::min;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
	#ifdef _WIN32
		string map_file_ = "../../data/highway_map.csv";
  #else
		string map_file_ = "../data/highway_map.csv";
  #endif
	
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

	// target velocity (in m/s)
	double ref_vel = 49.5 / 2.237;

	// commanded velocity
	double cmd_vel = 0.2;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
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


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &cmd_vel, &max_s]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
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
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
					
					/*
					
					Step 0: Generate smooth waypoints near car

					Step 1: Figure out time offset (to predict other car positions) based on prev path size
									time_offset = prev_path_size * T_STEP
									Generate predicitons of new state for each car, including our car:
										car_state = end point of prev path converted from (x,y) to (s,d)
										other_car_state = use const velocity & constant lane and predict for time_offset seconds from current position

					Step 2: Based on car_position in (s,d) figure out available operating_state such as Keep Lane (KL), Lane Change Left (LCL), Lane Change Right (LCR)
									Available operating_state is only limited by car_lane, so can't do LCL in left most lane (0) and can't do LCR in right most lane (2).

					Step 3: For each available operating_state generate target position in (s,d) with T = 1 sec.
									For KL: d = middle of car_lane, 
											LCL: d = middle of the lane to the left of car_lane
											LCR: d = middle of the lane to the right of car_lane
									For all states: s = project s 1 second ahead using current speed.
																	Where current speed can be calculated using last 2 path points in (s,d), if prev_path_size > 1.
																	If prev_path_size <= 1, use our_car_speed
					Step 4: For each state target s and d (s_t, d_t), generate (RND_TGT_COUNT) number of randomized target states (s_r, d_r) using gaussian distribution.
									Mu = s_t or d_t; Sigma = Use constants from constants file.

					Step 5: For each (s_r, d_r) generate JMT coefficients for car_s -> s_r and car_d -> d_r, assuming TRAJ_TIME second(s) time for reaching target.

					Step 6: Calculate total cost for each set of JMT coeff. Use original (s_t, d_t) for each state.
								cost function:			weight:
								s_diff_cost					++
								d_diff_cost					++++
								t_diff_cost					++
								average_spd_cost		+++
								max_jerk_cost				+++++
								collision_cost			+++++
								buffer_cost					++
								max_accel_cost			+++++

					Step 7: Find minimum cost trajectory coefficients
					Step 8: Using minimum cost coefficiencts, generate s and d targets for the next TRAJ_TIME second(s) using T_STEP time step.
					Step 9: Add on new targets to the prev_path existing ones, until reaching PATH_SIZE number of target points. 
									Use conversion function to convert from (s,d) to (x,y) and smoothed our waypoints.

					*/

					// car's lane
					int car_lane = 1;

					if (car_d >= 0 && car_d < 4) {
						car_lane = 0;
					}
					else if (car_d >= 4 && car_d < 8) {
						car_lane = 1;
					}
					else {
						car_lane = 2;
					}

					// target lane
					int target_lane = car_lane;

					// best lane
					int best_lane = car_lane;
					double best_lane_speed = 0;
					bool avail_lanes[3] = { 0, 0, 0 };

					// speed and distance to in s of the closest car ahead and behind our car for each lane
					vector<double> closest_ahead_speed = { MAX_SPEED, MAX_SPEED, MAX_SPEED };
					vector<double> closest_ahead_dist = { max_s, max_s, max_s };
					vector<double> closest_behind_speed = { MAX_SPEED, MAX_SPEED, MAX_SPEED };
					vector<double> closest_behind_dist = { max_s, max_s, max_s };

					// reference position
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					int prev_size = previous_path_x.size();
					vector<double> next_x_vals;
					vector<double> next_y_vals;
					vector<double> rough_x_vals;
					vector<double> rough_y_vals;

					FindClosestCars(sensor_fusion, sensor_fusion.size(), car_s, max_s, closest_ahead_speed, closest_ahead_dist, closest_behind_speed, closest_behind_dist);

					// check if there is a car in front that is too close
					// maintain speed of the car ahead, once get too close
					bool too_close = false;

					if (closest_ahead_dist[car_lane] < slowdown_dist) {
						ref_vel = closest_ahead_speed[car_lane];
						too_close = true;
					}

					
					// find best lane
					// if current lane is clear or has top speed, keep current lane
					if (closest_ahead_speed[car_lane] >= MAX_SPEED || closest_ahead_dist[car_lane] >= clear_dist) {
						best_lane_speed = closest_ahead_speed[car_lane];
						best_lane = car_lane;
					}
					else {
						for (int lane = 0; lane < 3; ++lane) {
							// find the best lane, based on the closest cars ahead in each lane
							if (best_lane_speed < closest_ahead_speed[lane] || closest_ahead_dist[lane] >= clear_dist) {
								best_lane_speed = closest_ahead_speed[lane];
								best_lane = lane;
							}
							// check available lanes for lane changes, based on current lane and cars in adjacent lanes
							if (closest_ahead_dist[lane] > lanechange_dist&& closest_behind_dist[lane] > lanechange_dist) {
								avail_lanes[lane] = 1;
							}
						}
					}
					

					// adjust the target lane based on best lane and available lanes (keep same, if none are available)
					for (int lane = 0; lane < 3; ++lane) {
						// check that best lane is available
						if (lane == best_lane && avail_lanes[best_lane]) {
							// for the left lane, can only move to middle lane or stay
							if (car_lane == 0) {
								if (best_lane == 0 || best_lane == 1) {
									target_lane = best_lane;
								}
								// change target to lane next to current lane to get closer to target
								else {
									if (avail_lanes[1]) {
										target_lane = 1;
									}
								}
							}
							// for middle lane, can move to any lane, if they are available
							else if (car_lane == 1) {
								target_lane = best_lane;
							}
							// for the right lane, can only move to middle lane or stay
							else {
								if (best_lane == 2 || best_lane == 1) {
									target_lane = best_lane;
								}
								// change target to lane next to current lane to get closer to target
								else {
									if (avail_lanes[1]) {
										target_lane = 1;
									}
								}
							}
						}
					}

					std::cout << "Best: " << best_lane << " ";
					std::cout << "Target: " << target_lane << " ";
					std::cout << "L0 S: " << closest_ahead_speed[0] * 2.237 << " ";
					std::cout << "L1 S: " << closest_ahead_speed[1] * 2.237 << " ";
					std::cout << "L2 S: " << closest_ahead_speed[2] * 2.237 << " ";
					std::cout << "L0 D: " << closest_ahead_dist[0] << " ";
					std::cout << "L1 D: " << closest_ahead_dist[1] << " ";
					std::cout << "L2 D: " << closest_ahead_dist[2] << " \r";

					// if no cars are too close in front, speed up to target speed
					if (too_close == false)
						ref_vel = 49.5 / 2.237;

					// accelerate or slow down with 0.18 m/s / 0.02 s = 9 m/s^2 maximum accel or decel
					if (cmd_vel < ref_vel) {
						cmd_vel += min(0.18, (ref_vel - cmd_vel));
					}
					else {
						cmd_vel -= min(0.18, (cmd_vel - ref_vel));
					}


					// Create initial 2 rough start points using current car position or previous path last 2 points
					if (prev_size < 2) {
						double car_x_prev = car_x - cos(car_yaw);
						double car_y_prev = car_y - sin(car_yaw);

						rough_x_vals.push_back(car_x_prev);
						rough_x_vals.push_back(car_x);
						rough_y_vals.push_back(car_y_prev);
						rough_y_vals.push_back(car_y);
					}
					else {
						double ref_x_prev, ref_y_prev;

						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];
						ref_x_prev = previous_path_x[prev_size - 2];
						ref_y_prev = previous_path_y[prev_size - 2];
						ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

						rough_x_vals.push_back(ref_x_prev);
						rough_x_vals.push_back(ref_x);
						rough_y_vals.push_back(ref_y_prev);
						rough_y_vals.push_back(ref_y);
					}

					// Pick 3 points away from the car to give coordinates for the spline
					// !!!!!!!!! PROTECT FOR MAX_S !!!!!!!!!!!!!!!!!!
					vector<double> start_xy = getXY(car_s + 30, (2.0 + 4.0 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> mid_xy		= getXY(car_s + 60, (2.0 + 4.0 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> final_xy = getXY(car_s + 90, (2.0 + 4.0 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					
					rough_x_vals.push_back(start_xy[0]);
					rough_x_vals.push_back(mid_xy[0]);
					rough_x_vals.push_back(final_xy[0]);

					rough_y_vals.push_back(start_xy[1]);
					rough_y_vals.push_back(mid_xy[1]);
					rough_y_vals.push_back(final_xy[1]);

					// Shift car's reference angle to 0 degrees
					for (int i = 0; i < rough_x_vals.size(); ++i) {
						double shift_x = rough_x_vals[i] - ref_x;
						double shift_y = rough_y_vals[i] - ref_y;

						rough_x_vals[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
						rough_y_vals[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
					}
					
					tk::spline s;

					// Create a spline using rough x-y coordinates
					s.set_points(rough_x_vals, rough_y_vals);

					// Add old path points to the new path first
					for (int i = 0; i < prev_size; ++i) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					// How far away we want to generate the path for
					double target_x = 30;
					double target_y = s(target_x);
					double target_dist = sqrt(target_x * target_x + target_y * target_y);
					// Number of time-steps it takes to reach target distance with reference velocity
					double N = (target_dist) / (T_STEP * cmd_vel);
					// Step size in x-dimension per one time-step
					double x_step = target_x / N;
					double next_x = 0;
					double next_y = 0;
					double x_add_on = 0;

					// Fill in the rest of the path points, after we already added old points
					for (int i = 0; i < PATH_SIZE - prev_size; ++i) {
						next_x = x_add_on + x_step;
						next_y = s(next_x);
						x_add_on = next_x;

						double x_ref = next_x;
						double y_ref = next_y;

						//Rotate back
						next_x = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
						next_y = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

						next_x += ref_x;
						next_y += ref_y;

						next_x_vals.push_back(next_x);
						next_y_vals.push_back(next_y);
					}

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
	#ifdef _WIN32
		if (h.listen("127.0.0.1", port)) {
			std::cout << "Listening to port " << port << std::endl;
		}
		else {
			std::cerr << "Failed to listen to port" << std::endl;
			return -1;
		}
	#else
		if (h.listen(port)) {
			std::cout << "Listening to port " << port << std::endl;
		}
		else {
			std::cerr << "Failed to listen to port" << std::endl;
			return -1;
		}
	#endif
  
  h.run();
}