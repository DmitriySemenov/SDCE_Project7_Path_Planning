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
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

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

	// Time Step
	double t_step = 0.02;

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
               &map_waypoints_dx,&map_waypoints_dy, &t_step, &ref_vel, &cmd_vel]
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
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
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

					// speed and distance to in s of the closest car ahead and behind our car for each lane
					vector<double> closest_ahead_speed = { 0.0, 0.0, 0.0 };
					vector<double> closest_ahead_dist = { 0.0, 0.0, 0.0 };
					vector<double> closest_behind_speed = { 0.0, 0.0, 0.0 };
					vector<double> closest_behind_dist = { 0.0, 0.0, 0.0 };

					// reference position
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					int prev_size = previous_path_x.size();
					vector<double> next_x_vals;
					vector<double> next_y_vals;
					vector<double> rough_x_vals;
					vector<double> rough_y_vals;

					// check if there is a car in front that is too close
					bool too_close = false;

					// find the closest car ahead and behind our car
					for (int i = 0; i < sensor_fusion.size(); ++i) {
						
						float d = sensor_fusion[i][6];
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double check_car_speed = sqrt(vx * vx + vy * vy);
						double check_car_s = sensor_fusion[i][5];
						
						// Account for wrapping around max_s
						check_car_s += (double)prev_size * t_step * check_car_speed;
						// Calculate distance, accounting for wrapping around max_s

						for (int lane = 0; lane < 3; ++lane) {
							// for every lane, find the car closest ahead and behind of us
							if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2)) {

								if (check_car_s >= car_s) {

									// maintain speed of the car ahead, once get too close
									if ((check_car_s - car_s) < 40 && lane == car_lane) {
										ref_vel = check_car_speed;
										too_close = true;
									}
								}	

							}
						}

					}

					// find the best lane, based on the closest cars ahead in each lane

					// check available lanes for lane changes, based on current lane and cars in adjacent lanes

					// adjust the target lane based on best lane and available lanes (keep same, if none are available)
					


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

					std::cout << "Target Speed: " << ref_vel * 2.237 << " MPH.";
					std::cout << "Commanded Speed: " << cmd_vel * 2.237 << " MPH. \r";

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
					vector<double> start_xy = getXY(car_s + 30, (2 + 4 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> mid_xy		= getXY(car_s + 60, (2 + 4 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> final_xy = getXY(car_s + 90, (2 + 4 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					
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

					// How many points we want to have in the path
					double path_size = 50;
					// How far away we want to generate the path for
					double target_x = 30;
					double target_y = s(target_x);
					double target_dist = sqrt(target_x * target_x + target_y * target_y);
					// Number of time-steps it takes to reach target distance with reference velocity
					double N = (target_dist) / (t_step * cmd_vel);
					// Step size in x-dimension per one time-step
					double x_step = target_x / N;
					double next_x = 0;
					double next_y = 0;
					double x_add_on = 0;

					// Fill in the rest of the path points, after we already added old points
					for (int i = 0; i < path_size - prev_size; ++i) {
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