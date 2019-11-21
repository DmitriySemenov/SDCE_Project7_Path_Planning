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
#include "constants.h"
#include "debug.h"
#include "Vehicle.h"

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

	// current time
	double curr_time = 0;

	// Our vehicle
	Vehicle our_veh;

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
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &cmd_vel, &max_s, &curr_time, &our_veh]
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
					// Convert car_speed from MPH to m/s
					car_speed /= 2.237;

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
					
					Step 0: Generate smooth waypoints near car. Use spline library.

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

					// Create data logging file for debug:
					// Append to existing file
					std::ofstream debug_log ("debug_log.csv", std::ofstream::app);
					
					curr_time += T_STEP;
					#ifdef DEBUG_CARSTATS
					debug_log << "Current Time: " << curr_time << std::endl;
					debug_log << "Car X, Car Y, Car S, Car D, Car Yaw, Car Speed (m/s)" << std::endl;
					debug_log << car_x << ", " << car_y << ", " << car_s << ", " << car_d << ", " << car_yaw << ", " << car_speed << std::endl;
					#endif
					// Step 0: Generate smooth waypoints near car. Use spline library.
					/////////////////START STEP 0 ////////////////////////////////////

					// Original waypoints to use for smooth waypoint generation
					vector<double> orig_map_waypoints_x;
					vector<double> orig_map_waypoints_y;
					vector<double> orig_map_waypoints_s;
					vector<double> orig_map_waypoints_dx;
					vector<double> orig_map_waypoints_dy;
					// New, smooth, waypoints
					vector<double> smooth_map_waypoints_x;
					vector<double> smooth_map_waypoints_y;
					vector<double> smooth_map_waypoints_s;
					vector<double> smooth_map_waypoints_dx;
					vector<double> smooth_map_waypoints_dy;

					int num_waypoints = map_waypoints_x.size();
					int next_wp_idx = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);

					for (int i = -ORIG_WP_BEHIND; i < ORIG_WP_AHEAD; ++i) {
						// Account for underflow by adding num_waypoints and for overflow by doing % num_waypoints.
						int orig_wp_idx = (next_wp_idx + i + num_waypoints) % num_waypoints;
						
						// Modify s position due to spline function requirement to have x-axis points monotonically increasing
						// Let modified s coordinate go negative or above max for spline input, but adjust s coordinate later to be within (0, max_s) bounds
						double orig_s = map_waypoints_s[orig_wp_idx];
						double car_next_wp_s = map_waypoints_s[next_wp_idx];

						if (i < 0 && orig_s > car_next_wp_s) {
							orig_s -= max_s;
						}
						if (i > 0 && orig_s < car_next_wp_s) {
							orig_s += max_s;
						}
						orig_map_waypoints_s.push_back(orig_s);
						orig_map_waypoints_x.push_back(map_waypoints_x[orig_wp_idx]);
						orig_map_waypoints_y.push_back(map_waypoints_y[orig_wp_idx]);
						orig_map_waypoints_dx.push_back(map_waypoints_dx[orig_wp_idx]);
						orig_map_waypoints_dy.push_back(map_waypoints_dy[orig_wp_idx]);
					}

					// Figure out number of smooth points, based on distance between points
					int num_smooth_points = (orig_map_waypoints_s[orig_map_waypoints_s.size() - 1] - orig_map_waypoints_s[0]) / SMOOTH_WP_DIST;
					double s_wp_start = orig_map_waypoints_s[0];
					// Adjust starting s coordinate later to be within(0, max_s) bounds
					if (s_wp_start < 0) {
						s_wp_start += max_s;
					}
					if (s_wp_start > max_s) {
						s_wp_start -= max_s;
					}

					for (int i = 0; i < num_smooth_points; i++) {
						double new_smooth_s = s_wp_start + i * SMOOTH_WP_DIST;
						// Adjust s coordinate to be less than max_s bound
						if (new_smooth_s > max_s) {
							new_smooth_s -= max_s;
						}
						smooth_map_waypoints_s.push_back(new_smooth_s);
					}
					smooth_map_waypoints_x = smooth_waypoints(orig_map_waypoints_s, orig_map_waypoints_x, SMOOTH_WP_DIST, num_smooth_points);
					smooth_map_waypoints_y = smooth_waypoints(orig_map_waypoints_s, orig_map_waypoints_y, SMOOTH_WP_DIST, num_smooth_points);
					smooth_map_waypoints_dx = smooth_waypoints(orig_map_waypoints_s, orig_map_waypoints_dx, SMOOTH_WP_DIST, num_smooth_points);
					smooth_map_waypoints_dy = smooth_waypoints(orig_map_waypoints_s, orig_map_waypoints_dy, SMOOTH_WP_DIST, num_smooth_points);

					#ifdef DEBUG_WAYPOINTS

					vector<double> car_smooth_xy = getXY(car_s, car_d, smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);
					debug_log << "SMOOTH OUTPUT" << std::endl;
					debug_log << "Car X, Car Y, Car S, Car D, Car Speed (m/s)" << std::endl;
					debug_log << car_smooth_xy[0] << ", " << car_smooth_xy[1] << ", " << car_s << ", " << car_d << ", " << car_speed << std::endl;

					debug_log << "modified original waypoints" << std::endl << "orig s, orig x, orig y, orig dx, orig dy" << std::endl;
					for (int i = 0; i < orig_map_waypoints_s.size(); ++i) {
						debug_log << orig_map_waypoints_s[i] << ", " << orig_map_waypoints_x[i] << ", " << orig_map_waypoints_y[i] << ", " << orig_map_waypoints_dx[i] << ", "<< orig_map_waypoints_dy[i] << std::endl;
						}
					debug_log << std::endl;

					debug_log << "smooth waypoints" << std::endl << "smooth s, smooth x, smooth y, smooth dx, smooth dy" << std::endl;
					for (int i = 0; i < smooth_map_waypoints_s.size(); ++i) {
						debug_log << smooth_map_waypoints_s[i] << ", " << smooth_map_waypoints_x[i] << ", " << smooth_map_waypoints_y[i] << ", " << smooth_map_waypoints_dx[i] << ", " << smooth_map_waypoints_dy[i] << std::endl;
					}
					#endif
					/////////////////END STEP 0 ////////////////////////////////////

					////////////// START STEP 1 ////////////////////////////////////
					/*
					Step 1: Figure out time offset(to predict other car positions) based on prev path size
										time_offset = prev_path_size * T_STEP
									Initialize our car using end point of prev path converted from (x, y) to (s, d)
									Initialize other cars using sensor fusion data and generate predicitons of positions, starting at time_offset
					*/

					int prev_size = previous_path_x.size();

					double time_offset = prev_size * T_STEP;

					// Initialization of our car

					double our_veh_x, our_veh_y, our_veh_angle;
					double our_veh_s, our_veh_s_dot, our_veh_s_ddot;
					double our_veh_d, our_veh_d_dot, our_veh_d_ddot;

					// use default values if not enough previous path points
					if (prev_size < 4) {
						our_veh_x = car_x;
						our_veh_y = car_y;
						our_veh_angle = deg2rad(car_yaw);
						our_veh_s = car_s;
						our_veh_d = car_d;
						our_veh_s_dot = car_speed;
						our_veh_d_dot = 0;
						our_veh_s_ddot = 0;
						our_veh_d_ddot = 0;
					}
					else {

						double  our_veh_x_prev, our_veh_y_prev, our_veh_vel_x, our_veh_vel_y, 
							our_veh_x_prev2, our_veh_y_prev2, our_veh_vel_x_prev, our_veh_vel_y_prev, 
							our_veh_acc_x, our_veh_acc_y;

						// Current vehicle position - last point of previous path
						our_veh_x = previous_path_x[prev_size - 1];
						our_veh_y = previous_path_y[prev_size - 1];

						// Previous vehicle position
						our_veh_x_prev = previous_path_x[prev_size - 2];
						our_veh_y_prev = previous_path_y[prev_size - 2];

						// Calculate s and d
						our_veh_angle = atan2(our_veh_y - our_veh_y_prev, our_veh_x - our_veh_x_prev);
						vector<double> sd_pos = getFrenet(our_veh_x, our_veh_y, our_veh_angle, smooth_map_waypoints_x, smooth_map_waypoints_y, smooth_map_waypoints_s);
						our_veh_s = sd_pos[0];
						our_veh_d = sd_pos[1];

						// Determine dx, dy vector from smooth waypoints
						// Since smooth waypoints are close enough together, dx, dy can be reused for previous two points 
						// to calculate velocity and acceleration
						int next_wp_index = NextWaypoint(our_veh_x, our_veh_y, our_veh_angle, smooth_map_waypoints_x, smooth_map_waypoints_y);
						double dx = smooth_map_waypoints_dx[next_wp_index - 1];
						double dy = smooth_map_waypoints_dy[next_wp_index - 1];

						// sx,sy is perpendicular to dx,dy
						double sx = -dy;
						double sy = dx;

						our_veh_vel_x = (our_veh_x - our_veh_x_prev) / T_STEP;
						our_veh_vel_y = (our_veh_y - our_veh_y_prev) / T_STEP;

						// Calculate s_dot & d_dot
						// Project V (vx,vy) velocity vector onto S (sx,sy) and D (dx,dy) vectors
						// Use dot products of V with S and V with D
						our_veh_s_dot = our_veh_vel_x * sx + our_veh_vel_y * sy;
						our_veh_d_dot = our_veh_vel_x * dx + our_veh_vel_y * dy;

						// calculate s_doubledot, d_doubledot from xy acceleration
						our_veh_x_prev2 = previous_path_x[prev_size - 3];
						our_veh_y_prev2 = previous_path_y[prev_size - 3];
						our_veh_vel_x_prev = (our_veh_x_prev - our_veh_x_prev2) / T_STEP;
						our_veh_vel_y_prev = (our_veh_y_prev - our_veh_y_prev2) / T_STEP;
						our_veh_acc_x = (our_veh_vel_x - our_veh_vel_x_prev) / T_STEP;
						our_veh_acc_y = (our_veh_vel_y - our_veh_vel_y_prev) / T_STEP;
						our_veh_s_ddot = our_veh_acc_x * sx + our_veh_acc_y * sy;
						our_veh_d_ddot = our_veh_acc_x * dx + our_veh_acc_y * dy;

					}

					our_veh.s = our_veh_s;
					our_veh.s_dot = our_veh_s_dot;
					our_veh.s_doubledot = our_veh_s_ddot;
					our_veh.d = our_veh_d;
					our_veh.d_dot = our_veh_d_dot;
					our_veh.d_doubledot = our_veh_d_ddot;

					// Initialization and prediction of other cars
					vector<Vehicle> other_veh;

					for (int i = 0; i < sensor_fusion.size(); ++i) {
						double other_veh_s = sensor_fusion[i][5];
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double other_veh_s_dot = sqrt(vx * vx + vy * vy);
						double other_veh_s_doubledot = 0;
						double other_veh_d = sensor_fusion[i][6];
						double other_veh_d_dot = 0;
						double other_veh_d_doubledot = 0;
						
						other_veh.push_back(Vehicle(other_veh_s, other_veh_s_dot, other_veh_s_doubledot, other_veh_d, other_veh_d_dot, other_veh_d_doubledot));
						other_veh[i].generate_predictions(time_offset);
					}
					////////////// END STEP 1   ////////////////////////////////////
					
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

					vector<double> next_x_vals;
					vector<double> next_y_vals;
					vector<double> rough_x_vals;
					vector<double> rough_y_vals;

					FindClosestCars(sensor_fusion, sensor_fusion.size(), car_s, max_s, closest_ahead_speed, closest_ahead_dist, closest_behind_speed, closest_behind_dist);

					// check if there is a car in front that is too close
					// maintain speed of the car ahead, once get too close
					bool too_close = false;

					if (closest_ahead_dist[car_lane] < slowdown_dist) {
						if (closest_ahead_dist[car_lane] < brake_dist) {
							ref_vel = std::max(closest_ahead_speed[car_lane] - 5.0, 0.0);
						}
						else {
							ref_vel = closest_ahead_speed[car_lane];
						}
						

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
					vector<double> start_xy_new = getXY(fmod(car_s + 30, max_s), (2.0 + 4.0 * target_lane), smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);
					vector<double> mid_xy_new		= getXY(fmod(car_s + 60, max_s), (2.0 + 4.0 * target_lane), smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);
					vector<double> final_xy_new = getXY(fmod(car_s + 90, max_s), (2.0 + 4.0 * target_lane), smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);
					
					vector<double> start_xy = getXY(fmod(car_s + 30, max_s), (2.0 + 4.0 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> mid_xy = getXY(fmod(car_s + 60, max_s), (2.0 + 4.0 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> final_xy = getXY(fmod(car_s + 90, max_s), (2.0 + 4.0 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

					start_xy = start_xy_new;
					mid_xy = mid_xy_new;
					final_xy = final_xy_new;

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
					
					// Close log file
					debug_log.close();

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