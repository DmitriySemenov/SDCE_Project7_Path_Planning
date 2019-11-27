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
#include "cost.h"

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


	double prev_target_d = 6;

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
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, & prev_target_d, &cmd_vel, &max_s, &curr_time, &our_veh]
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

					#ifdef DEBUG_ENA
					// Create data logging file for debug:
					// Append to existing file
					std::ofstream debug_log ("debug_log.csv", std::ofstream::app);
					debug_log << "########, ########, ########, ########, ########, ########, ########, ########, ########, ########, ########, ########, ########, ########, ########";
					debug_log << std::endl;
					if (previous_path_x.size() > 0)
						curr_time += (PATH_SIZE - previous_path_x.size()) * T_STEP;
					#endif

					

					#ifdef DEBUG_CARSTATS
					debug_log << "Current Time: " << curr_time << std::endl;
					debug_log << "Car X, Car Y, Car S, Car D, Car Yaw, Car Speed (m/s)" << std::endl;
					debug_log << car_x << ", " << car_y << ", " << car_s << ", " << car_d << ", " << car_yaw << ", " << car_speed << std::endl;
					#endif
					
					/////////////////START STEP 0 ////////////////////////////////////
					// Step 0: Generate smooth waypoints near our car. Use spline library.

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
					int num_smooth_points = (int)((orig_map_waypoints_s[orig_map_waypoints_s.size() - 1] - orig_map_waypoints_s[0]) / SMOOTH_WP_DIST);
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

					unsigned int prev_size = previous_path_x.size();
					prev_size = min(prev_size, PREV_PATH_MAX);
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
						if (next_wp_index == 0)
							next_wp_index = smooth_map_waypoints_dx.size();
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

					for (unsigned int i = 0; i < sensor_fusion.size(); ++i) {
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
					////////////// START STEP 2   //////////////////////////////////
					/*
							Step 2: Based on our_veh position in (s, d) and other_veh predictions, figure out available operating_state.
											Possible states: Keep Lane(KL), Lane Change Left(LCL), Lane Change Right(LCR)
											Can't do LCL in left most lane (0) and can't do LCR in right most lane(2).
					*/

					our_veh.upd_available_states(other_veh);

					#ifdef DEBUG_CARPRED
					debug_log << "Predict Time: " << (curr_time + time_offset) << std::endl;

					debug_log << "S, D, S_DOT, S_DBL_DOT, D_D, D_DBL_DOT" << std::endl;
					debug_log << our_veh.s << ", " << our_veh.d << ", " << our_veh.s_dot << ", " << our_veh.s_doubledot << ", " << our_veh.d_dot << ", " << our_veh.d_doubledot << std::endl;

					debug_log << "Other Cars (S & Lane):" << std::endl;
					debug_log << other_veh[0].predictions[0].s << ", " << other_veh[0].predictions[0].lane;
					for (unsigned int i = 1; i < other_veh.size(); ++i)
						debug_log << ", " << other_veh[i].predictions[0].s << ", " << other_veh[i].predictions[0].lane;
					debug_log << std::endl;

					debug_log << "Our Lane, Available States:" << std::endl;
					debug_log << our_veh.lane;
					for (unsigned int i = 0; i < our_veh.available_states.size(); ++i)
						debug_log << ", " << our_veh.available_states[i];
					debug_log << std::endl;
					#endif
					////////////// END STEP 2   ////////////////////////////////////
					////////////// START STEP 3   //////////////////////////////////
					/*
							Step 3: For each available operating_state generate target positions in (s, d) projected into the future with T = TRAJ_TIME sec.
									For KL : d = middle of car_lane,
									LCL : d = middle of the lane to the left of car_lane
									LCR : d = middle of the lane to the right of car_lane
									For all states : s = project s 1 second ahead using current speed and accel, but considering max speed limit.
									
									Using these KL, LCL, LCR targets (s_tgt,d_tgt), for each available state
									generate RND_TGT_COUNT(10) targets using gaussian distribution and sigma defined in costants.h
					*/

					our_veh.gen_targets();

					#ifdef DEBUG_TARGETS
					debug_log << "Target Positions At Time: " << (curr_time + time_offset + TRAJ_TIME) << std::endl;

					debug_log << "TGT S1";
					for (unsigned int i = 1; i < our_veh.target_s.size(); ++i)
						debug_log << ", TGT S" << i+1;
					debug_log << std::endl;
					
					debug_log << our_veh.target_s[0];
					for (unsigned int i = 1; i < our_veh.target_s.size(); ++i)
						debug_log << ", " << our_veh.target_s[i];
					debug_log << std::endl;
					

					debug_log << "TGT D1";
					for (unsigned int i = 1; i < our_veh.target_d.size(); ++i)
						debug_log << ", TGT D" << i+1;
					debug_log << std::endl;

					debug_log << our_veh.target_d[0];
					for (unsigned int i = 1; i < our_veh.target_d.size(); ++i)
						debug_log << ", " << our_veh.target_d[i];
					debug_log << std::endl;
					#endif
					////////////// END STEP 3   ////////////////////////////////////

					////////////// START STEP 4   //////////////////////////////////
					/* 
							Step 4: For each(s_t, d_t) generate JMT coefficients for each car_s->s_t car_d->d_t,
										assuming TRAJ_TIME second(s) time for reaching target. 
					*/
					our_veh.generate_coeffs_for_targets();		
					////////////// END STEP 4   ////////////////////////////////////

					////////////// START STEP 5   //////////////////////////////////
					/* 
							Step 5: For each target, generate trajectory and calculate total cost using multiple cost functions.
											Find the lowest cost trajectory and use it as target trajectory.
					*/

					int best_traj_idx = 0;
					double best_traj_cost = 99999;

					for (int tr = 0; tr < our_veh.target_s.size(); ++tr) {
						vector<vector<double>> trajectory;

						trajectory = our_veh.generate_traj_for_target(tr);

						double traj_cost = 0;
						double coll = collision_cost(trajectory, other_veh) * COLLISION_W;
						double buff = buffer_cost(trajectory, other_veh) * BUFF_W;
						double notmid = not_middle_lane_cost(trajectory) * NOTMID_W;
						double tooclose = too_close_cost(trajectory, other_veh) * TOOCLOSE_W;
						double spdlim = exceeds_speed_limit_cost(trajectory) * SPEEDLIM_W;
						double highspd = high_spd_cost(trajectory) * HIGHSPD_W;
						double maxaccel = exceeds_accel_limit_cost(our_veh.s_dot, trajectory) * MAXACCEL_W;
						double notfastest = not_fastest_lane_cost(our_veh, trajectory) * NOTFASTEST_W;
						double diffd = diffd_cost(prev_target_d, trajectory) * DIFFD_W;

						traj_cost = coll + buff + notmid + tooclose + spdlim + highspd + maxaccel + notfastest + diffd;

						#ifdef DEBUG_COSTS
						debug_log << "TRAJECTORY " << tr << " COSTS" << std::endl;
						debug_log << "COLL, BUFF, NOTMID, TOOCLOSE, SPDLIM, HS, ACC, NOTFASTEST, TOT" << std::endl;
						debug_log << coll << ", ";
						debug_log << buff << ", ";
						debug_log << notmid << ", ";
						debug_log << tooclose << ", ";
						debug_log << spdlim << ", ";
						debug_log << highspd << ", ";
						debug_log << maxaccel << ", ";
						debug_log << notfastest << ", ";
						debug_log << traj_cost << std::endl;
						#endif			

						if (traj_cost < best_traj_cost) {
							best_traj_cost = traj_cost;
							best_traj_idx = tr;
						}
					}

					vector<vector<double>> trajectory = our_veh.generate_traj_for_target(best_traj_idx);
					double target_s = our_veh.target_s[best_traj_idx];
					double target_d = our_veh.target_d[best_traj_idx];
					
					////////////// END STEP 5   ////////////////////////////////////

					////////////// START STEP 6   //////////////////////////////////
					/* 
							Step 6: Add on new targets to the prev_path existing ones, until reaching PATH_SIZE number of target points. 
											Use conversion function to convert from (s,d) to (x,y) and smoothed out waypoints.
					*/

					/*
					// reference position
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					vector<double> next_x_vals;
					vector<double> next_y_vals;
					vector<double> rough_x_vals;
					vector<double> rough_y_vals;

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
					double half_d;
					double quarter_d;
					if (our_veh.d > target_d)
						quarter_d = our_veh.d - (our_veh.d - target_d) / 4;
					else
						quarter_d = our_veh.d + (target_d - our_veh.d) / 4;

					if (our_veh.d > target_d)
						half_d = our_veh.d - (our_veh.d - target_d) / 2;
					else
						half_d = our_veh.d + (target_d - our_veh.d) / 2;

					vector<double> quarter_xy = getXY((our_veh.s + (target_s - our_veh.s) / 3), quarter_d, smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);
					vector<double> half_xy = getXY((our_veh.s + (target_s - our_veh.s) / 1.5), half_d, smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);
					vector<double> target_xy = getXY(target_s, target_d, smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);
					vector<double> ahead_xy = getXY((target_s + 30.0), target_d, smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);
					vector<double> far_ahead_xy = getXY((target_s + 60.0), target_d, smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);

					rough_x_vals.push_back(quarter_xy[0]);
					rough_x_vals.push_back(half_xy[0]);
					rough_x_vals.push_back(target_xy[0]);
					rough_x_vals.push_back(ahead_xy[0]);
					rough_x_vals.push_back(far_ahead_xy[0]);

					rough_y_vals.push_back(quarter_xy[1]);
					rough_y_vals.push_back(half_xy[1]);
					rough_y_vals.push_back(target_xy[1]);
					rough_y_vals.push_back(ahead_xy[1]);
					rough_y_vals.push_back(far_ahead_xy[1]);

					// Shift car's reference angle to 0 degrees
					for (unsigned int i = 0; i < rough_x_vals.size(); ++i) {
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

					// Commanded velocity
					cmd_vel = (target_s - our_veh.s) / TRAJ_TIME;

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
					for (unsigned int i = 0; i < PATH_SIZE - prev_size; ++i) {
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
					*/

					////////////// END STEP 6   ////////////////////////////////////
					
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					vector<double> rough_s_traj;
					vector<double> rough_x_traj;
					vector<double> rough_y_traj;
					vector<double> smooth_s_traj;
					vector<double> smooth_x_traj;
					vector<double> smooth_y_traj;

					double prev_s = our_veh.s - our_veh.s_dot * T_STEP;

					// Create initial 2 rough start points using current car position or previous path last 2 points
					if (prev_size >= 2) {
						rough_s_traj.push_back(prev_s);
						rough_x_traj.push_back(previous_path_x[prev_size - 2]);
						rough_y_traj.push_back(previous_path_y[prev_size - 2]);
						rough_s_traj.push_back(our_veh.s);
						rough_x_traj.push_back(previous_path_x[prev_size - 1]);
						rough_y_traj.push_back(previous_path_y[prev_size - 1]);
					}
					else 
					{
						prev_s = our_veh_s - 1;
						double prev_x = our_veh_x - cos(our_veh_angle);
						double prev_y = our_veh_y - sin(our_veh_angle);
						rough_s_traj.push_back(prev_s);
						rough_x_traj.push_back(prev_x);
						rough_y_traj.push_back(prev_y);
						rough_s_traj.push_back(our_veh.s);
						rough_x_traj.push_back(our_veh_x);
						rough_y_traj.push_back(our_veh_y);
					}

					// Account for bad detection of lane by simulator
					if (target_d > 8 && our_veh_s > BADRIGHTLANE_S_MIN && our_veh_s < BADRIGHTLANE_S_MAX)
						target_d = 9.4;

					// Other two points of rough trajectory use target_d and current s + 30, current s + 60
					double target_s3 = our_veh_s + 30;
					double target_d3 = target_d;
					vector<double> target_xy3 = getXY(target_s3, target_d3, smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);
					double target_x3 = target_xy3[0];
					double target_y3 = target_xy3[1];
					rough_s_traj.push_back(target_s3);
					rough_x_traj.push_back(target_x3);
					rough_y_traj.push_back(target_y3);

					double target_s4 = target_s3 + 30;
					double target_d4 = target_d;
					vector<double> target_xy4 = getXY(target_s4, target_d4, smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);
					double target_x4 = target_xy4[0];
					double target_y4 = target_xy4[1];
					rough_s_traj.push_back(target_s4);
					rough_x_traj.push_back(target_x4);
					rough_y_traj.push_back(target_y4);

					// Generate smooth path s values, using ACCEL_LIMIT and target_s
					if (target_s < our_veh_s)
						target_s += MAXIMUM_S;
					double target_s_dot = (target_s - our_veh.s) / TRAJ_TIME;
					// Make sure to slow down if get too close
					if (our_veh.car_ahead_dist[our_veh.lane] < TOOCLOSE_DIST)
						target_s_dot = our_veh.car_ahead_spd[our_veh.lane] - 2.0;
					double current_s = our_veh.s;
					double current_s_dot = our_veh.s_dot;
					for (int i = 0; i < (PATH_SIZE - prev_size); i++) {
						double v_incr;
						if (fabs(target_s_dot - current_s_dot) < 3 * ACCEL_LIMIT * T_STEP) {
							v_incr = 0;
						}
						else {
							if (target_s_dot > current_s_dot)
								v_incr = ACCEL_LIMIT * T_STEP;
							else
								v_incr = -DECEL_LIMIT * T_STEP;
						}
						current_s_dot += v_incr;
						current_s += current_s_dot * T_STEP;
						smooth_s_traj.push_back(current_s);

					}

					smooth_x_traj = smooth_traj_points(rough_s_traj, rough_x_traj, smooth_s_traj);
					smooth_y_traj = smooth_traj_points(rough_s_traj, rough_y_traj, smooth_s_traj);

					// Add previous path points to next path
					for (int i = 0; i < prev_size; i++) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}
					// Add new xy points
					for (int i = 0; i < smooth_x_traj.size(); i++) {
						next_x_vals.push_back(smooth_x_traj[i]);
						next_y_vals.push_back(smooth_y_traj[i]);
					}

					#ifdef DEBUG_TRAJ
					double debug_target_s = our_veh.target_s[best_traj_idx];
					double debug_target_d = our_veh.target_d[best_traj_idx];

					std::cout.precision(3);
					std::cout << "D: " << debug_target_d;
					std::cout << " S: " << car_s;
					std::cout << " SPD: " << target_s_dot * 2.237;
					std::cout << " COLL: " << collision_cost(trajectory, other_veh)* COLLISION_W;
					std::cout << " BUFF: " << buffer_cost(trajectory, other_veh)* BUFF_W;
					std::cout << " NOTM: " << not_middle_lane_cost(trajectory)* NOTMID_W;
					std::cout << " TOOCL: " << too_close_cost(trajectory, other_veh) * TOOCLOSE_W;
					std::cout << " SPDLM: " << exceeds_speed_limit_cost(trajectory)* SPEEDLIM_W;
					std::cout << " HS: " << high_spd_cost(trajectory)* HIGHSPD_W;
					std::cout << " ACC: " << exceeds_accel_limit_cost(our_veh.s_dot, trajectory)* MAXACCEL_W;
					std::cout << " NOTFST: " << not_fastest_lane_cost(our_veh, trajectory)* NOTFASTEST_W;
					std::cout << " DIFF: " << diffd_cost(prev_target_d, trajectory)* DIFFD_W << "      \r";


					debug_log << "BEST TRAJECTORY: " << std::endl;
					debug_log << "COST, IDX, TGT S, TGT D, TGT SPD";
					debug_log << std::endl;
					debug_log << best_traj_cost << ", " << best_traj_idx << ", " << debug_target_s << ", " << debug_target_d << ", " << target_s_dot * 2.237 << std::endl;
					#endif

					#ifdef DEBUG_NEXTPATH
					debug_log << "Next X Vals:" << std::endl;
					debug_log << next_x_vals[0];
					for (unsigned int i = 1; i < next_x_vals.size(); ++i)
						debug_log << ", " << next_x_vals[i];
					debug_log << std::endl;
					debug_log << "Next Y Vals:" << std::endl;
					debug_log << next_y_vals[0];
					for (unsigned int i = 1; i < next_y_vals.size(); ++i)
						debug_log << ", " << next_y_vals[i];
					debug_log << std::endl;
					#endif
          
					prev_target_d = target_d;

					msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					
					#ifdef DEBUG_ENA
					// Close log file
					debug_log.close();
					#endif

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