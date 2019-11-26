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
									For each state target s and d (s_t, d_t), generate (RND_TGT_COUNT) number of randomized target states (s_r, d_r) using gaussian distribution.
									Mu = s_t or d_t; Sigma = Use constants from constants file.

					Step 4: For each (s_r, d_r) generate JMT coefficients for car_s -> s_r and car_d -> d_r, assuming TRAJ_TIME second(s) time for reaching target.

					Step 5: Calculate total cost for each set of JMT coeff. Use original (s_t, d_t) for each state.
								cost function:			weight:
								s_diff_cost					++
								d_diff_cost					++++
								t_diff_cost					++
								average_spd_cost		+++
								max_jerk_cost				+++++
								collision_cost			+++++
								buffer_cost					++
								max_accel_cost			+++++

							Find minimum cost trajectory coefficients.

					Step 6: Add on new targets to the prev_path existing ones, until reaching PATH_SIZE number of target points. 
									Use conversion function to convert from (s,d) to (x,y) and smoothed our waypoints.

					*/

					// Create data logging file for debug:
					// Append to existing file
					std::ofstream debug_log ("debug_log.csv", std::ofstream::app);
					debug_log << "########, ########, ########, ########, ########, ########, ########, ########, ########, ########, ########, ########, ########, ########, ########";
					debug_log << std::endl;

					if (previous_path_x.size() > 0)
						curr_time += (PATH_SIZE - previous_path_x.size())* T_STEP;

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

										cost function:						weight:
										collision_cost						+++++
										buffer_cost								++
										exceeds_speed_limit_cost	+++++
										high_spd_cost							+++
										max_jerk_cost							+++++
										max_accel_cost						+++++

										Find the lowest cost trajectory and use it as target trajectory.
					*/

					int best_traj_idx = 0;
					double best_traj_cost = 99999;

					for (int tr = 0; tr < our_veh.target_s.size(); ++tr) {
						vector<vector<double>> trajectory;
						double traj_cost = 99999;

						trajectory = our_veh.generate_traj_for_target(tr);
						traj_cost = total_traj_cost(trajectory, other_veh);
						if (traj_cost < best_traj_cost) {
							best_traj_cost = traj_cost;
							best_traj_idx = tr;
						}
					}

					vector<vector<double>> trajectory = our_veh.generate_traj_for_target(best_traj_idx);

					#ifdef DEBUG_TRAJ
					double debug_target_s = our_veh.target_s[best_traj_idx];
					double debug_target_d = our_veh.target_d[best_traj_idx];

					debug_log << "BEST TRAJECTORY: " << std::endl;
					debug_log << "TGT S, TGT D";
					debug_log << std::endl;
					debug_log << debug_target_s << ", " << debug_target_d << std::endl;

					debug_log << "TRAJ S1";
					for (unsigned int i = 1; i < trajectory.size(); ++i)
						debug_log << ", TRAJ S" << i + 1;
					debug_log << std::endl;

					debug_log << trajectory[0][0];
					for (unsigned int i = 1; i < trajectory.size(); ++i)
						debug_log << ", " << trajectory[i][0];
					debug_log << std::endl;

					debug_log << "TRAJ D1";
					for (unsigned int i = 1; i < trajectory.size(); ++i)
						debug_log << ", TRAJ D" << i + 1;
					debug_log << std::endl;

					debug_log << trajectory[0][1];
					for (unsigned int i = 1; i < trajectory.size(); ++i)
						debug_log << ", " << trajectory[i][1];
					debug_log << std::endl;
					#endif
					////////////// END STEP 5   ////////////////////////////////////

					////////////// START STEP 6   //////////////////////////////////
					/* 
							Step 6: Add on new targets to the prev_path existing ones, until reaching PATH_SIZE number of target points. 
											Use conversion function to convert from (s,d) to (x,y) and smoothed out waypoints.
					*/

					////////////// END STEP 6   ////////////////////////////////////
					
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					// Add old path points to the new path first
					for (int i = 0; i < prev_size; ++i) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					// Fill in the rest of the path points, after we already added old points
					for (unsigned int i = 0; i < PATH_SIZE - prev_size; ++i) {
						vector<double> nextXY;
						double next_s = trajectory[i][0];
						double next_d = trajectory[i][1];

						nextXY = getXY(next_s, next_d, smooth_map_waypoints_s, smooth_map_waypoints_x, smooth_map_waypoints_y);

						next_x_vals.push_back(nextXY[0]);
						next_y_vals.push_back(nextXY[1]);
					}

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