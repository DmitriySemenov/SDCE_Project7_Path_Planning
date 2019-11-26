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
							if (closest_ahead_dist[lane] > LANECHG_DIST && closest_behind_dist[lane] > LANECHG_DIST) {
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
