#ifndef COST_H
#define COST_H

#include <vector>
#include <algorithm>
#include <cmath>
#include "constants.h"
#include "Vehicle.h"
using std::vector;
using std::min;
using std::max;


/////////// UTILITY FUNCTIONS /////////////////////
double logistic(double x) {
	// A function that returns a value between 0 and 1 for x in the range[0, infinity] 
	// and - 1 to 1 for x in the range[-infinity, infinity]. 
	// Useful for cost functions.
	return 2.0 / (1 + exp(-x)) - 1.0;
}

double nearest_approach(vector<vector<double>> &traj, vector<Vehicle> &prediction) {
	// Calculates the closest distance to a vehicle during a trajectory.
	double closest = 999999;
	for (int i = 0; i < TRAJ_SIZE; i++) {
		double traj_s = traj[i][0];
		double traj_d = traj[i][1];

		// Account for overrun
		double min_s = min(traj_s, prediction[i].s);
		double max_s = max(traj_s, prediction[i].s);
		double norm_dist_s = fabs(max_s - min_s);
		double adj_dist_s = fabs(max_s - (min_s + MAXIMUM_S));
		double actual_dist_s = min(norm_dist_s, adj_dist_s);

		double current_dist = sqrt(pow(actual_dist_s, 2) + pow(traj_d - prediction[i].d, 2));
		if (current_dist < closest) {
			closest = current_dist;
		}
	}
	return closest;
}

double nearest_approach_to_any_vehicle(vector<vector<double>> &traj, vector<Vehicle> &vehicles) {
	// Calculates the closest distance to any vehicle during a trajectory.
	double closest = 999999;
	for (int i = 0; i < vehicles.size(); i++) {
		double current_dist = nearest_approach(traj, vehicles[i].predictions);
		if (current_dist < closest) {
			closest = current_dist;
		}
	}
	return closest;
}

double nearest_approach_to_vehicle_in_lane(vector<vector<double>>& traj, vector<Vehicle>& vehicles) {
	// Calculates the closest distance to a vehicle in lane during a trajectory.
	double closest = 999999;

	for (int i = 0; i < TRAJ_SIZE; ++i) {
		double traj_s = traj[i][0];
		double traj_d = traj[i][1];
		double curr_lane = 0;

		if (traj_d < 4)
			curr_lane = 0;
		else if (traj_d >= 4 && traj_d < 8)
			curr_lane = 1;
		else
			curr_lane = 2;

		for (int v = 0; v < vehicles.size(); ++v) {
			if (vehicles[v].lane == curr_lane) {

				// Account for overrun
				double min_s = min(traj_s, vehicles[v].predictions[i].s);
				double max_s = max(traj_s, vehicles[v].predictions[i].s);
				double norm_dist_s = fabs(max_s - min_s);
				double adj_dist_s = fabs(max_s - (min_s + MAXIMUM_S));
				double actual_dist_s = min(norm_dist_s, adj_dist_s);

				double current_dist = sqrt(pow(actual_dist_s, 2) + pow(traj_d - vehicles[v].predictions[i].d, 2));
				if (current_dist < closest) {
					closest = current_dist;
				}
			}

		}

	}
	
	return closest;
}

double nearest_s_dist_to_vehicle_in_lane(vector<vector<double>>& traj, vector<Vehicle>& vehicles) {
	// Calculates the closest distance in s-dimension to a vehicle in front in lane during a trajectory.
	double closest = 999999;

	for (int i = 0; i < TRAJ_SIZE; ++i) {
		double traj_s = traj[i][0];
		double traj_d = traj[i][1];
		double curr_lane = 0;

		if (traj_d < 4)
			curr_lane = 0;
		else if (traj_d >= 4 && traj_d < 8)
			curr_lane = 1;
		else
			curr_lane = 2;

		for (int v = 0; v < vehicles.size(); ++v) {
			if (vehicles[v].lane == curr_lane) {
				double vehicle_s = vehicles[v].predictions[i].s;

				// Account for overrun
				double min_s = min(traj_s, vehicle_s);
				double max_s = max(traj_s, vehicle_s);
				double norm_dist_s = fabs(max_s - min_s);
				double adj_dist_s = fabs(max_s - (min_s + MAXIMUM_S));
				double actual_dist_s = min(norm_dist_s, adj_dist_s);
				double current_dist;
				
				if (actual_dist_s == adj_dist_s) {
					if (min_s == traj_s)
						traj_s += MAXIMUM_S;
					else
						vehicle_s += MAXIMUM_S;
				}
				
				current_dist = vehicle_s - traj_s;

				if (current_dist < closest && current_dist > 0) {
					closest = current_dist;
				}
			}

		}

	}

	return closest;
}

///////////////////// COST FUNCTIONS ////////////////
double collision_cost(vector<vector<double>> &traj, vector<Vehicle> &vehicles) {
	// Binary cost function which penalizes collisions.
	double nearest = nearest_approach_to_vehicle_in_lane(traj, vehicles);
	if (nearest < 2 * VEH_RADIUS) {
		return 1;
	}
	else {
		return 0;
	}
}

double buffer_cost(vector<vector<double>> &traj, vector<Vehicle> &vehicles) {
	// Penalizes getting close to other vehicles.
	double nearest = nearest_s_dist_to_vehicle_in_lane(traj, vehicles);
	return logistic(2 * VEH_RADIUS / nearest);
}

double too_close_cost(vector<vector<double>>& traj, vector<Vehicle>& vehicles) {
	// Strict penalty when getting too close to other vehicles.
	double nearest = nearest_s_dist_to_vehicle_in_lane(traj, vehicles);
	if (nearest < 6 * VEH_RADIUS) {
		return logistic(2 * VEH_RADIUS / nearest);
	}
	return 0;
}

double not_middle_lane_cost(vector<vector<double>>& traj) {
	double final_d = traj[TRAJ_SIZE - 1][1];
	if (final_d < 4 || final_d > 8) {
		return 1.0;
	}
	return 0;
}

double not_fastest_lane_cost(Vehicle our_veh, vector<vector<double>>& traj) {
	double fastest_spd = 0;
	double target_d = traj[TRAJ_SIZE - 1][1];
	double target_lane_spd = 0;

	for (int l = 0; l < 3; ++l) {
		double lane_spd = our_veh.car_ahead_spd[l];
		if (lane_spd > fastest_spd)
			fastest_spd = lane_spd;
	}

	if (target_d < 4) {
		target_lane_spd = our_veh.car_ahead_spd[0];
	}
	else if (target_d >= 4 && target_d < 8) {
		target_lane_spd = our_veh.car_ahead_spd[1];
	}
	else {
		target_lane_spd = our_veh.car_ahead_spd[2];
	}

	return logistic((fastest_spd - target_lane_spd) / fastest_spd);
}

double exceeds_speed_limit_cost(vector<vector<double>> &traj) {
	// Penalizes vehicle speed in s greater than MAX_SPEED

	double final_s = traj[TRAJ_SIZE - 1][0];
	double start_s = traj[0][0];

	// Account for overrun
	double min_s = min(start_s, final_s);
	double max_s = max(start_s, final_s);
	double norm_dist_s = fabs(max_s - min_s);
	double adj_dist_s = fabs(max_s - (min_s + MAXIMUM_S));
	double actual_dist_s = min(norm_dist_s, adj_dist_s);
	if (actual_dist_s == adj_dist_s) {
		if (min_s == final_s)
			final_s += MAXIMUM_S;
		else
			start_s += MAXIMUM_S;
	}

	double s_vel = (final_s - start_s)/ TRAJ_TIME;

	if (s_vel > MAX_SPEED) {
			return 1;
	}
	return 0;
}

double exceeds_accel_limit_cost(double curr_s_dot, vector<vector<double>>& traj) {
	// Penalizes vehicle accel in s greater than MAX_SPEED

	double final_s = traj[TRAJ_SIZE - 1][0];
	double start_s = traj[0][0];

	// Account for overrun
	double min_s = min(start_s, final_s);
	double max_s = max(start_s, final_s);
	double norm_dist_s = fabs(max_s - min_s);
	double adj_dist_s = fabs(max_s - (min_s + MAXIMUM_S));
	double actual_dist_s = min(norm_dist_s, adj_dist_s);
	if (actual_dist_s == adj_dist_s) {
		if (min_s == final_s)
			final_s += MAXIMUM_S;
		else
			start_s += MAXIMUM_S;
	}

	double traj_s_dot = (final_s - start_s) / TRAJ_TIME;
	double s_accel = (curr_s_dot - curr_s_dot) / TRAJ_TIME;
	if (s_accel > MAXACCEL_W)
		return 1;
	return 0;
}

double high_spd_cost(vector<vector<double>> &traj) {
	// Rewards high speed.
	double final_s = traj[TRAJ_SIZE - 1][0];
	double start_s = traj[0][0];

	// Account for overrun
	double min_s = min(start_s, final_s);
	double max_s = max(start_s, final_s);
	double norm_dist_s = fabs(max_s - min_s);
	double adj_dist_s = fabs(max_s - (min_s + MAXIMUM_S));
	double actual_dist_s = min(norm_dist_s, adj_dist_s);
	if (actual_dist_s == adj_dist_s) {
		if (min_s == final_s)
			final_s += MAXIMUM_S;
		else
			start_s += MAXIMUM_S;
	}

	double s_vel = (final_s - start_s) / TRAJ_TIME;

	return logistic((MAX_SPEED - s_vel) / MAX_SPEED);
}

double diffd_cost(double prev_target_d, vector<vector<double>>& traj) {
	double target_d = traj[TRAJ_SIZE - 1][1];

	if (fabs(prev_target_d - target_d) > 0.01)
		return 1;
	return 0;
}

#endif