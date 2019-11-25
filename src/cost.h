#ifndef COST_H
#define COST_H

#include <vector>
#include <algorithm>
#include <cmath>
#include "constants.h"
#include "Vehicle.h"
using std::vector;


/////////// UTILITY FUNCTIONS /////////////////////
double logistic(double x) {
	// A function that returns a value between 0 and 1 for x in the range[0, infinity] 
	// and - 1 to 1 for x in the range[-infinity, infinity]. 
	// Useful for cost functions.
	return 2.0 / (1 + exp(-x)) - 1.0;
}

double nearest_approach(vector<vector<double>> traj, vector<Vehicle> prediction) {
	// Calculates the closest distance to a vehicle during a trajectory.
	double closest = 999999;
	for (int i = 0; i < TRAJ_SIZE; i++) {
		double traj_s = traj[i][0];
		double traj_d = traj[i][1];
		double current_dist = sqrt(pow(traj_s - prediction[i].s, 2) + pow(traj_d - prediction[i].d, 2));
		if (current_dist < closest) {
			closest = current_dist;
		}
	}
	return closest;
}

double nearest_approach_to_any_vehicle(vector<vector<double>> traj, vector<Vehicle> vehicles) {
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

vector<double> s_velocities_for_trajectory(vector<vector<double>> traj) {
	// Calculates s velocities for trajectory
	vector<double> s_velocities;
	for (int i = 1; i < traj.size(); i++) {
		s_velocities.push_back((traj[i][0] - traj[i - 1][0]) / T_STEP);
	}
	return s_velocities;
}

vector<double> d_velocities_for_trajectory(vector<vector<double>> traj) {
	// Calculates d velocities for trajectory
	vector<double> d_velocities;
	for (int i = 1; i < traj.size(); i++) {
		d_velocities.push_back((traj[i][1] - traj[i - 1][1]) / T_STEP);
	}
	return d_velocities;
}


///////////////////// COST FUNCTIONS ////////////////
double collision_cost(vector<vector<double>> traj, vector<Vehicle> vehicles) {
	// Binary cost function which penalizes collisions.
	double nearest = nearest_approach_to_any_vehicle(traj, vehicles);
	if (nearest < 2 * VEH_RADIUS) {
		return 1;
	}
	else {
		return 0;
	}
}

double buffer_cost(vector<vector<double>> traj, vector<Vehicle> vehicles) {
	// Penalizes getting close to other vehicles.
	double nearest = nearest_approach_to_any_vehicle(traj, vehicles);
	return logistic(2 * VEH_RADIUS / nearest);
}

double exceeds_speed_limit_cost(vector<vector<double>> traj) {
	// Penalizes vehicle speed in s greater than MAX_SPEED
	vector<double> s_vel_traj = s_velocities_for_trajectory(traj);
	for (double s_vel : s_vel_traj) {
		if (s_vel > MAX_SPEED) {
			return 1;
		}
	}
	return 0;
}

double high_spd_cost(vector<vector<double>> traj) {
	// Rewards high final speed.
	vector<double> s_vel_traj = s_velocities_for_trajectory(traj);
	double final_s_vel;

	final_s_vel = s_vel_traj[s_vel_traj.size() - 1];
	return logistic((MAX_SPEED - final_s_vel) / MAX_SPEED);
}

#endif