#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using std::map;
using std::string;
using std::vector;

class Vehicle {
public:

	// Variables
	double s;
	double s_dot;
	double s_doubledot;
	double d;
	double d_dot;
	double d_doubledot;
	unsigned int lane;
	string state;
	vector<string> available_states;
	vector<double> target_s, target_d;
	vector<vector<double>> s_traj_coeffs, d_traj_coeffs;
	double car_ahead_dist[3];
	double car_behind_dist[3];
	double car_ahead_spd[3];

	// State predictions for other vehicles
	vector<Vehicle> predictions;

	// Constructors
	Vehicle();
	Vehicle(double s, double s_dot, double s_doubledot, double d, double d_dot, double d_doubledot);

	// Destructor
	virtual ~Vehicle();

	// Functions
	void upd_closest_veh(const vector<Vehicle>& other_vehs);
	void upd_lane(double d_curr);
	void upd_available_states(vector<Vehicle> &other_vehicles);
	void gen_targets();
	void perturb_target(double s_t, double d_t);
	void generate_predictions(double time_offset);
	void generate_coeffs_for_targets();
	vector<vector<double>> generate_traj_for_target(int target_index);
	vector<double> JMT(vector<double>& start, vector<double>& end, double T);
};

#endif  // VEHICLE_H
