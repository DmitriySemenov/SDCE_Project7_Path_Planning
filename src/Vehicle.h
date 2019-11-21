#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
public:

	double s;
	double s_dot;
	double s_doubledot;
	double d;
	double d_dot;
	double d_doubledot;
	string state;
	vector<string> available_states;
	vector<double> s_traj_coeffs, d_traj_coeffs;

	// State predictions for other vehicles
	vector<Vehicle> predictions;

	// Constructors
	Vehicle();
	Vehicle(double s, double s_dot, double s_doubledot, double d, double d_dot, double d_doubledot);

	// Destructor
	virtual ~Vehicle();

	void generate_predictions(double time_offset);
};

#endif  // VEHICLE_H
