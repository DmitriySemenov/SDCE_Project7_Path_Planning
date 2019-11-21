#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include "constants.h"
//#include "cost.h"

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle() {
	this->s = 0;
	this->s_dot = 0;
	this->s_doubledot = 0;
	this->d = 0;
	this->d_dot = 0;
	this->d_doubledot = 0;
	this->lane = 0;
}

Vehicle::Vehicle(double s, double s_dot, double s_doubledot, double d, double d_dot, double d_doubledot) {
	this->s = s;
	this->s_dot = s_dot;
	this->s_doubledot = s_doubledot;
	this->d = d;
	this->d_dot = d_dot;
	this->d_doubledot = d_doubledot;
	this->state = "KL";
	this->available_states.push_back("KL");
	this->upd_lane(d);
}

Vehicle::~Vehicle() {}

void Vehicle::upd_lane(double d_curr) {
	
	if (d_curr < 4)
		this->lane = 0;
	else if (d_curr >= 4 && d_curr < 8)
		this->lane = 1;
	else
		this->lane = 2;
}

void Vehicle::upd_available_states(vector<Vehicle> &other_vehicles) {
	/* Check for available states (KL, LCL, LCR), based on current lane of the vehicle
		and predicted position of other vehicles */
	
	// Reset available states
	this->available_states.clear();

	// Keep Lane is always available
	this->available_states.push_back("KL");

	// Update current vehicle lane
	this->upd_lane(this->d);

	// Update other vehicle lane and find closest vehicle in each lane
	for (unsigned int i = 0; i < other_vehicles.size(); ++i) {
		other_vehicles[i].upd_lane(other_vehicles[i].predictions[0].d);
	}

	// Find distance to closest other vehicle in each lanes
	double car_ahead_dist[3] = { MAXIMUM_S, MAXIMUM_S, MAXIMUM_S };
	double car_behind_dist[3] = { MAXIMUM_S, MAXIMUM_S, MAXIMUM_S };

	for (unsigned int i = 0; i < other_vehicles.size(); ++i) {
		upd_closest_veh(other_vehicles, car_ahead_dist, car_behind_dist);
	}

	// Check if the vehicles in adjacent lanes are far enough to allow lane change
	// Vehicle is in the middle lane
	if (this->lane == 1) {
		// Check lane change to the left
		if ((car_ahead_dist[0] > LANECHG_DIST) && (car_behind_dist[0] > LANECHG_DIST))
			this->available_states.push_back("LCL");

		// Check lane change to the right
		if ((car_ahead_dist[2] > LANECHG_DIST) && (car_behind_dist[2] > LANECHG_DIST))
			this->available_states.push_back("LCR");
	}
	// Vehicle is in the left lane
	else if (this->lane == 0) {
		// Check lane change to the right
		if ((car_ahead_dist[1] > LANECHG_DIST) && (car_behind_dist[1] > LANECHG_DIST))
			this->available_states.push_back("LCR");
	}
	// Vehicle is in the right lane
	else {
		// Check lane change to the left
		if ((car_ahead_dist[1] > LANECHG_DIST) && (car_behind_dist[1] > LANECHG_DIST))
			this->available_states.push_back("LCL");
	}
}

void Vehicle::upd_closest_veh(const vector<Vehicle>& other_vehs, double (&car_ahead_dist)[3], double (&car_behind_dist)[3]) {

	// Find the closest car ahead and behind our car at start of prediction time
	// since our car position is at start of predicted position
	for (unsigned int i = 0; i < other_vehs.size(); ++i) {

		double d = other_vehs[i].predictions[0].d;
		double check_car_speed = other_vehs[i].predictions[0].s_dot;
		double check_car_s = other_vehs[i].predictions[0].s;
		double check_car_dist = MAXIMUM_S;
		double main_car_s = this->s;

		for (unsigned int lane = 0; lane < 3; ++lane) {
			// for every lane, find the car closest ahead and behind us
			if (d < (2.0 + 4.0 * lane + 2.0) && d >(2.0 + 4.0 * lane - 2.0)) {

				// car ahead
				if (check_car_s >= main_car_s) {
					check_car_dist = check_car_s - main_car_s;
					if (check_car_dist < car_ahead_dist[lane]) {
						car_ahead_dist[lane] = check_car_dist;
					}
				}
				// car behind
				else {
					check_car_dist = main_car_s - check_car_s;
					if (check_car_dist < car_behind_dist[lane]) {
						car_behind_dist[lane] = check_car_dist;
					}
				}

			}
		}

	}

}
void Vehicle::generate_predictions(double time_offset) {
	/* Generates new predictions for other vehicles to be used in trajectory 
		generation for our vehicle. */
	this->predictions.clear();

	// Assume other vehicles are moving at constant s velocity and keep its d position

	for (unsigned int i = 0; i < TRAJ_SIZE; ++i) {
		double t = time_offset + (i * T_STEP);
		double new_s = this->s + this->s_dot * t;
		// Account for over/underflow
		if (new_s > MAXIMUM_S)
			new_s -= MAXIMUM_S;
		if (new_s < 0)
			new_s += MAXIMUM_S;

		this->predictions.push_back(Vehicle(new_s,this->s_dot,0,this->d,0,0));
	}

}