#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include "constants.h"
//#include "cost.h"

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle() {}

Vehicle::Vehicle(double s, double s_dot, double s_doubledot, double d, double d_dot, double d_doubledot) {
	this->s = s;
	this->s_dot = s_dot;
	this->s_doubledot = s_doubledot;
	this->d = d;
	this->d_dot = d_dot;
	this->d_doubledot = d_doubledot;
	this->state = "KL";
}

Vehicle::~Vehicle() {}


void Vehicle::generate_predictions(double time_offset) {
	// Generates new predictions for other vehicles to be used in trajectory 
	// generation for our vehicle.
	this->predictions.clear();

	// Assume other vehicles are moving at constant s velocity and keep its d position

	for (int i = 0; i < TRAJ_SIZE; ++i) {
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