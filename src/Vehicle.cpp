#include "vehicle.h"
#include "constants.h"
#include <random>
#include <algorithm>
#include <iterator>
#include <chrono>
//#include "cost.h"

using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
	/* Update vehicle's current lane */
	
	if (d_curr < 4)
		lane = 0;
	else if (d_curr >= 4 && d_curr < 8)
		lane = 1;
	else
		lane = 2;
}

void Vehicle::upd_available_states(vector<Vehicle> &other_vehicles) {
	/* Check for available states (KL, LCL, LCR), based on current lane of the vehicle
		and predicted position of other vehicles */
	
	// Reset available states
	available_states.clear();

	// Keep Lane is always available
	available_states.push_back("KL");

	// Update current vehicle lane
	upd_lane(this->d);

	// Update other vehicle lane and find closest vehicle in each lane
	for (unsigned int i = 0; i < other_vehicles.size(); ++i) {
		other_vehicles[i].upd_lane(other_vehicles[i].predictions[0].d);
	}

	// Find distance to closest other vehicle in each lanes
	double car_ahead_dist[3] = { MAXIMUM_S, MAXIMUM_S, MAXIMUM_S };
	double car_behind_dist[3] = { MAXIMUM_S, MAXIMUM_S, MAXIMUM_S };

	upd_closest_veh(other_vehicles, car_ahead_dist, car_behind_dist);

	// Check if the vehicles in adjacent lanes are far enough to allow lane change
	// Vehicle is in the middle lane
	if (lane == 1) {
		// Check lane change to the left
		if ((car_ahead_dist[0] > LANECHG_DIST) && (car_behind_dist[0] > LANECHG_DIST))
			available_states.push_back("LCL");

		// Check lane change to the right
		if ((car_ahead_dist[2] > LANECHG_DIST) && (car_behind_dist[2] > LANECHG_DIST))
			available_states.push_back("LCR");
	}
	// Vehicle is in the left lane
	else if (lane == 0) {
		// Check lane change to the right
		if ((car_ahead_dist[1] > LANECHG_DIST) && (car_behind_dist[1] > LANECHG_DIST))
			available_states.push_back("LCR");
	}
	// Vehicle is in the right lane
	else {
		// Check lane change to the left
		if ((car_ahead_dist[1] > LANECHG_DIST) && (car_behind_dist[1] > LANECHG_DIST))
			available_states.push_back("LCL");
	}
}

void Vehicle::upd_closest_veh(const vector<Vehicle>& other_vehs, double (&car_ahead_dist)[3], double (&car_behind_dist)[3]) {

	/* Find the closest car ahead and behind our car at start of prediction time
		since our car position is at start of predicted position */

	for (unsigned int i = 0; i < other_vehs.size(); ++i) {

		double d = other_vehs[i].predictions[0].d;
		double check_car_speed = other_vehs[i].predictions[0].s_dot;
		double check_car_s = other_vehs[i].predictions[0].s;
		double check_car_dist = MAXIMUM_S;
		double main_car_s = this->s;

		for (unsigned int l = 0; l < 3; ++l) {
			// for every lane, find the car closest ahead and behind us
			if (d < (2.0 + 4.0 * l + 2.0) && d >(2.0 + 4.0 * l - 2.0)) {

				// car ahead
				if (check_car_s >= main_car_s) {
					check_car_dist = check_car_s - main_car_s;
					if (check_car_dist < car_ahead_dist[l]) {
						car_ahead_dist[l] = check_car_dist;
					}
				}
				// car behind
				else {
					check_car_dist = main_car_s - check_car_s;
					if (check_car_dist < car_behind_dist[l]) {
						car_behind_dist[l] = check_car_dist;
					}
				}

			}
		}

	}

}

void Vehicle::gen_targets() {
	/* For each available operating_state generate target positions in (s, d) projected into the future with T = TRAJ_TIME sec.
		For KL : d = middle of car_lane,
		LCL : d = middle of the lane to the left of car_lane
		LCR : d = middle of the lane to the right of car_lane
		For all states : s = project position TRAJ_TIME sec. ahead using current speed and accel, but considering max speed limit.

		Using these KL, LCL, LCR targets (s_tgt,d_tgt), for each available state
		generate RND_TGT_COUNT(10) targets using gaussian distribution and sigma defined in costants.h */

	// Reset targets
	target_s.clear();
	target_d.clear();

	double s_t, d_t;

	for (int i = 0; i < available_states.size(); ++i) {

		if (available_states[i] == "KL") {
			d_t = 2.0 + lane * 4.0;

			double t = 0;
			double new_s_dot = s_dot;
			double new_s = s;

			for (unsigned int i = 0; i < TRAJ_SIZE; ++i) {
				t = (i+1) * T_STEP;
				new_s_dot = std::min(s_dot + s_doubledot * t, MAX_SPEED);
				new_s += new_s_dot * T_STEP;

				// Account for over/underflow
				if (new_s > MAXIMUM_S)
					new_s -= MAXIMUM_S;
				if (new_s < 0)
					new_s += MAXIMUM_S;
			}
			s_t = new_s;

			perturb_target(s_t, d_t, SIGMA_S, SIGMA_D_KL);
		}
		else if (available_states[i] == "LCL") {
			d_t = 2.0 + (lane - 1) * 4.0;

			double t = 0;
			double new_s_dot = s_dot;
			double new_s = s;

			for (unsigned int i = 0; i < TRAJ_SIZE; ++i) {
				t = (i + 1) * T_STEP;
				new_s_dot = std::min(s_dot + s_doubledot * t, MAX_SPEED);
				new_s += new_s_dot * T_STEP;

				// Account for over/underflow
				if (new_s > MAXIMUM_S)
					new_s -= MAXIMUM_S;
				if (new_s < 0)
					new_s += MAXIMUM_S;
			}
			s_t = new_s;

			perturb_target(s_t, d_t, SIGMA_S, SIGMA_D);
		}
		else {
			d_t = 2.0 + (lane + 1) * 4.0;

			double t = 0;
			double new_s_dot = s_dot;
			double new_s = s;

			for (unsigned int i = 0; i < TRAJ_SIZE; ++i) {
				t = (i + 1) * T_STEP;
				new_s_dot = std::min(s_dot + s_doubledot * t, MAX_SPEED);
				new_s += new_s_dot * T_STEP;

				// Account for over/underflow
				if (new_s > MAXIMUM_S)
					new_s -= MAXIMUM_S;
				if (new_s < 0)
					new_s += MAXIMUM_S;
			}
			s_t = new_s;

			perturb_target(s_t, d_t, SIGMA_S, SIGMA_D);
		}
	}
	
}

void Vehicle::perturb_target(double mu_s, double mu_d, double sig_s, double sig_d) {
	// Generate RND_TGT_COUNT number of target s and d coordinates
	std::default_random_engine generator;
	generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
	std::normal_distribution<double> s_dist(mu_s, sig_s);
	std::normal_distribution<double> d_dist(mu_d, sig_d);

	for (int i = 0; i < RND_TGT_COUNT; ++i) {
		double s_tgt = s_dist(generator);
		double d_tgt = d_dist(generator);

		// Account for over/underflow
		if (s_tgt > MAXIMUM_S)
			s_tgt -= MAXIMUM_S;
		if (s_tgt < 0)
			s_tgt += MAXIMUM_S;

		target_s.push_back(s_tgt);
		target_d.push_back(d_tgt);
	}
	
}
void Vehicle::generate_predictions(double time_offset) {
	/* Generates new predictions for other vehicles to be used in trajectory 
		generation for our vehicle. */

	predictions.clear();

	// Assume other vehicles are moving at constant s velocity and keep its d position

	for (unsigned int i = 0; i < TRAJ_SIZE; ++i) {
		double t = time_offset + (i * T_STEP);
		double new_s = s + s_dot * t;
		// Account for over/underflow
		if (new_s > MAXIMUM_S)
			new_s -= MAXIMUM_S;
		if (new_s < 0)
			new_s += MAXIMUM_S;

		predictions.push_back(Vehicle(new_s,s_dot,0,d,0,0));
	}

}

void Vehicle::generate_coeffs_for_targets() {
	/* For each pair of s and d targets, generate JMT coefficients*/
	s_traj_coeffs.clear();
	d_traj_coeffs.clear();

	for (int i = 0; i < target_s.size(); ++i) {
		
		double tgt_s = target_s[i];
		double tgt_d = target_d[i];
		//If target wraps around max, use target+MAX for coefficient generation
		if (tgt_s < s)
			tgt_s += MAXIMUM_S;

		double tgt_s_dot = (target_s[i] - s) / TRAJ_TIME;

		vector<double> start_s = { s, s_dot, s_doubledot };
		vector<double> end_s = { tgt_s, tgt_s_dot, 0 };
		vector<double> start_d = { d, d_dot, d_doubledot };
		vector<double> end_d = { tgt_d, 0, 0 };

		s_traj_coeffs.push_back(JMT(start_s, end_s, TRAJ_TIME));
		d_traj_coeffs.push_back(JMT(start_d, end_d, TRAJ_TIME));
	}
}

vector<vector<double>> Vehicle::generate_traj_for_target(int target_index) {
	/* Generates vector of trajectory coordinate pairs (s,d) for a given trajectory coefficients*/
	vector<double> s_coeff = s_traj_coeffs[target_index];
	vector<double> d_coeff = d_traj_coeffs[target_index];
	vector<vector<double>> trajectory;

	for (unsigned int i = 0; i < TRAJ_SIZE; ++i) {
		vector<double> s_d_curr;
		double t = i * T_STEP;
		double s_curr, d_curr;
		// generate s and d using polynomial coefficients
		// s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
		s_curr = s_coeff[0] + s_coeff[1] * t + s_coeff[2] * pow(t, 2) + s_coeff[3] * pow(t, 3) + s_coeff[4] * pow(t, 4) + s_coeff[5] * pow(t, 5);
		d_curr = d_coeff[0] + d_coeff[1] * t + d_coeff[2] * pow(t, 2) + d_coeff[3] * pow(t, 3) + d_coeff[4] * pow(t, 4) + d_coeff[5] * pow(t, 5);
		
		// Limits check
		if (s_curr > MAXIMUM_S)
			s_curr -= MAXIMUM_S;

		s_d_curr.push_back(s_curr);
		s_d_curr.push_back(d_curr);
		trajectory.push_back(s_d_curr);
	}
	return trajectory;
}
vector<double> Vehicle::JMT(vector<double>& start, vector<double>& end, double T) {
	/**
	 * Calculate the Jerk Minimizing Trajectory that connects the initial state
	 * to the final state in time T.
	 *
	 * @param start - the vehicles start location given as a length three array
	 *   corresponding to initial values of [s, s_dot, s_double_dot] or [d, d_dot, d_double_dot]
	 * @param end - the desired end state for vehicle. Like "start" this is a
	 *   length three array.
	 * @param T - The duration, in seconds, over which this maneuver should occur.
	 *
	 * @output an array of length 6, each value corresponding to a coefficent in
	 *   the polynomial:
	 *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
	 *
	 * EXAMPLE
	 *   > JMT([0, 10, 0], [10, 10, 0], 1)
	 *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
	 */
	MatrixXd A = MatrixXd(3, 3);
	A << T * T * T, T* T* T* T, T* T* T* T* T,
		3 * T* T, 4 * T* T* T, 5 * T* T* T* T,
		6 * T, 12 * T* T, 20 * T* T* T;

	MatrixXd B = MatrixXd(3, 1);
	B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
		end[1] - (start[1] + start[2] * T),
		end[2] - start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai * B;

	vector <double> result = { start[0], start[1], .5 * start[2] };

	for (int i = 0; i < C.size(); ++i) {
		result.push_back(C.data()[i]);
	}

	return result;
}