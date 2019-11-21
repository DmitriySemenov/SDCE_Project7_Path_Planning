#ifndef HELPERS_H
#define HELPERS_H

#define _USE_MATH_DEFINES

#include <math.h>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"
#include "constants.h"

// for convenience
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y,
												 const vector<double> &maps_s) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = maps_s[0];
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);
	// Protect for over or underflow.
	if (frenet_s > MAXIMUM_S)
		frenet_s -= MAXIMUM_S;
	if (frenet_s < 0)
		frenet_s += MAXIMUM_S;
  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {

	// Adjust maps_s coordinates, if s is less than the first waypoint's s coordinate
	// Subtract MAXIMUM_S from s coordinate until the detecting decrease
	// Decrease indicates that s overflows
	vector<double> maps_s_mod = maps_s;

	if (s < maps_s_mod[0]) {
		double prev_maps_s = 0;
		for (int i = 0; i < maps_s_mod.size(); ++i) {
			if (maps_s_mod[i] > prev_maps_s) {
				prev_maps_s = maps_s_mod[i];
				maps_s_mod[i] -= MAXIMUM_S;
			}
			else {
				// Found the point where s coordinate overflows
				break;
			}
		}
	}

	int prev_wp = -1;

  while (s > maps_s_mod[prev_wp+1]) {
    ++prev_wp;
		// protect for getting to the end of waypoints
		if (prev_wp >= (int)(maps_s_mod.size() - 1)) {
			break;
		}
		// protect for s coordinate overflowing
		if (maps_s_mod[prev_wp + 1] < maps_s_mod[prev_wp]) {
			break;
		}
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s- maps_s_mod[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

void FindClosestCars(vector<vector<double>> sensed_cars, int sensed_cars_count, double main_car_s, double max_s,
	vector<double>& car_ahead_speed, vector<double>& car_ahead_dist,
	vector<double>& car_behind_speed, vector<double>& car_behind_dist) {


	// find the closest car ahead and behind our car
	for (int i = 0; i < sensed_cars_count; ++i) {

		double d = sensed_cars[i][6];
		double vx = sensed_cars[i][3];
		double vy = sensed_cars[i][4];
		double check_car_speed = sqrt(vx * vx + vy * vy);
		double check_car_s = sensed_cars[i][5];
		double check_car_dist = max_s;

		// !!!!!!!!! PROTECT FOR MAX_S !!!!!!!!!!!!!!!!!!
		
		for (int lane = 0; lane < 3; ++lane) {
			// for every lane, find the car closest ahead and behind of us
			if (d < (2.0 + 4.0 * lane + 2.0) && d >(2.0 + 4.0 * lane - 2.0)) {
				
				// car ahead
				if (check_car_s >= main_car_s) {
					check_car_dist = check_car_s - main_car_s;
					if (check_car_dist < car_ahead_dist[lane]) {
						car_ahead_dist[lane] = check_car_dist;
						car_ahead_speed[lane] = check_car_speed;
					}
				}
				// car behind
				else {
					check_car_dist = main_car_s - check_car_s;
					if (check_car_dist < car_behind_dist[lane]) {
						car_behind_dist[lane] = check_car_dist;
						car_behind_speed[lane] = check_car_speed;
					}
				}

			}
		}

	}

}

vector<double> JMT(vector<double>& start, vector<double>& end, double T) {
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
	A << T * T * T, T* T* T* T,		T* T* T* T* T,
			 3 * T* T,	4 * T* T* T,	5 * T* T* T* T,
			 6 * T,			12 * T* T,		20 * T* T* T;

	MatrixXd B = MatrixXd(3, 1);
	B <<	end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
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

vector<double> smooth_waypoints(vector<double> frenet_point_s, vector<double> cart_point, double distance, int output_size) 
{
	// use the spline library to generate smooth waypoints
	tk::spline s;
	s.set_points(frenet_point_s, cart_point); 
	vector<double> smooth_points;
	double start_s = frenet_point_s[0];

	if (frenet_point_s.size() != cart_point.size()) {
		std::cout << "ERROR DURING SMOOTHING. SIZE MISMATCH" << std::endl;
		return { 0 };
	}

	// Start at the first s-point and increment by distance amount of meters to get the next interpolated point
	for (int i = 0; i < output_size; ++i) {
		smooth_points.push_back(s(start_s + i * distance));
	}
	return smooth_points;
}
#endif  // HELPERS_H