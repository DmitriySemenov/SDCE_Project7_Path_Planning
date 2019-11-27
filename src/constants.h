#ifndef CONSTANTS_H
#define CONSTANTS_H

// Maximum S coordinate 
const double MAXIMUM_S = 6945.554;

// Amount of Original waypoints to use for Smooth waypoint generation
const int ORIG_WP_BEHIND = 3;
const int ORIG_WP_AHEAD = 5;

// Distance between Smooth waypoints
const double SMOOTH_WP_DIST = 1; // m

// Max vehicle speed target
const double MAX_SPEED = 46.5 / 2.237;

// Vehicle radius
const double VEH_RADIUS = 3; // m

// Lane change allowed distance to other cars
const double LANECHG_DIST = VEH_RADIUS * 2;

// Distance to slow down when getting too close
const double TOOCLOSE_DIST = VEH_RADIUS * 3;

// Time Step
const double T_STEP = 0.02;

// Jerk and acceleration limits
//const double MAX_ACCEL = 9; // m / s / s

// Randomized target generation
const int S_NEG_OFF = -10;
const int S_POS_OFF = 2;
const int S_INCR = 1;

// Path Time
const double PATH_TIME = 1; // sec

// How many points we want to have in the path
const unsigned int PATH_SIZE = PATH_TIME / T_STEP;

// How many previous path points we want to keep
const unsigned int PREV_PATH_MAX = 25;

// Trajectory time (should be at least the same time as path time in order to generate points to fill the path)
const double TRAJ_TIME = 1; // sec

// How many points we want to have in the trajectory
const unsigned int TRAJ_SIZE = TRAJ_TIME / T_STEP;

const double ACCEL_LIMIT = 7; // m/s
const double DECEL_LIMIT = 8;

const double COLLISION_W	= 10000.0;
const double BUFF_W				= 500.0;
const double NOTMID_W			= 30.0;
const double TOOCLOSE_W		= 5000.0;
const double SPEEDLIM_W		= 3000.0;
const double HIGHSPD_W		= 2000.0;
const double MAXACCEL_W		= 3000.0;
const double NOTFASTEST_W = 1000.0;
const double DIFFD_W			= 50.0;

const double BADRIGHTLANE_S_MIN = 4850;
const double BADRIGHTLANE_S_MAX = 5150;
#endif