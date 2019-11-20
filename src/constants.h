#ifndef CONSTANTS_H
#define CONSTANTS_H

const double slowdown_dist = 30.0;
const double lanechange_dist = 10.0;
const double clear_dist = 150;

// Maximum S coordinate 
const double MAXIMUM_S = 6945.554;

// Amount of Original waypoints to use for Smooth waypoint generation
const double ORIG_WP_BEHIND = 3;
const double ORIG_WP_AHEAD = 5;

// Distance between Smooth waypoints
const double SMOOTH_WP_DIST = 5; // m

// Max vehicle speed
const double MAX_SPEED = 49.5 / 2.237;

// Vehicle radius
const double VEH_RADIUS = 1.25; // m

// Time Step
const double T_STEP = 0.02;

// Jerk and acceleration limits
const double MAX_JERK = 10;  // m / s / s / s
const double MAX_ACCEL = 10; // m / s / s

// Randomized target generation
const double SIGMA_S = 10; // m
const double SIGMA_D = 1; // m
const double RND_TGT_COUNT = 10; 

// Trajectory time
const double TRAJ_TIME = 1; // sec

// How many points we want to have in the path
const int PATH_SIZE = TRAJ_TIME/ T_STEP;
#endif