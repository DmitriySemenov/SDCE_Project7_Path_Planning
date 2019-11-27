# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates
-
["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Requirements Overview
- The car is able to drive at least 4.32 miles without incident.
- The car drives according to the speed limit.
- Max Acceleration and Jerk are not Exceeded.
- The car must not come into contact with any of the other cars on the road.
- The car stays in its lane, except for the time between changing lanes.
- The car is able to change lanes.

## Implementation Overview

To complete this project the following approach has been chosen:

Step 0: Generate smooth waypoints near our car. Use spline library.

Step 1: Figure out time offset (to predict other car positions) based on prev path size
	time_offset = prev_path_size * T_STEP
      Initialize our car using end point of prev path converted from (x, y) to (s, d).
      Initialize other cars using sensor fusion data and generate predicitons of positions, starting at time_offset.
      
Step 2: Based on our_veh position in (s, d) and other_veh predictions, figure out available operating_state.
         Possible states: Keep Lane(KL), Lane Change Left(LCL), Lane Change Right(LCR)
         Can't do LCL in left most lane (0) and can't do LCR in right most lane(2). 
         Can't make a lane change, if there are cars nearby.
	 
Step 3: For each available operating_state generate target positions in (s, d) projected into the future with T = TRAJ_TIME sec.
      For KL : d = middle of car_lane,
      LCL : d = middle of the lane to the left of car_lane
      LCR : d = middle of the lane to the right of car_lane
      For all states : s = project TRAJ_TIME second(s) ahead using current speed and accel, but considering max speed limit.
      Using these KL, LCL, LCR targets (s_tgt,d_tgt), for each available state
      generate a few other targets up to S_NEG_OFF meters behind and S_POS_OFF meters ahead, with a S_INCR meters increment step.
      
Step 4: For each (s_t, d_t) generate JMT coefficients, assuming TRAJ_TIME second(s) time for reaching target. 

Step 5: For each target, generate trajectory using JMT coefficients and calculate total cost using multiple cost functions.
         Find the lowest cost trajectory and use it as target trajectory.
	 
Step 6: Add on new targets to the prev_path existing ones, until reaching PATH_SIZE number of target points. 
         Use conversion function to convert from (s,d) to (x,y) and smooth out waypoints.
         
## Implementation Details
First step (or actually Step 0) is to generate smooth waypoints near our car using the spline library.
This helps the car avoid jerking and drive smoothly.
The smooth_waypoints() function that does the interpolation can be found in the helpers.h 

In Step 1, our_veh object (representing the ego vehicle) of class Vehicle is updated with the latest predictions for position, velocity, and acceleration in frenet coordinate system. These predictions start from the point at the end of the previous path, if enough points are available to recreate the values. 

At the same time, vector of Vehicle objects representing all the other vehicles detected by sensor fusion is created and initialized.
For each other vehicle, trajectory predictions are made using generate_predictions() function, starting from the time offset:
      time_offset = prev_path_size * T_STEP
This is done because ego vehicle's trajectory will be generated starting from time_offset as well.

In Step 2, available operating states for ego vehicle are updated.
Possible states are: Keep Lane (KL), Lane Change Left (LCL), and Lane Change Right (LCR).

KL state is always available. The other two states depend on whether there is a lane to change to and if there is a nearby car in that lane. This is handled by Vehicle's class upd_available_states() function.

In Step 3, for each available operating state, target positions in (s, d) are generated. They are projected TRAJ_TIME second(s) into the future. Here are how the targets are generated using gen_targets() function:
   For KL : d = middle of car_lane
      LCL : d = middle of the lane to the left of car_lane
      LCR : d = middle of the lane to the right of car_lane
   For all states : s = projected TRAJ_TIME second(s) ahead using current speed and acceleration, considering max speed limit.
This gives a maximum of 3 target pairs (s_tgt, d_tgt).

Using these targets (s_tgt,d_tgt), for each available state, a few other targets up to S_NEG_OFF meters behind and S_POS_OFF meters ahead are generated, with a S_INCR meters increment step. This is done by perturb_target() function. This is done to allow the vehicle a chance to slow down or speed up, depending which target has the lowest cost function value.

In Step 4, for each target pair (s, d), generate_coeffs_for_targets() function is used to create Jerk Minimizing Trajectory coefficients for s and d dimension. It's assumed that vehicle starts with the state predicted at the end of previous path and needs to reach target_s and target_d in TRAJ_TIME second(s).

Step 5 is a key step where best trajectory and corresponding set of target_s and target_d is generated.
For each target pair, a trajectory is generated using JMT coefficients and a generate_traj_for_target() function.
Next, a number of cost functions are calculated for each trajectory. These functions include:

- collision_cost(): Uses nearest_approach_to_vehicle_in_lane() function to calculate the closest ego vehicle gets to other vehicles. If the nearest vehicle is less than (2 * VEH_RADIUS) meters away, a cost penalty is applied.

- buffer_cost(): Uses nearest_s_dist_to_vehicle_in_lane() function to generate a cost based on how far ahead other vehicles in the lane are in relation to the ego vehicle. This penalizes trajectories which have vehicles closer to ego vehicle. This is a non-binary cost.

- not_middle_lane_cost(): This cost function applies a slight penalty to trajectories that don't end up in the middle lane. The idea is that it is easier to make lane changes from the middle lane, since both left and right lane are potentially available.

- too_close_cost(): This cost function applies a penalty that kicks in when ego vehicle gets within 6 * VEH_RADIUS meters of the vehicle ahead during the trajectory. The cost increases as the distance shortens.

- exceeds_speed_limit_cost(): This cost function penalizes trajectories that exceed the maximum speed limit. 

- high_spd_cost(): This function increases cost for trajectories the farther away they are from the maximum speed limit. This helps pick trajectory that will result in vehicle going the fastest.

- exceeds_accel_limit_cost(): Penalizes trajectories that exceed acceleration limits.

- not_fastest_lane_cost(): Penalizes trajectories that are in the lane that does not have the fastest speed (based on the speed of the vehicle ahead).

- diffd_cost(): Penalizes changes in target_d, compared to previous target_d. This helps minimize target lane (corresponding to target_d) from switching too often.

Each cost is weighted differently. Here's the order of costs, based on the highest weight:

   1) collision_cost
   2) too_close_cost
   3) exceeds_speed_limit_cost
   4) exceeds_accel_limit_cost
   5) high_spd_cost
   6) not_fastest_lane_cost
   7) buffer_cost
   8) diffd_cost
   9) not_middle_lane_cost

All costs for each trajectory are added up and the lowest cost trajectory is found. It is then used as a target trajectory with corresponding target_s and target_d values.

In final Step 6, target x and y values are generated. Initial idea was to use JMT trajectory generated s and d values and convert them to x and y. However, it turned out that they were not always smooth enough to stay within the acceleration and speed limits, so only final target_s and target_d are used. All the previous steps are still valid, just the way the trajectory is generated differs.

First, rough targets are generated using previous path's two final positions, ego vehicle's s-position + 30, and ego vehicle's s-position + 60 meters. Target d-positions for the last two rough targets are target_d from the trajectory.
Then, these s and d positions get converted to X and Y targets, using getXY() function.

Target vehicle speed is determined using the difference between target_s and current_s position.
Then, target_s position for each time step is generated for smooth trajectory using this target velocity and acceleration.
Acceleration to reach target speed is limited by ACCEL_LIMIT and DECEL_LIMIT, depending if the vehicle is speeding up or slowing down.

Then, spline interpolation is used to generate smooth x and y positions using rough s, x, and y positions and target_s position for each time step.

The new path target points are added to the previous path and the final trajectory is sent back to the simulator.

## Additional Notes

Simulator sometimes incorrectly detects vehicle being out of lane when vehicle is in the right most lane. This happens between s-coordinate 4850 and s-coordinate 5150 (approximately). There is a special piece of code that checks for that and adjusts d-target slightly from 10 (middle of right lane) to 9.4, to avoid this issue.

Throughout the implementation care is taken to account for overflow of s-coordinate. This is especially important when calculating distance to the vehicle up ahead, calculating velocity, or generating a new target_s position.

There are debug outputs that are saved in a separate csv file that are available, if that functionality is enabled. Corresponding defines are located in the debug.h header file.

Target maximum velocity is set slightly lower than 50 MPH, because the actual vehicle speed fluctuates slightly around the target. This is mostly due to inaccuracy of converting between frenet and cartesian coordinates.
