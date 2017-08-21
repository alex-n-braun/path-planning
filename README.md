# path-planning
Planning a Path for an Autonomous Vehicle in a Simulator


## Table of Contents
- Intro
- Dependencies
- Contents of the Submission
- Details on the Simulation
- Preparations
    - Data Structures
    - Initial Tests
    - (Recording of) Sensor Fusion Data
    - Predictions
    - State Machine
    - Trajectories
    - External Dependencies
- Implementation of the Path Planning Algorith
    - Path Planning
- Final Remarks


## Intro
What a project! After about two weeks of full-time work I have the feeling that I can submit my results. It is clear that this project is very different from all the other projects I have done so far during this Udacity course. The main difference in my opinion is that there is very little help and guidance for the student to complete the job. The walk-through video came very late, and while it helps with the initial stumbling blocks, there is still very much to be done for the student. To do this project was only possible for me since I had my holidays at the same time coincidentally. This has to be criticized to some extent, because it has to be assumed that many of the students go to work every day -- and in parallel to a full-time job, this project is close to impossible.


## Dependencies
* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```


## Contents of the Submission
- `README.md`: this readme
- `CMakeLists.txt`: cmake configuration file
- `data/highway_map.csv`: map of the street
- `src/`
    -  External libraries: `Eigen-3.3`, `json.hpp` by [Niels Lohmann](http://nlohmann.me), `spline.h` by Tino Kluge with modifications by myself
    -  `data.h`
    -  `detection.h`, `detection.cpp`
    -  `ego.h`, `ego.cpp`
    -  `figerexercises.h`, `fingerexercises.cpp`
    -  `helpers.h`, `helpers.cpp`
    -  `highwaymap.h`, `highwaymap.cpp`
    -  `predictions.h`, `predictions.cpp`
    -  `records.h`, `records.cpp`
    -  `statemachine.h`, `statemachine.cpp`
    -  `trajectory.h`, `trajectory.cpp`
    -  `main.cpp`

There are more files in the `src` directory (`models.h` and `models.cpp`); both are not needed for running the path planner.

## Details on the Simulation
1. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases). In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

2. **The map of the highway is in data/highway_map.txt**. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

3. **Here is the data provided from the Simulator to the C++ Program**: Main car's localization Data (No Noise)
    - ["x"] The car's x position in map coordinates
    - ["y"] The car's y position in map coordinates
    - ["s"] The car's s position in frenet coordinates
    - ["d"] The car's d position in frenet coordinates
    - ["yaw"] The car's yaw angle in the map
    - ["speed"] The car's speed in MPH
    - Previous path data given to the Planner (Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time.):
        - ["previous_path_x"] The previous list of x points previously given to the simulator
        - ["previous_path_y"] The previous list of y points previously given to the simulator
    - Previous path's end s and d values
        - ["end_path_s"] The previous list's last point's frenet s value
        - ["end_path_d"] The previous list's last point's frenet d value
    - Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
        - ["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

4. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. Therefore it is not necessary to implement a controller for this project. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. **(NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.)**

5. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given.


## Preparations

### Helper Functions and Types
In `helpers.h` and `helpers.cpp` we find miscellaneous functions and types:

- Conversions between degrees and radians
- `double` versions for `abs`, `min`, `max`, ordering of two numbers (`dsort`)
- Various vector operations

### Data Structures
Communication between Simulator and Path Planner is done using uWebSockets library in JSON format. For interpreting the JSON string, the JSON for Modern C++ library by Niels Lohmann is used. 

In order to simplify things, a struct `Telemetry` and a struct `Response` is defined in `data.h`. The struct `Telemetry` contains all the information that is retreived from the simulator, while the struct `Response` contains all the information that has to be sent back to the simulator.

The class `HighwayMap` in `highwaymap.h` and `highwaymap.cpp` handles everything related to the waypoints map provided in the file `data/highway_map.csv`, including methods `distance`, `ClosestWaypoint`, `NextWaypoint`, `getFrenet` and `getXY`, originally provided as functions by Udacity. Furthermore there are the methods `getSmoothXY` and `getSmoothFrenet` which are based on spline interpolation; the latter uses an iterative solver. The **smooth** versions where introduced to overcome the kinks that can be seen in the transformations provided by `getXY` and `getFrenet`.


### Initial Tests and Finger Exercises
1. First of all, the connection between the path planner and the simulator has to be made. The code provided by Udacity does so without any change. 

2. Now, a series of waypoints is sent to the simulator in order to make the car move. As with the previous versions of the simulator, I have to set `export LANG=` in order to force the simulator to send numbers in the correct format; it seems some parser does not work correctly if I leave `LANG=de_DE.UTF-8`, as it is the default configuration on my machine.  
The car now moves with constant velocity (directly into the trees).  
You will find this test, along with other tests, as `fe_constpeed()` in `fingerexercises.h` and `fingerexercises.cpp`

3. Make the car move on a circle. See `fe_circle()` in `fingerexercises.*`.  
Of course, the car will not stay on the lane, and eventually other cars will collide.

4. Make the car follow the center of the street (ignoring other vehicles) by setting the planned path to the waypoints in the map, see `fe_waypoints()`. The car follows the center marking on the road with a speed corresponding to the distance of the waypoints divided by the 20ms time steps.

5. Make the car follow the right lane, `fe_smooth_rightmostlane` (ignoring other vehicles). We do so by using the `getXY` method as well as the `getSmoothXY` method of `HighwayMap`. The first one interpolates linearly between the waypoints, the second one uses the spline library by Tino Kluge. Note: the spline class was extended by a method `derivative` in order to compute the first derivative of the spline. If the center of the street, given by the waypoints, is used to define a spline, the `derivative` method then allows easily to compute a normal vector on any point of the center line. This in turn can be used to compute a smooth (s,d)=>(x,y) transformation.

6. There is one more finger exercise `fe_even_more_smooth_rightmostlane`. Here, we re-use the path information that is sent by the simulator. In order to extend the path to its desired length, we need to know the final s value. The value that is provided by the simulator seems not to be very precise, therefore I tried to use the `getFrenet` method provided by Udacity. Which also seems to be unprecise. Therefore I implemented a method `getSmoothFrenet` in the class `HighwayMap` which iteratively solves the problem to a precision of less than 1mm absolute error in s (not more than 5 iterations needed).

7. In order to fix the speed of the car, the curvature of the road is taken into account `fe_rightmostlane_constspeed`. For this, the spline class was extended by a second derivative.

8. In `fe_rightmostlane_constdist` the speed is scaled down to a value that corresponds to the speed of the car ahead of the ego car. For this, a simple planning (assuming constant speed of the other cars) is used to predict the distance in the future. For the prediction, see `predictions.h` and `predictions.cpp`

If you like to run the finger exercises, you simply need to uncomment the corresponsing calls in `main.cpp`, while commenting out the call to `ego.path`.

### (Recording of) Sensor Fusion Data
In `detection.h` and `detection.cpp`, the classes `Detection` and `Record` take the sensor fusion data for a single car and keep record of the history of detections. The records for the different cars are stored in the class `Records` in `records.cpp`, `records.h`. The class specifies a range in meters for keeping track of the cars within this range only.

### Predictions
Based on the sensor fusion data, the class `Records` in `records.h`, `records.cpp` does a book-keeping of the recorded data for the different cars on the road. A maximum distance in Frenet coordinates is taken into account. In `predictions.h`, `predictions.cpp`, the class `Straight` predicts the position of a car given by sensor data in a record (see above) assuming constant speed of that car. Uncertainty from the sensor data or from other sources is not considered.

### State Machine
The class `StateMachine` in `statemachine.h`, `statemachine.cpp` defines a simple class to hold the state of the ego car, which can be, in our case, `lanefollow` and `lanechange`. Additionally, the class has the property `dest_lane` which defines the destination lane if the car is in the state `lanechange`. This makes three possible states in total. The decisions for state changes are taken in the class `Ego`, see below.

### Trajectories
The files `trajectory.h` and `trajectory.cpp` contain the classes `Spline` and `MinJerk`, which serve for completely different purposes. `Spline` is used to fit a spline onto the waypoints of the map, and to fit splines to the predicted trajectories of the other cars. `MinJerk` is used to compute minimum jerk trajectories for the ego car.

### External Dependencies
The implementation makes use of `Eigen-3.3`, the `json` lib by Niels Lohmann and the `spline` lib by Tino Kluge. The latter was slightly modified by me in order to incorporate the first and the second derivative of the spline, which can be helpful for example to compute the curvature of a trajectory.

## Implementation of the Path Planning Algorithm

The `main` function mainly consists of a callback function for the communication with the simulator. You will also find there an instance of `HighwayMap highway_map`, `Records records` for book-keeping of the sensor-data, and `Ego ego`, which is the class containing path planning stuff. Inside the callback function, first of all the sensor information is read from the json stream and stored to an instance of the class `Telemetry telemetry`. With that, the book-keeping `records` is updated. Now again, the records of the sensor information are used to predict the future positions of the other cars (assumtions: constant speed, no uncertainties) using an instance of `Predictions::Predictions predictions`. 

### Path Planning

With that finally a call to `ego.path` is made, returning an instance of class `Response` that contains the trajectory of the ego car.

**Method `path`**

The basic idea now is to recursively create a tree structure with possible maneuvers (here: keep lane, change to one of the two other lanes; lane-change-preparation states are not implemented), to rate the sequence of maneuvers and to successively discard those maneuvers that a low-rated. 

First of all there is a book-keeping `vector<Point> storage` that holds elements of class `Ego::Point` which stores information about the ego car at each point on time of the path planning, as for example x,y and s,d, or the planned state of the state machine. This storage contains the planned path from the last computation step (so this is more information than what is returned  as `previous_path_x/y` from the simulator). This storage vector is reduced to a size `min_keep_steps` at the beginning of each computation step.

With a call to `ego.generate_plan` a plan is generated; here all the maneuver tree generation, rating and decision is performed. This relatively sparse plan (delta time is 0.24s for the upper-most recursion step) then is sampled to part trajectories with delta time = 0.02s, which finally are glued together.

**Method `generate_plan`**

Now the meat is in the method `generate_plan` and in the methods called from here. From the most recent `point` in the `storage` vector that was passed to `generate_plan`, it is decided in which state the state machine is at the moment (`lanefollow` or `lanechange`).

1. `lanefollow`: from this state three future paths are generated calling the method `lane_follow` for future lane following, and two calls to the method `lane_change` for changing the lane in the future, with two possible destination lanes. After this, each of the so-generated sub-plans is rated with a score that has been computed during the above method calls, **as well as the overall travelled distance that is possible with each of the sub-plans**. The best plan is returned, the others are discarded.
2. `lanechange`: If the state machine is in this state, a call to the method `lane_change` is performed.

**Method `lane_change`**

First, the car ahead of ego is determined (if there is one in range). That means in Frenet coordinates that the area between the ego in (s,d) and the range and the desired d (s_range, d_des) is scanned for other cars, using the predicted positions in predictions. For the ego we use the detected speed of the closest car and the goal speed (I choose 49.25MpH) to compute an interpolated speed that is safe to go (compare method `calc_safe_speed`). If the difference of the start value for `d` and the desired value for d, `start_d-des_d`, are below `0.5m`,the state machine switches to state `lanefollow`, else the state `lanechange` is kept. With that, a new instance of `Ego` is created in order to compute a future point in time. Here, the delta time is increased by a factor of 1.3 in order to save computation time. For this future point in time, a sub-plan in created by recursively calling the method `generate_plan`. The score of this future plan is lower weighted (by a factor of 0.5), since the future is uncertain, and added to the actual value of the score, which only consists of `-abs(start_d-des_d)/delta_t` --- a fast change in `d` value is penalized. If the recursion depth is just close future (recursion_depth<5), a car besides ego (determined by a call of the method `find_car_aside`) during lane change (you shouldn't change lane if a car is besides you) is penalized with an infinite penalty.

**Method `lane_follow`**

As before, the car ahead of ego is determined (if there is one in range), and a safe speed is computed. A future version of `Ego` is used again to recursively call `generate_plan`, but this time the state machine does not change state here (just in `generate_plan` itself). Again, the delte time is increased by a factor of `1.3`. The score of the computation step just consists of the future score, reduced by a factor of `0.5`, since the future is uncertain.

The state switching is distributed accross the methods `generate_plan` and `lane_change`. Sorry for that.

**Method `generate_successor_trajectory`**

This is one more important method. Here, the jerk-minimal trajectories are computed using the class `Trajectory::MinJerk` from `trajectory.h` and `trajectory.cpp`. The initial conditions are taken from the current point in time `Ego::Point point`, which either comes from the `storage` vector, or is created as `Ego::Point newpoint` for a future point in time during the recursion in `lane_change` or `lane_follow`. The final state is given by the desired **speed** in `s`-direction (the **position** is `s`-direction is **not** fixed) as well as the final value for `d`. The higher derivatives (acceleration and jerk in `s`-direction, speed and acceleration in `d`-direction) are fixed to zero for the final conditions. The class `Trajectory::MinJerk` uses the `Eigen` library to solve the set of equations for the jerk-minimal trajectories. 

## Final Remarks

Well, that's it. As already mentioned above, the state machine is not clearly separated in its own component, but mixed into the path generation code. This is not very nice. Also, the computation time needed is quite high on my Core i7-6700 CPU @ 3.4GHz. There are many optimization possibilities. One could play with the delta time for the recursive path planner, for example. On the other hand, the path planner is quite incomplete, for example, it does not consider the possibility to prepare for a lane change as discussed during the lectures. Also, statistical uncertainty in the prediction of the future car positions is not taken into account at all.

After some weeks of hard work, including many dead ends, I still hope that this is sufficient to pass the criteria.


