# path-planning
Planning a Path for an Autonomous Vehicle in a Simulator



## Table of Contents
- Dependencies
- Contents of the Submission
- Details on the Simulation
- Preparations
    - Data Structures
    - Initial Tests
    - Vehicle Models
    - (Recording of) Sensor Fusion Data
- Implementation of the Path Planning Algorith
    - Prediction

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


## Details on the Simulation
1. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases). In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

* **The map of the highway is in data/highway_map.txt**. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

* **Here is the data provided from the Simulator to the C++ Program**: Main car's localization Data (No Noise)
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

* The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. Therefore it is not necessary to implement a controller for this project. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. **(NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.)**

* There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given.


## Preparations

### Data Structures
Communication between Simulator and Path Planner is done using uWebSockets library in JSON format. For interpreting the JSON string, the JSON for Modern C++ library by Niels Lohmann is used. 

In order to simplify things, a struct `Telemetry` and a struct `Response` is defined in `data.h`. The struct `Telemetry` contains all the information that is retreived from the simulator, while the struct `Response` contains all the information that has to be sent back to the simulator.

The class `HighwayMap` in `highwaymap.h` and `highwaymap.cpp` handles everything related to the waypoints map provided in the file `data/highway_map.csv`, including methods `distance`, `ClosestWaypoint`, `NextWaypoint`, `getFrenet` and `getXY`, originally provided as functions by Udacity. They have been moved to the class in order to improve the structure of the code. Furthermore there are the methods `getSmoothXY` and `getSmoothFrenet` which are based on spline interpolation; the latter uses an iterative solver.


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


### Vehicle Models
For now, there are two vehicle models implemented in the namespace `Models` in `models.h` and `models.cpp`:

1. Class `MovingPoint`: a model with a state space (x,y,vx,vy)

2. Class `Bicycle`: a model with a state space (x, y, theta, v), where theta is the heading of the vehicle. 

Both classes share a common structure, in fact it should be easily possible to reimplement the two as specialisations of a generic template with a very high reuse of the code. However, at this point I shift this to the future ;-).

The common structure looks as follows: the model class defines four subclasses, `Properties`, `State`, `Actuation` and `Update`.

1. `Properties`: properties of an instance of this model. In case of a bike, this would be the distance between the axes. A point particle however has no properties in this sense, hence the subclass `Properties` of `MovingPoint` is empty.

2. `State`: the state of a vehicle is defined by its position and velocity. In case of a point particle, this would be x, y, vx, vy. In case of a bike, this would be x, y, heading theta and speed. In both cases, the state is stored in a 4-element vector.

3. `Actuation`: a set of values that define an action that can be performed on the vehicle. For a point particle, this is an acceleration in x,y direction, given by ax,ay. For a bike, this is an acceleration a and a steering angle delta.

4. `Update`: this class computes an update for the state of the vehicle, given the properties of the vehicle, the actual state, an actuation, and a delta time. For a point particle, as mentioned above, there are no properties. The update for (x,y,vx,vy) is simply dt\*(vx,vy,ax,ay). We neglect the contribution of the acceleration on the positional update, which would be 0.5\*ax\*dt^2 and 0.5\*ay\*dt^2, assuming that this is a reasonable approximation.
For a bike, as mentioned before, the distance of the axes is the property. The update for (x, y, theta, v) is given by dt\*(v cos(theta), v sin(theta), v/L tan(delta), a).

Both models contain a method to set its state, `set_state`, as well as three methods to evolve the state in time: `advance`, `advance_state` and `advanced_vehicle`. The three latter methods all perform the same update action, where

1. `advance` returns a state object containing the advanced state,

2. `advance_state` does not return anything, instead it updates the state of the model,

3. `advanced_vehicle` generates a new instance of the vehicle (in the updated state), without altering the state of the given instance.

These methods are defined for convenience; I will see which one is best suited for my needs.


### (Recording of) Sensor Fusion Data




## Implementation of the Path Planning Algorithm

### Prediction

