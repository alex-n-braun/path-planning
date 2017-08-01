# path-planning
Planning a Path for an Autonomous Vehicle in a Simulator



## Table of Contents
- Dependencies
- Details on the Simulation
- Implementation of the Path Planning Algorith
    - Data Structures
    - Initial Tests


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


## Implementation of the Path Planning Algorithm

### Data Structures
Communication between Simulator and Path Planner is done using uWebSockets library in JSON format. For interpreting the JSON string, the JSON for Modern C++ library by Niels Lohmann is used. 

In order to simplify things, a struct `Telemetry` and a struct `Response` is defined in `data.h`. The struct `Telemetry` contains all the information that is retreived from the simulator, while the struct `Response` contains all the information that has to be sent back to the simulator.

The class `HighwayMap` in `highwaymap.h` and `highwaymap.cpp` handles everything related to the waypoints map provided in the file `data/highway_map.csv`, including methods `distance`, `ClosestWaypoint`, `NextWaypoint`, `getFrenet` and `getXY`, originally provided as functions by Udacity. They have been moved to the class in order to improve the structure of the code.


### Initial Tests and Finger Exercises
1. First of all, the connection between the path planner and the simulator has to be made. The code provided by Udacity does so without any change. 
- Now, a series of waypoints is sent to the simulator in order to make the car move. As with the previous versions of the simulator, I have to set `export LANG=` in order to force the simulator to send numbers in the correct format; it seems some parser does not work correctly if I leave `LANG=de_DE.UTF-8`, as it is the default configuration on my machine.  
The car now moves with constant velocity (directly into the trees).  
You will find this test, along with other tests, as `fe_constpeed()` in `fingerexercises.h` and `fingerexercises.cpp`
- Make the car move on a circle. See `fe_circle()` in `fingerexercises.*`.  
Of course, the car will not stay on the lane, and eventually other cars will collide.
- Make the car follow the center of the street (ignoring other vehicles) by setting the planned path to the waypoints in the map, see `fe_waypoints()`. The car follows the center marking on the road with a speed corresponding to the distance of the waypoints divided by the 20ms time steps.
- Make the car follow the right lane (ignoring other vehicles). We do so by using the `getXY` method as well as the `getSmoothXY` method of `HighwayMap`. The first one interpolates linearly between the waypoints, the second one uses the spline library by Tino Kluge. Note: the spline class was extended by a method `derivative` in order to compute the first derivative of the spline. If the center of the street, given by the waypoints, is used to define a spline, the `derivative` method then allows easily to compute a normal vector on any point of the center line. This in turn can be used to compute a smooth (s,d)=>(x,y) transformation.


