# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Classes in the code
* Vehicle: maintaning the vehicle parameters like speed, position, lane, etc. 
* Road: computing distance between two vehicles, determining the front and rear vehicles, etc.
* Planner: The core component for path planning.

### Logic of Path Planning.
Here are the key componets implemented for path planning.

* **Prediction**. Implemented in the function `Planner::generatePredictions`. Given the vehciles on the road, their positions at the next time step (in 0.02 seconds) are predicted. In addition to the position within their current lanes, their possible positions at the adjacent lanes are also predicted, if their `d` values indicate that they are close to the lane boundary. Therefore the function `Planner::generatePredictions` outputs a list of predicted vehicles whose number may be larger than the actual number of the vehciles. Because one vehcile could have multiple possible future positions in the next time step, depending on whether it may make a lane change.

* **Choosing the target lane and the target speed.** Implemented in the function `Planner::chooseLane`. This function evaluates the ego lane and possible ajacent lanes, and chooses the best one accoring to a evaluation score. 

* **Evaluating a lane.** Implemented in the function `Planner::evaluateLane`. For a given lane, this function evaluates the score if we move to (or stay in) this lane by prodviding a score. The larger the score, the better. The logic of computing the evaluation score is as the following: 
    
    * If the given lane is different from the ego lane, then first check the vehicle behind in the given lane. If the distance between the ego vehice and the vehcile behind is smaller than the threshold `Planner::DIST_BUFFER`, then the score for this lane is 0.

    * Check the distance between the front vehcile in the given lane and the ego vehicle. Based on this distance, we can obtain the `target_speed` if we want to move to (or stay in) this lane. 

        * If the distance is large enough, the `target_speed` can be maximum, namely `Planner::SPEED_LIMIT`
        * If the distance is not long enough for full speed, but larger than the threshold `Planner::DIST_BUFFER`, then `target_speed` is the same as the speed of the front vehicle. Namely we want to keep the same speed as the front vehicle on this given lane.
        * If the disance is smaller than `Planner::DIST_BUFFER`, which means the ego vehicle is too close to the front vehicle, then `targe_speed = min(5, front_vehicle._v)`. So in the case, we aggressively lower the speed to avoid collision.
    
    * Once we get the `target_speed` for the given lane, then the score is computed as `target_speed - 2*abs(lane_info._lane - _ego._lane)`. That is, we prefer higher target speed, but at the same time we prefer staying in the ego lane if the target speed in the given lane is not much higher than the speed in the ego lane.

* **Generating the trajectory**. Implemented in funtion `Planner::generateTrajectory`. Onece the target lane and target speed is selected, I use the spline to generate the trajectory, suggested by the course material. 

## Result and Reflections

The planner described above can work pretty well. As can be seen in the screen shot, it can run more than 35 miles without any incidents (I stopped the simulater manually then). 

![Example](screenshot.png)

However during the experiments, I did see some collisions, here are my solutions and throughs:

* **Reduce the number of trajectory points.** Parameter `Planner::NUM_TRAJECTORY_POINTS` specifies the number of points in the generated trajctory. Currently the design is to let the ego vechile finish the trajectory points, which are generated some time steps before. Therefore, if the number of trajectory is too long, something may  in the middle of the trajectory. For example a vehcile makes a lane change. This change may not be seen several time steps before when the trajectory is generated. To avoid this problem, I reduce the  `Planner::NUM_TRAJECTORY_POINTS` to 25, i.e. 0.5 seconds.

* **More accurate predictions.** Reducing the number of points in the trajectory is helpful. But I could still see collisions sometimes. Current predictions is very rough. A leaned predition model should be better. However, current code framework does not provide the derivative values with respect to and `s` and `d`, so the Navie Baysian classifiers we built in the course cannot be applied directly.

* **Local and greedy approach.** Currently the approach is local and greedy. Namely, it just sees the ajacent lanes and choose the one with the maximal target speed. However, sometimes, it may be better to first move to a slower lane and then further change to another lane that is faster than the ego lane. That requires more complicated cost functions.

## Origial README
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

