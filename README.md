# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
![alt text](https://github.com/SyedAzizEnam/Path-Planning/blob/master/path-plan.png)

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goal
This repo describes a path planner that is able to navigate safely around a simulated highway enviornment. This is done by passing and receiving messages from the simulator. The simulator provides us attributes about our car and other vehicles surrounding us. The path planner returns a trajectory for our car to follow.

# Path Planner
The path planner is broken into two parts, a behavior planner and a trajectory generator. The behavior planner consists of a finite state machine that changes between states. Each state assigns different values to fields that are used in the trajectory generator.

# Finite State Machine

The planner is initialized with the state "STRAIGHT" and stays that way until a car is found infront of it. Then the car transitions to "MATCH_SPEED" where it trys to match the speed of the car in front of it. From there we decide whether to go straight or check whether we can switch lanes. If there suffcient space in either lane then we switch lanes and then finally return to going straight.     

-![Finite State Machine](https://github.com/SyedAzizEnam/Path-Planning/blob/master/fsm.png)

# Trajectory Generation

The path is created by fitting a spline to 5 ways points. The waypoints consist of the last two points from the previous path and 3 points ahead spaced 30 meters apart. Once the spline is fitted we compute new points on the spline and append them to the previous path. The spacing of the points is determined by the velocity set by the behaviour planner. We append the necessary amount of points to previous path to keep a constant path size of 50 points.     

### Video

Here is video of the planner in action!

[![IMAGE ALT TEXT
HERE](https://img.youtube.com/vi/9bwMzlo6MhA/0.jpg)](https://www.youtube.com/watch?v=9bwMzlo6MhA)


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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
