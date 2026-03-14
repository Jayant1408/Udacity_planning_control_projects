# Project: Motion Planning and Decision Making for Autonomous Vehicles

**Udacity Self-Driving Car Engineer Nanodegree – Planning Module**

## Introduction
In this project we implement several core components of the traditional `hierarchical task network`- based planner: the behaviour planner and the motion planner. Both the behaviour and the motion planner modules will be able to:
1. Navigate safely avoiding static roadside obstacles (e.g., cars, bicycles, trucks) via nudging or lane changes that are placed such that their location intersects the ego lane . 
2. Appropriately handle and stop at intersections (e.g., 3-way / 4-way intersections, round-abouts) 
3. Track the lane centre line properly.

Many behaviour / motion planning systems for autonomous vehicles utilise `finite-state machine`, such as Stanford's Junior [], the DARPA Grand Challenge Winner. In this project we also make use of the finite-state machine to handle state transitions from one active manoeuvre (i.e., `DECEL_TO_STOP`) to another. The FSM implements the behaviour planning module of the hierarchical-task network. This submodule manages the goal-state of the ego-vehicle and lookahead distance functionality (i.e., the longitudinal distance needed to travel by the ego-vehicle in order to come to a complete stop). Here we introduce comfort constraints which ensure that the deceleration for stopping remains within safe and comfortable limits

## Core Goals:
* Implement a `finite-state machine` to model behavioural planning logic.
* Perform static object `collision detection`.
* Use the cubic spiral to represent paths and trajectories for the `motion planner`
* Design and implement `cost functions` that penalise collisions / proximity to obstacles / deviations from lane centre
* Perform best-trajectory selection via cost function optimisation.

## Project Implementation Tasks
### Behaviour Planning `behavior_planner_FSM.cpp`
* (Done) Implement the look-ahead distance function;
* (Done) Compute a goal state (pose, velocity) situated behind a stopping point;
* (Done) Compute the goal speed for a nominal state;
* (Done) Track the existing goal when in `DECEL_TO_STOP` state;
* (Done) Calculate the distance travelled w.r.t. rectlinear motion;
* (Done) Compute `DECEL_TO_STOP` state w.r.t. distance rather than speed;
* (Done) Move the ego-vehicle to a `STOPPED` state;
* (Done) Track the existing goal when in `STOPPED` state;
* (Done) Move ego-vehicle to a `FOLLOW_LANE` state.

### Cost Functions `cost_functions.cpp`
* (Done) Evaluate cost w.r.t. placement of objects (as circles);
* (Done) Evaluate cost w.r.t. distance to objects (as circles);
* (Done) Evaluate cost w.r.t. distance between last waypoint on spiral and final goal state.

### Motion Planning `motion_planner.cpp`
* (Done) Plan paths based on perpendicular heading direction;
* (Done) Plan paths based on goal-offset location.

### Velocity Profile Generation `velocity_profile_generator.cpp`
* (Done) Compute the distance travelled w.r.t. rectilinear motion for the linear velocity profile;
* (Done) Compute the final velocity w.r.t. rectilinear motion for the linear velocity profile;

### Fine-Tuning Hierarchial Planner `planning_params.h`
* (Done) Define the number of paths (goal states) to enumerate;
* (Done) Define the number of waypoints to enumerate for each path spiral.


## Project Setup & Results
<!-- ### Motion Planning Decision Making For Autonomous Vehicles -->

#### Background
The vehicle behaviours we consider for this project are as follows:
* `FOLLOW_LANE`: Ego-vehicle keeps lane and maintains the configured `_speed_limit`;
* `FOLLOW_VEHICLE`: Ego-vehicle keeps lane and maintains the speed of the lead vehicle;
* `DECEL_TO_STOP`: Ego-vehicle moves towards the goal-state location (minus a distance `_stop_line_buffer`) in order to come to a complete stop using a comfortable acceleration;
* `STOPPED`: Ego-vehicle waits at e.g., intersection / stop sign for `_req_stop_time` (sec).

In the current implementation of the behaviour planner in `behavior_planner_FSM.cpp` we neglect the `FOLLOW_VEHICLE` action and instead choose to move the vehicle to a different lane to avoid any possible collisions.

#### Prerequisites
In order to execute this project, we installed the following dependencies —

Python:
* `Python 3.7`
* `Carla Simulator`
* `PyPI Pip`
* `Numpy`
* `Pygame`
* `Gtest`

C++:
* `C++14`
* `Eigen 3.3.7`
* `Carla Simulator`
* `rpclib 2.2.1`

#### Running and compiling the modules

##### Setting the hyperparameters
The motion planner has several hyperparameters whose values can be modified. Inside the `planning_params.h` file, you will find the following two parameters:

```cpp
/*** Planning Constants ***/
// Number of paths (goals) to create
// CANDO: Modify the number of path offsets to generate
#define P_NUM_PATHS 5                  // Number of deviations to enumerate
``` 

and

```cpp
// Number of waypoints to use in polynomial spiral (path)
// CANDO: Modify the number of waypoints to use in each path
// NOTE: Needs to be sufficiently large to avoid compile error
#define P_NUM_POINTS_IN_SPIRAL 25
```

which are on lines 19 and 51, respectively.

Modifying these values will allow us to control the number of and the coarseness / fineness of the polynomial spiral-based paths. 

We recommended setting the `P_NUM_POINTS_IN_SPIRAL` to a value of `25` or greater in order to avoid runtime errors (`Unknown PCM default` / `segmentation fault (core dumped)`).

##### Configuring CMAKE build
In order to use this modified version of the project code, which has renamed the `'project/starter_code/'` subfolder to `'project/planner'`, you must change line 3 of the  _original_ `run_main.sh` script to the following:

```sh
./{SUBFOLDER_NAME}/spiral_planner&
```

where `{SUBFOLDER_NAME}` should be `planner`, which matches the renamed subfolder in this repository. Note that you may ignore this step if running the `run_main.sh` script from inside this repository.

##### Creating the executable
In order to create the executable file, open a new console window and navigate to the `'/project/planner'`subdirectory. Then, execute the following commands in the console:

```console
root@foobar:/opt/web-terminal/Motion_Planning_Decision_Making_For_Autonomous_Vehicles/project/planner/#  cmake .
root@foobar:/opt/web-terminal/Motion_Planning_Decision_Making_For_Autonomous_Vehicles/project/planner/#  make
```

Note that here our project root directory is named `Motion_Planning_Decision_Making_For_Autonomous_Vehicles`, but this might be different depending on how you obtained the starter code for this project.

##### Configuring the runtime environment
If you are using the Udacity VM, i.e., the project workspace running Ubuntu 18.04.5 LTS, you will need to perform two extra steps before the executable can be run.

The first step is to run the CARLA configuration script. First, set the superuser from `root` to `student` in a separate console window with the following command:
```console
root@foobar:/opt/web-terminal/#  su - student
```

You may get a `Permission denied` error, but you can ignore this if you see the `student` user in the console command line as follows:

```console
student@foobar:  ...
```

Now, with the `student` user account configured, navigate to the `/opt/carla-simulator/` subdirectory and run the build script, i.e.,

```console
student@foobar:/#  cd /opt/carla-simulator
student@foobar:/opt/carla-simulator/#  SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl
```

This should set the CARLA Simulator to headless mode and prevent the programme from incurring any `Segmentation fault (core dumped)` errors.

## TODO This should be the first step as uWS is not there

The second step is to run the `install-ubuntu.sh` build script which installs the necessary dependencies for the Ubuntu LTS runtime environment. To do so, open a new console window, navigate to the project subfolder (here, that's `Motion_Planning_Decision_Making_For_Autonomous_Vehicles/project`), and run the following:

```console
root@foobar:/opt/web-terminal/Motion_Planning_Decision_Making_For_Autonomous_Vehicles/project/#  ./install-ubuntu.sh
```

You may have to agree to the installation of the dependencies by entering `Y` when prompted.

With these runtime configuration steps out of the way, open a _separate console window_ (making sure the CARLA build script from earlier is running in another). Then, navigate back to the project root directory:

```console
root@foobar:/opt/web-terminal/#  cd /opt/web-terminal/Motion_Planning_Decision_Making_For_Autonomous_Vehicles
```

and proceed to the programme execution steps in the following section.

##### Executing the programme
Once the project has been built successfully, the executable can be run with the following command:
```console
root@foobar:/opt/web-terminal/Motion_Planning_Decision_Making_For_Autonomous_Vehicles/project/#  ./run_main.sh
```

Depending on which environment you are running, you may experience a silent fail error. On the Udacity VM, this is expected. Simply use CTRL + C keys to halt the programme. Run the programme script again (same as above), then the CARLA Simulator should start without problems. If you continue to experience errors running the `./run_main.sh` script, make sure that you're running the script in a _new_ console window and have the CARLA headless script running under the `student` user in another console window (see _Configuring the runtime environment_). 

In the case that you get a `Unknown PCL default` error, try modifying the `P_NUM_POINTS_IN_SPIRAL` hyperparameter value (increasing this number worked for me here).

With the programme running successfully, you should observe the ego-vehicle manoeuvre automatically around obstacles, come to a complete stop at the stop sign, and even be able to make a right-turn from the complete stop into one of the available lanes.

##### More information
This build / configure / execute sequence has been adapted from the [original `how_to_run.txt` instructions]. For instructions pertaining to this repository for use with the Udacity VM, see `how_to_run.txt` file here. 

This concludes the programming tasks for this project. Refer to the "Tasks" check-list to make sure all TODOs are completed. Recommended literature to review: hierarchical planning [1], finite-state machines for AVs [2][5], real-time motion planning [6], path planning algorithms [3], and applications to vehicles [2][6][7].


#### Results

The following output was produced during a test run of the 
<img src="results/2022-12-29-Figure-1-Testing-Motion-Planner-in-CARLA.gif" width="100%" height="100%" alt="Figure 1. Testing the Motion Planner in the CARLA Simulator.">

$$
\begin{align}
\textrm{Figure 1. Testing the Motion Planner in the CARLA Simulator.}
\end{align}
$$


## 3. Closing Remarks
###### Alternatives

* Change the number of paths to generate (`P_NUM_PATHS`);
* Increase the number of points in the path (`P_NUM_POINTS_IN_SPIRAL`);

##### Extensions of task
* Consider using deep learning-based techniques for path planning / obstacle avoidance (e.g., `RL with particle filtering` )
* Implement the `FOLLOW_VEHICLE` state in `state_transition` function;
* Implement a motion controller for the `DECEL_TO_STOP` state in `state_transition` function;
* Choose a trajectory planner suitable for real-time applications (e.g., `Frenet Optimal Trajectory`);

## 4. Future Work
* Implement a motion controller for the `DECEL_TO_STOP` state in `state_transition`
