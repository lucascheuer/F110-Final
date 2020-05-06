# ESE 615 Final Project
## Team Road Runner
*A project by Arnav Dhamija, Luca Scheuer, Saumya Shah*

### Installation
Download our project using:

```
git clone --single-branch --branch lane-change https://github.com/lucascheuer/F110-Final.git
```

Our project requires the `osqp` and `osqp-eigen` packages for running our Model Predictive Control (MPC) subroutine. This can be installed by running:

```
# osqp
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
cmake --build . --target install

# osqp-eigen
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ../
make
sudo make install
```

Once these are installed, clone this repository in your catkin workspace `src/` directory. Run `catkin_make`, and then launch our node using:

```
roslaunch milestone-3 hrhc_gym.launch
```
### Project Structure

Our project is divided into the following directories:
```
* csv/ - CMA-ES trajectories, pre-computed minipaths
* inc/ - documented .hpp header files
* launch/ - launch files
* node/ - contains the node which runs our HRHC object
* src/ - .cpp files with definitions of all the functions of functions in header files
```

We have the following classes:
```
* Constraints - stores constrainsts for input, velocity, and slip angles
* Cost - specifies cost Q and R matrices
* HRHC - main class which calls functions of other classes
* Input - specifies input format (velocity, steering angle)
* Model - linearizes dynamics given a state and input
* MPC - implements Model Predictive Control using OSQP
* OccGrid - occupancy grid and helper functions for checking collisions with obstacles
* Trajectory - loads CSV for trajectories around the track
* RRT (Unused) - improved version of RRT*
* State - specifies state format (x,y,orientation)
* TrajectoryPlanner - chooses best mini-path for given trajectory
* Transforms - static functions for various tf Transforms
* Visualizer - renders visualizations
```

### Approach

#### MPC

Our final submission for the final race uses MPC for tracking a reference trajectory. This builds upon some of our work from the previous milestones such as using CMA-ES for generating an optimal cubic spline trajectory to loop around the track.

The reference trajectory is generated by ranking a set of pre-computed mini-trajectories according to how the trajectories maximize progress along the CMA-ES trajectory.

Initially, we tried to use an improved version of steering angle constrained RRT* using cubic splines to smoothen the RRT* path. However, we ran into issues with making the car follow the RRT* trajectory, as RRT* trajectories do not use dynamics for generation. It is challenging to create a good velocity vs steering angle function.

MPC uses the dynamics of the car to assign costs to states over a horizon of timesteps. By reformulating our MPC into a quadratic programming problem with constraints, we can use an off-the-shelf solver such as OSQP to solve for a minimum cost trajectory which the car can follow. The state of the car we track in this case is the x- and y-coordinates of the car and its orientation in the map frame. In our implementation, the MPC and `/drive` publisher run on different threads. This way, the publishing of `/drive` messages is always at a constant frequency, irrespective of how long the solver takes to compute a solution. Using MPC gives us robust, collision-free trajectories which can be executed at nearly 4.5m/s.

#### Lane Switching

When racing with an opponent, it's likely that our trajectory will be blocked by the opponent. This can result in all of the aforementioned mini-trajectories being infeasilbe, causing MPC to return no solution. To fix this issue, we have created multiple trajectories parallel to our original trajectory. If the ego car detects its current trajectory is blocked within a defined lookahead distance, we switch to tracking an collision-free concentric trajectory. This improves our overtaking performance.
