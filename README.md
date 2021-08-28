# Mobile Robot using Python
## Environment
Python 3<br>
```
pip install -r requirements.txt
```
## Table of Contents
1. [Path Planning](path_planning)
    * A Star
    * Dijkstra
    * Probabilistic Road Map
    * Rapidly-exploring random tree
    * Cubic Spline (path generator from point)
2. [Path Tracking](path_tracking)
    * Linear MPC
    * LQR Steering / Speed Steering
    * Move to Pose
    * Rear Wheel Feedback
    * Stanley Control

## Results
### Path Planning
#### A Star
![A Star](results/path_planning/a_star.png)

#### Dijkstra
![Dijkstra](results/path_planning/dijkstra.png)

#### Probabilistic Road Map
![PRM](results/path_planning/prm.png)

#### Rapidly-exploring Random Tree
![RRT](results/path_planning/rrt.png)

#### Cubic Spline path generation
![Cubic Spline](results/path_planning/spline.png)

### Path Tracking
#### The Car-like Mobile Robot model
The robot kinematic model was used in this package is Car-like robot model. For more information, please consider the [Car-like Mobile Robot](docs/car-like_robot_model.md)
#### Linear MPC
![Linear MOC](results/path_tracking/linear_mpc.png)

#### LQR Steering
![LQR Steering](results/path_tracking/lqr_steering.png)
#### LQR Speed Steering
![LQR Speed Steering](results/path_tracking/lqr_speed_steer.png)
#### Move to Pose
![Move to Pose](results/path_tracking/move2pose.png)
#### Move to Pose with Obstacle avoiding using Dynamic Window Approach
![Dynamic Window Approach](results/path_tracking/dwa.png)
#### Pure Pursuit
![Pure Pursuit](results/path_tracking/pure_pursuit.png)
#### Rear Wheel Feedback
![Rear wheel](results/path_tracking/rear_wheel.png)
#### Stanley Control
![Stanley Control](results/path_tracking/stanley.png)