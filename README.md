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
    * Cubic Spline (path generator from point)
2. [Path Tracking](path_tracking)
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

#### Cubic Spline path generation
![Cubic Spline](results/path_planning/spline.png)

### Path Tracking
#### The bicycle model
The robot kinematic model was used in this package is Car-like robot model.
#### Move to Pose
![Move to Pose](results/path_tracking/move2pose.png)
#### Rear Wheel Feedback
![Rear wheel](results/path_tracking/rear_wheel.png)
#### Stanley Control
![Stanley Control](results/path_tracking/stanley.png)