# Smooth Path Planning - overlaying a velocity profile
The **trajectory** is a time-series of pose data $(x, y, \theta)$ along with controls $(v, \omega)$ that define a motion profile for a differential-drive robot. [Reference](https://jwdinius.github.io/blog/2018/eta3traj/)

## Constraints
* The trajectory have fixed initial conditions.
$$ v_0 \leq v_{max}; a_0 \leq a_{max} $$
where $v_{max}$, $a_{max}$ are the maximum velocity and acceleration.
* The terminal condition:
$$ v = 0; a = 0 $$
* Kinematic limits must be obeyed everywhere along the trajectory. These limits are on velocity, acceleration, and jerk.
* The kinematic limits are symmetric:
$$ *_{min} = -*_{max} $$

## The method
### Section 1. Maximum jerk
The Robot starts at $v = v_0$ and $a = a_0$. The amount of time needed to get to maximum acceleration from the initial acceleration $a_0$ is:
$$ \Delta a = a_{max} - a_0 $$
$$ \Delta t = \frac{\Delta a}{j_{max}} $$

Let $s_{s_1}$ denote the path length traversed by the robot while applying maximum jerk:
$$ s_{s_1} = v_0 \Delta t + \frac{1}{2} a_0 {\Delta t}^2 + \frac {1}{6} j_{max}{\Delta t}^3 $$
$$ v_{s_1} = v_0 + \frac{1}{2}j_{max}{\Delta t}^2 $$

### Section 2. Maximum acceleration
The robot  continues at maximum acceleration ulti it needs to begin slowing down to hit $v_{max}$. The time-of-traversal for this second section is:
$$ v_f \equiv v'_{max} - \frac{a^2_{max}}{2j_{max}} $$
$$ \Delta v = v_f - v_{s_1} $$
$$ \Delta t = \frac{\Delta v}{a_{max}} $$

Let $s_{s_2}$ denote the path length traversed by the robot while applying maximum acceleration:
$$ s_{s_2} = v_{s_1}\Delta t + \frac{1}{2}a_{max}\Delta t^2 $$
$$ v_{s_2} = v_{s_1} + a_{max}\Delta t $$

### Section 3. Minimum jerk
In order to obey the jerk limits, the robot must apply minimum jerk to take out all of the acceleration.
$$ \Delta a = -a_{max} $$
$$ \Delta t = \frac{-a_{max}}{j_{min}} = \frac{a_{max}}{j_{max}} $$

Let $s_{s_3}$ denote the path length traversed by the robot while applying minimum jerk

$$ s_{s_3} = v_{s_2}\Delta t + \frac{1}{6}j_{min}\Delta t^3 $$
$$ v_{s_3} = v_{s_2} + \frac{1}{2}j_{min}\Delta t^2 $$

### Section 4. Cruise, initial consideration
There is no velocity change

### Section 5. Minimum jerk
At the end of the cruise section, or the end of the  first minimum jerk section if there is no cruise section, apply minimum jerk again until max deceleration is reached.
$$ \Delta a = a_{min} = -a_{max} $$
$$ \Delta t = \frac{-a_{max}}{j_{min}} = \frac{a_{max}}{j_{max}} $$

Let $s_{s_5}$ denote the path length traversed by the robot while again applying minimum jerk:
$$ s_{s_5} = v'_{max}\Delta t + \frac{1}{6}j_{min}\Delta t^3 $$
$$ v_{s_5} = v'_{max} + \frac{1}{2}j_{min}\Delta t^3 $$

### Section 6. Minimum acceleration
The robot will continue deceleratig at $a_{min}$ until the time when it needs to apply maximum jerk to hit the terminal constraint of zero velocity and acceleration.
$$ \Delta v = v_{s_5} - \frac{a^2_{min}}{2j_{max}} $$
$$ \Delta t = \frac{\Delta v}{a_{max}} $$

Let $s_{s_6}$ denote the path length traversed by the robot while applying minimum acceleration:

$$ s_{s_6} = v_{s_5}\Delta t + \frac{1}{2}a_{min}\Delta t^2 $$
$$ v_{s_6} = v_{s_5} + a_{min}\Delta t $$

### Section 7. Maximum jerk
Finally, to come to a stop at zero velocity we again apply maximum jerk
$$ \Delta a = 0 - a_{min} = a_{max} $$
$$ \Delta t = \frac{\Delta a}{j_{max}} $$

Let $s_{s_7}$ denote the path length traversed by the robot while again applying maximum jerk
$$ s_{s_7} = v_{s_6} \Delta t - \frac{1}{6} j_{max}\Delta t^3 $$
$$ v_{s_7} = v_{s_6} - \frac{1}{2}j_{max}\Delta t^2 $$

## Section 4. Cruise, final consideration
At this point, we have everything we need to determine how long of a section we will be able to cruise along at $v'_{max}$.
Upon construction of the path, we computed a total segment length, $s_{total}$, from which we will now subtract all of the computed segment lengths thus far:

$$ s_{s_4} = s_{total} - \sum_{i\neq4}s_{s_i} $$
$$ \Delta t = \frac{v'_{max}}{s_{s_4}} $$
where $\Delta t$ is the time-of-traversal.
