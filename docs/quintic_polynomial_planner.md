# Quintic Polynomial Planner

## Quintic Polynominals for one-dimensional robot
Assume that a one-dimensional robot motion $x(t)$ at time $t$ is formulated as a puintic polynomials based on time as follow:
$$ x(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5 $$
$$ v(t) = x'(t) = a_1 + 2a_2 t + 3a_3 t^2 + 4a_4 t^3 + 5 a_5 t^4 $$
$$ a(t) = x''(t) = 2a_2 + 6a_3 t + 12a_4 t^2 + 20 a_5 t^3 $$

where: $a_1, a_2, a_3, a_4, a_5$ are parameters of the quintic polynomial. <br>

Boundary conditions: Terminal states (start + end) are known:
* Start position: $(x_s, v_s, a_s)$ respectively
* End position: $(x_f, v_f, a_f)$ respectively

At the start time $t = t_s = 0$
$$ x(0) = a_0 = x_s $$
$$ x'(0) = a_1 = v_s $$
$$ x''(0) = 2a_2 = a_s $$
Therefore, $ a_0, a_1, a_2 $ are obtained.

At the end time $t = t_f$
$$ x(t_f) = a_0 + a_1 t_f + a_2 t_f^2 + a_3 t_f^3 + a_4 t_f^4 + a_5 t_f^5 = x_f$$
$$ x'(t_f) = a_1 + 2a_2 t_f + 3a_3 t_f^2 + 4a_4 t_f^3 + 5 a_5 t_f^4 = v_f $$
$$ x''(t) = 2a_2 + 6a_3 t_f + 12a_4 t_f^2 + 20 a_5 t_f^3 = a_f $$

From 3 equation above, we have the linear equation $ \mathbf{A}\mathbf{x} = \mathbf{b}$:
$$ \left[\begin{array}{ccc} t_f^3 & t_f^4 &t_f^5 \\ 3t_f^2 & 4t_f^3 & 5t_f^4 \\ 6t_f & 12t_f^2 & 20t_t^3 \end{array}\right] \left[\begin{array}{c} a_3 \\ a_4 \\ a_5\end{array}\right] = \left[\begin{array}{c} x_f - x_s - v_st_s - 0.5a_st_f^2 \\ v_f - v_s - a_st_f \\ a_f - a_s \end{array}\right]$$
Solve the equations, we obtain all parameter.

## Quintic polynomials for two dimensional robot motion (Oxy)

Two dimensional robot motion in Oxy plane.
$$ x(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5 $$
$$ y(t) = b_0 + b_1 t + b_2 t^2 + b_3 t^3 + b_4 t^4 + b_5 t^5 $$

Boundary conditions: Terminal state(start + end) are known:
* Start position: $ (x_s, y_s, \theta_s, v_s, a_s) $ respectively
* End position: $ (x_f, y_f, \theta_f, v_f, a_f) $ respectively

Each velocity and acceleration boundary condition can be calculated with each orientation.

$$ v_{xs} = v_s\cos\theta_s ;v_{ys} = v_s \sin\theta_s $$
$$ v_{xf} = v_f\cos\theta_f ;v_{yf} = v_f \sin\theta_f $$