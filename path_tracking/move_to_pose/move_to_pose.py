import matplotlib.pyplot as plt
import numpy as np
from random import random

# Simulation parameters
Kp_rho = 9
Kp_alpha = 15
Kp_beta = -3
dt = 0.01

show_animation = True

def move_to_pose(sx, sy, sq, gx, gy, gq):
    """
    rho is the distance between the robot and the goal position
    alpha is the angle to the goal relative to the heading of the robot
    beta is the angle between the robot's position and the goal position plus the goal angle
    Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
    Kp_beta*beta rotates the line so that it is parallel to the goal angle
    """
    x = sx; y = sy; q = sq
    x_diff = gx - x; y_diff = gy - y
    x_traj, y_traj = [], []

    rho = np.hypot(x_diff, y_diff)
    while rho > 0.001:
        x_traj.append(x); y_traj.append(y)
        x_diff = gx - x; y_diff = gy - y

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff) - q + np.pi)%(2*np.pi) - np.pi
        beta = (gq - q - alpha + np.pi) % (2 * np.pi) - np.pi

        v = Kp_rho*rho
        w = Kp_alpha*alpha + Kp_beta*beta

        if alpha > np.pi /2 or alpha < -np.pi/2:
            v = -v
        
        q += w*dt
        x += v*np.cos(q)*dt
        y += v*np.sin(q)*dt

        if show_animation:
            plt.cla()
            plt.arrow(sx, sy, np.cos(sq), np.sin(sq), color='r', width=0.1)
            plt.arrow(gx, gy, np.cos(gq), np.sin(gq), color='g', width=0.1)
            plot_vehicle(x, y, q, x_traj, y_traj)

def plot_vehicle(x, y, q, x_traj, y_traj):
    # Corners of triangular vehicle
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, q)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot(x_traj, y_traj, 'b--')

    # ESC to stop
    plt.gcf().canvas.mpl_connect('key_release_event',
                                lambda event: [exit(0) if event.key == 'escape' else None])
    plt.xlim(0, 20)
    plt.ylim(0, 20)
    plt.pause(dt)

def transformation_matrix(x, y, q):
    return np.array([   [np.cos(q), -np.sin(q), x],
                        [np.sin(q), np.cos(q), y],
                        [0, 0, 1]])

if __name__ == '__main__':
    x_start = 20 * random()
    y_start = 20 * random()
    theta_start = 2 * np.pi * random() - np.pi
    for i in range(5):
        x_goal = 20 * random()
        y_goal = 20 * random()
        theta_goal = 2 * np.pi * random() - np.pi
        print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
              (x_start, y_start, theta_start))
        print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
              (x_goal, y_goal, theta_goal))
        move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)
        x_start = x_goal
        y_start = y_goal
        theta_start = theta_goal