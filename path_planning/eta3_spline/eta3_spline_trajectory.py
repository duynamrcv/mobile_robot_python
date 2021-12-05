import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from eta3_spline_path import Eta3Path, Eta3PathSegment

show_animation = True

class MaxVelocityNotReached(Exception):
    def __init__(self, actual_vel, max_vel):
        self.message = 'Actual velocity {} does not equal desired max velocity {}!'.format(
            actual_vel, max_vel)

class Eta3Trajectory(Eta3Path):
    '''
    eta3_trajectory
    input: segments - list of Eta3PathSegment instances defining a continuous trajectory
    '''
    def __init__(self, segments, max_vel, v0=0.0, a0=0.0, max_accel=2.0, max_jerk=5.0):
        assert max_vel > 0 and v0 >= 0 and a0 >= 0 and max_accel > 0 and max_jerk > 0\
                and a0 <= max_accel and v0 <= max_vel
        super(Eta3Trajectory, self).__init__(segments=segments)
        self.total_length = sum([s.segment_length for s in self.segments])
        self.max_vel = float(max_vel)
        self.v0 = float(v0)
        self.a0 = float(a0)
        self.max_accel = float(max_accel)
        self.max_jerk = float(max_jerk)
        
        length_array = np.array([s.segment_length for s in self.segments])

        self.cum_lengths = np.concatenate((np.array([0]), np.cumsum(length_array)))

        # compute the velocity 
        self.velocity_profile()
        self.ui_prev = 0
        self.prev_seg_id = 0

    def velocity_profile(self):
        r'''                  /~~~~~----------------\
                             /                       \
                            /                         \
                           /                           \
                          /                             \
        (v=v0, a=a0) ~~~~~                               \ ~~~~~ (vf=0, af=0)
                    pos.|pos.|neg.|   cruise at    |neg.| neg. |neg.
                    max |max.|max.|     max.       |max.| max. |max.
                    jerk|acc.|jerk|    velocity    |jerk| acc. |jerk
            index     0    1    2      3 (optional)   4     5     6
        '''
        # delta_a - accel change from initial position to end of maximal jerk section
        delta_a = self.max_accel - self.a0
        # t_s1 - time-of-traversal of maximal jerk section
        t_s1 = delta_a/self.max_jerk
        # v_s1 - velocity at the end of the maximal jerk section
        v_s1 = self.v0 + self.a0*t_s1 + 1/2*self.max_jerk*t_s1**2
        # s_s1 - length of the maximal jerk section
        s_s1 = self.v0*t_s1 + 1/2*self.a0*t_s1**2 + 1/6*self.max_jerk*t_s1**3

        # t_sf: time of traversal  of final section, which is also maximal jerk, but has final velocity 0
        t_sf = self.max_accel/self.max_jerk
        # v_sf: velocity at beginning of final section
        v_sf = 1/2*self.max_jerk*t_sf**2
        # s_sf: length of final section
        s_sf = 1/6*self.max_jerk*t_sf**3

        # solve for the maximum achievable velocity based on the kinematic limits imposed by max_accel
        # and max_jerk this leads to a quadratic equation in v_max: a*v_max**2 + b*v_max + c = 0
        a = 1/self.max_accel
        b = 3/2*self.max_accel/self.max_jerk + v_s1/self.max_jerk \
            - (self.max_accel**2/self.max_jerk + v_s1)/self.max_accel
        c = s_s1 + s_sf - self.total_length - 7/3*self.max_accel**3/self.max_jerk**2 \
            - v_s1*(self.max_accel/self.max_jerk + v_s1/self.max_accel) \
            + (self.max_accel**2/self.max_jerk + v_s1/self.max_accel)**2/(2*self.max_accel)
        v_max = (-b + np.sqrt(b**2 - 4*a*c))/(2*a)

        # v_max represents the maximum velocity that could be attained if there was no cruise period
        # if this velocity is less than our desired max velocity, the max velocity needs to be updated
        if self.max_vel > v_max:
            self.max_vel = v_max

        # setup arrays to store values at END of trajectory sections
        self.times = np.zeros((7,))
        self.vels = np.zeros((7,))
        self.seg_lengths = np.zeros((7,))

        # section 0: max jerk tp to max acceleration
        self.times[0] = t_s1
        self.vels[0] = v_s1
        self.seg_lengths[0] = s_s1

        # section 1: accelerate at max_accel
        delta_v = self.max_vel - 1/2*self.max_accel**2/self.max_jerk - self.vels[0]

        self.times[1] = delta_v/self.max_accel
        self.vels[1] = self.vels[0] + self.max_accel*self.times[1]
        self.seg_lengths[1] = self.vels[0]*self.times[1] + 1/2*self.max_accel*self.times[1]**2

        # section 2: decrease acceleration until max speed
        self.times[2] = self.max_accel/self.max_jerk
        self.vels[2] = self.vels[1] + self.max_accel*self.times[2] - 1/2*self.max_jerk*self.times[2]**2
        if not np.isclose(self.vels[2], self.max_vel):
            raise MaxVelocityNotReached(self.vels[2], self.max_vel)
        self.seg_lengths[2] = self.vels[1]*self.times[2] + 1/2*self.max_accel*self.times[2]**2 \
                                - 1/6*self.max_jerk*self.times[2]**3

        # section 3: do in the last
        
        # section 4: apply min jerk until min acceleration
        self.times[4] = self.max_accel/self.max_jerk
        self.vels[4] = self.max_vel - 1/2*self.max_jerk*self.times[4]**2
        self.seg_lengths[4] = self.vels[4]*self.times[4] - 1/6*self.max_jerk*self.times[4]**3

        # section 5: deceleration at max rate
        delta_v = self.vels[4] - v_sf
        self.times[5] = delta_v/self.max_accel
        self.vels[5] = self.vels[4] - self.max_accel*self.times[5]
        self.seg_lengths[5] = self.vels[4]*self.times[5] - 1/2*self.max_accel*self.times[5]**2

        # section 6: max jerk to get zero velocity and zero acceleration
        self.times[6] = t_sf
        self.vels[6] = self.vels[5] - 1/2*self.max_jerk*t_sf**2

        try:
            assert np.isclose(self.vels[6], 0)
        except AssertionError as e:
            print('The final velocity {} is not zero'.format(self.vels[6]))
            raise e

        self.seg_lengths[6] = s_sf

        if self.seg_lengths.sum() < self.total_length:
            # section 3
            self.seg_lengths[3] = self.total_length - self.seg_lengths.sum()
            self.vels[3] = self.max_vel
            self.times[3] = self.seg_lengths[3]/self.max_vel

        # make sure that all of the times are positive, otherwise the
        # kinematic limits chosen cannot be enforced on the path
        assert(np.all(self.times >= 0))
        self.total_time = self.times.sum()
    
    def get_interp_param(self, seg_id, s, ui, tol=0.001):
        def f(u):
            return self.segments[seg_id].f_length(u)[0] - s
        
        def fprime(u):
            return self.segments[seg_id].s_dot(u)
        
        while (0 <= ui <= 1) and abs(f(ui)) > tol:
            ui -= f(ui)/fprime(ui)
        ui = max(0, min(ui, 1))
        return ui

    def calc_traj_point(self, time):
        # compute velocity at time
        if time <= self.times[0]:
            linear_velocity = self.v0 + 1/2*self.max_jerk*time**2
            s = self.v0*time + 1/6*self.max_jerk*time**3
            linear_accel = self.max_jerk*time
        elif time <= self.times[:2].sum():
            delta_t = time - self.times[0]
            linear_velocity = self.vels[0] + self.max_accel*delta_t
            s = self.seg_lengths[0] + self.vels[0]*delta_t + 1/2*self.max_accel*delta_t**2
            linear_accel = self.max_accel
        elif time <= self.times[:3].sum():
            delta_t = time - self.times[:2].sum()
            linear_velocity = self.vels[1] + self.max_accel*delta_t - 1/2*self.max_jerk*delta_t**2
            s = self.seg_lengths[:2].sum() + self.vels[1]*delta_t + 1/2*self.max_accel*delta_t**2 \
                - 1/6*self.max_jerk*delta_t**3
            linear_accel = self.max_accel - self.max_jerk*delta_t
        elif time <= self.times[:4].sum():
            delta_t = time - self.times[:3].sum()
            linear_velocity = self.vels[3]
            s = self.seg_lengths[:3].sum() + self.vels[3]*delta_t 
            linear_accel = 0
        elif time <= self.times[:5].sum():
            delta_t = time - self.times[:4].sum()
            linear_velocity = self.vels[3] - 1/2*self.max_jerk*delta_t**2
            s = self.seg_lengths[:4].sum() + self.vels[3]*delta_t - 1/6*self.max_jerk*delta_t**3
            linear_accel = -self.max_jerk*delta_t
        elif time <= self.times[:6].sum():
            delta_t = time - self.times[:5].sum()
            linear_velocity = self.vels[4] - self.max_accel*delta_t
            s = self.seg_lengths[:5].sum() + self.vels[4]*delta_t - 1/2*self.max_accel*delta_t**2
            linear_accel = -self.max_accel
        elif time < self.times.sum():
            delta_t = time - self.times[:6].sum()
            linear_velocity = self.vels[5] - self.max_accel*delta_t + 1/2*self.max_jerk*delta_t**2
            s = self.seg_lengths[:6].sum() + self.vels[5]*delta_t - 1/2*self.max_accel*delta_t**2 \
                + 1/6*self.max_jerk*delta_t**3
            linear_accel = -self.max_accel + self.max_jerk*delta_t
        else:
            linear_velocity = 0
            s = self.total_length
            linear_accel = 0
        seg_id = np.max(np.argwhere(self.cum_lengths <= s))

        if seg_id == len(self.segments):
            seg_id = -1
            ui = 1
        else:
            curr_segment_length = s - self.cum_lengths[seg_id]
            ui = self.get_interp_param(seg_id=seg_id, s=curr_segment_length, ui=self.ui_prev)
        
        if not seg_id == self.prev_seg_id:
            self.ui_prev = 0
        else:
            self.ui_prev = ui
        
        self.prev_seg_id = seg_id

        # compute angular velocity of current point = (ydd*xd - xdd*yd) / (xd**2 + yd**2)
        d = self.segments[seg_id].calc_deriv(ui, order=1)
        dd = self.segments[seg_id].calc_deriv(ui, order=2)

        # su - the rate of change of arclength wrt u
        su = self.segments[seg_id].s_dot(ui)
        if not np.isclose(su, 0) and not np.isclose(linear_velocity, 0):
            # ut - time-derivative of interpolation parameter u
            ut = linear_velocity/su
            # utt - time-derivative of ut
            utt = linear_accel/su - (d[0]*dd[0] + d[1]*dd[1])/su**2*ut
            xt = d[0]*ut
            yt = d[1]*ut
            xtt = dd[0]*ut**2 + d[0]*utt
            ytt = dd[1]*ut**2 + d[1]*utt
            angular_velocity = (ytt*xt - xtt*yt)/linear_velocity**2
        else:
            angular_velocity = 0

        # combine path point with orientation and velocities
        pos = self.segments[seg_id].calc_point(ui)
        state = np.array([pos[0], pos[1], np.arctan2(d[1], d[0]), linear_velocity, angular_velocity])
        return state

def test1(max_vel=0.5):

    for i in range(10):
        trajectory_segments = []
        # segment 1: lane-change curve
        start_pose = [0, 0, 0]
        end_pose = [4, 3.0, 0]
        # NOTE: The ordering on kappa is [kappa_A, kappad_A, kappa_B, kappad_B], with kappad_* being the curvature derivative
        kappa = [0, 0, 0, 0]
        eta = [i, i, 0, 0, 0, 0]
        trajectory_segments.append(Eta3PathSegment(
            start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

        traj = Eta3Trajectory(trajectory_segments,
                               max_vel=max_vel, max_accel=0.5)

        # interpolate at several points along the path
        times = np.linspace(0, traj.total_time, 101)
        state = np.empty((5, times.size))
        for j, t in enumerate(times):
            state[:, j] = traj.calc_traj_point(t)

        if show_animation:  # pragma: no cover
            # plot the path
            plt.plot(state[0, :], state[1, :])
            plt.pause(1.0)

    plt.show()
    # if show_animation:
    #     plt.close("all")

def test2(max_vel=1.0):
    trajectory_segments = []

    # segment 1: lane-change curve
    start_pose = [0, 0, 0]
    end_pose = [4, 4, 0]
    # NOTE: The ordering on kappa is [kappa_A, kappad_A, kappa_B, kappad_B], with kappad_* being the curvature derivative
    kappa = [0, 0, 0, 0]
    eta = [1, 0, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 2: line segment
    start_pose = [4, 4, 0]
    end_pose = [3, 10, 0]
    kappa = [0, 0, 0, 0]
    eta = [1, 0, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 3: cubic spiral
    start_pose = [3, 10, 0]
    end_pose = [0, 0, 0]
    kappa = [0, 0, 0, 0]
    eta = [1, 0, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # construct the whole path
    traj = Eta3Trajectory(trajectory_segments,
                           max_vel=max_vel, max_accel=0.5, max_jerk=1)

    # interpolate at several points along the path
    times = np.linspace(0, traj.total_time, 1001)
    state = np.empty((5, times.size))
    for i, t in enumerate(times):
        state[:, i] = traj.calc_traj_point(t)

    # plot the path

    if show_animation:
        fig, ax = plt.subplots()
        x, y = state[0, :], state[1, :]
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segs = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segs, cmap=plt.get_cmap('inferno'))
        ax.set_xlim(np.min(x) - 1, np.max(x) + 1)
        ax.set_ylim(np.min(y) - 1, np.max(y) + 1)
        lc.set_array(state[3, :])
        lc.set_linewidth(3)
        ax.add_collection(lc)
        axcb = fig.colorbar(lc)
        axcb.set_label('velocity(m/s)')
        ax.set_title('Trajectory')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.pause(1.0)

        fig1, ax1 = plt.subplots()
        ax1.plot(times, state[3, :], 'b-')
        ax1.set_xlabel('time(s)')
        ax1.set_ylabel('velocity(m/s)', color='b')
        ax1.tick_params('y', colors='b')
        ax1.set_title('Control')
        ax2 = ax1.twinx()
        ax2.plot(times, state[4, :], 'r-')
        ax2.set_ylabel('angular velocity(rad/s)', color='r')
        ax2.tick_params('y', colors='r')
        fig.tight_layout()
        plt.show()

def test3(max_vel=1.0):
    trajectory_segments = []

    # segment 1: lane-change curve
    start_pose = [0, 0, 0]
    end_pose = [4, 1.5, 0]
    # NOTE: The ordering on kappa is [kappa_A, kappad_A, kappa_B, kappad_B], with kappad_* being the curvature derivative
    kappa = [0, 0, 0, 0]
    eta = [4.27, 4.27, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 2: line segment
    start_pose = [4, 1.5, 0]
    end_pose = [5.5, 1.5, 0]
    kappa = [0, 0, 0, 0]
    eta = [0, 0, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # segment 3: cubic spiral
    start_pose = [5.5, 1.5, 0]
    end_pose = [7.5, 2, 0]
    kappa = [0, 0, 1, 1]
    eta = [1.88, 1.88, 0, 0, 0, 0]
    trajectory_segments.append(Eta3PathSegment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    # construct the whole path
    traj = Eta3Trajectory(trajectory_segments,
                           max_vel=max_vel, max_accel=0.5, max_jerk=1)

    # interpolate at several points along the path
    times = np.linspace(0, traj.total_time, 1001)
    state = np.empty((5, times.size))
    for i, t in enumerate(times):
        state[:, i] = traj.calc_traj_point(t)

    # plot the path

    if show_animation:
        fig, ax = plt.subplots()
        x, y = state[0, :], state[1, :]
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segs = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segs, cmap=plt.get_cmap('inferno'))
        ax.set_xlim(np.min(x) - 1, np.max(x) + 1)
        ax.set_ylim(np.min(y) - 1, np.max(y) + 1)
        lc.set_array(state[3, :])
        lc.set_linewidth(3)
        ax.add_collection(lc)
        axcb = fig.colorbar(lc)
        axcb.set_label('velocity(m/s)')
        ax.set_title('Trajectory')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.pause(1.0)

        fig1, ax1 = plt.subplots()
        ax1.plot(times, state[3, :], 'b-')
        ax1.set_xlabel('time(s)')
        ax1.set_ylabel('velocity(m/s)', color='b')
        ax1.tick_params('y', colors='b')
        ax1.set_title('Control')
        ax2 = ax1.twinx()
        ax2.plot(times, state[4, :], 'r-')
        ax2.set_ylabel('angular velocity(rad/s)', color='r')
        ax2.tick_params('y', colors='r')
        fig.tight_layout()
        plt.show()

if __name__ == "__main__":
    test2()