#!/usr/bin/env  python
#from basic_skills.player_data import Pose2D
#from reaction_algorithms import *
import math

ACC = 0
CONST = 1
DEC = 2
DONE = 3

class TrajGenerator(object):
    def __init__(self, max_vel=1000.0, max_acc=6000.0, freq=60.0, reactor=None):
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.delta_time = 1/freq
        self.reactor = reactor

        # profile state for online mode
        self.traj_state = ACC
        self.last_vel = 0.0
        self.acc_dist = 0.5 * self.max_acc * self.delta_time * self.delta_time

        # reached waypoint
        self.epsilon = 1.0

        # expected position for control
        self.expected_position = 0.0

        # feedback controller
        self.controller = PIDController(p_gain = 10.0)

        # waypoint to waypoint
        self.traj = [[], []]
        self.traj_idx

    def get_point_to_point_params(self, dist, vel_i, max_vel):
        vel_diff = max_vel - vel_i
        acc_time = abs(vel_diff) / self.max_acc
        acc_steps = int(acc_time/ self.delta_time)

        if acc_steps < 1:
            acc_steps = 1
        vel_incr_step = vel_diff / float(acc_steps)

        tmp_x = 0
        for i in range(acc_steps):
            tmp_x += (i * vel_incr_step * self.delta_time)

        return abs(tmp_x), acc_steps, vel_incr_step


    def get_online_position(self, x_1, x_2, vel_i):
        vel = 0.0
        dist = x_2 - x_1

        #TODO might need to add a done check
        if self.traj_state == ACC:
            vel = self.last_vel + self.max_acc

            if vel >= self.max_vel:
                self.traj_state = CONST

            elif dist < self.acc_dist:
                self.traj_state = DEC

        elif self.traj_state == CONST:
            vel = self.max_vel

            if dist < self.acc_dist:
                self.traj_state = DEC

        elif self.traj_state == DEC:
            vel = self.last_vel - self.max_acc

            if dist < self.epsilon:
                self.traj_state = DONE
        else:
            self.traj_state = ACC

        self.last_vel = vel
        self.expected_position = x_1 + (vel * self.delta_time)

        return vel

    # courtesy of:
    # https://hackaday.io/project/5765-flexsea-wearable-robotics-toolkit/log/24796-trajectory-generation-trapezoidal-speed-profile
    # 1D only
    def get_waypoint_traj(self, x_1, x_2, vel_i):

        dist = x_2 - x_1
        sign = math.copysign(1, dist)

        max_vel = sign * self.max_vel

        acc_dist, acc_steps, vel_incr_step = self.get_point_to_point_params(dist, vel_i, max_vel)
        tot_acc_dist = 2 * acc_dist

        # assuming same acceleration and deceleration
        if tot_acc_dist > dist:
            max_vel = sign * math.sqrt(self.max_acc * abs(dist))
            acc_dist, acc_steps, vel_incr_step = self.get_point_to_point_params(dist, vel_i, max_vel)
            tot_acc_dist = 2 * acc_dist

        const_vel_dist = abs(dist)  - tot_acc_dist
        const_vel_time = const_vel_dist/ abs(max_vel)
        #print const_vel_time
        const_vel_steps = int(const_vel_time / self.delta_time)
        #print const_vel_steps

        vels = []
        tmp_vel = vel_i
        # accerlating
        for i in range(acc_steps):
            tmp_vel += vel_incr_step
            vels.append(tmp_vel)

        # constant speed
        for i in range(const_vel_steps):
            vels.append(tmp_vel)

        # decelerating
        for i in range(acc_steps):
            tmp_vel -= vel_incr_step
            vels.append(tmp_vel)

        #print 'Velocities: {0}'.format(vels)
        return vels

    # TODO move to move_to action
    def rotate_coords(self, x, y, theta):
        x_new = (x * math.cos(theta)) - (y * math.sin(theta))
        y_new = (x * math.sin(theta)) + (y * math.cos(theta))
        return x_new, y_new

    '''
    # Get all of the trajectory
    def get_traj(self, way_pts, theta):
        x_traj = []
        y_traj = []

        last_x = 0.0
        last_y = 0.0

        for curr, nxt in zip(way_pts, way_pts[1:]):
            tmp_x = []
            tmp_y = []

            tmp_x = self.get_point_to_point_traj(curr.x, nxt.x, last_x)
            tmp_y = self.get_point_to_point_traj(curr.y, nxt.y, last_y)

            x_traj += tmp_x
            y_traj += tmp_y
            last_x = tmp_x[len(tmp_x)-1]
            last_y = tmp_y[len(tmp_y)-1]

        x_traj.append(0.0)
        y_traj.append(0.0)

        if self.reactor:
            self.reactor.apply_reaction()

        self.rotate(x_traj, y_traj, theta)
        return zip(x_traj, y_traj)
    '''
    def get_velocity(self, curr_position, next_point):
        next_x = 0.0
        next_y = 0.0

        if self.online:
            next_x = self.get_online_position(curr_position.x, next_point.x)
            next_y = self.get_online_position(curr_position.y, next_point.y)
        else:
            if not self.traj:
                self.traj_idx = 0
                self.traj = self.get_waypoint_traj(curr_position, next_point, self.last_vel)
            #TODO get entire path trajectory code

            next_x = self.traj[0][self.traj_idx]
            next_y = self.traj[1][self.traj_idx]
            self.traj_idx += 1

        # TODO add reactor code

        final_x = self.controller.apply(next_x)
        final_y = self.controller.apply(next_y)

        return final_x/ self.delta_time, final_y / self.delta_time

'''
Generic implementation of a simple PID controller
'''
class PIDController(object):
    def __init__(self, p_gain = 0.0, i_gain = 0.0, d_gain = 0.0):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain

        self.total_error = 0.0
        self.last_error = 0.0
        self.expected_position = 0.0

    def apply(self, command, current_reading):
        error = command - current_reading
        change_error = error - self.last_error
        self.total_error += error

        self.last_error = error

        return (self.p_gain * error) + (self.i_gain * self.acc_error) + (self.d_gain * change_error)
