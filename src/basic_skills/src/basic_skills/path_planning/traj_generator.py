#!/usr/bin/env  python
from basic_skills.player_data import Pose2D
import math

class TrajGenerator(object):
    def __init__(self, max_vel=1000.0, max_acc=6000.0, max_ang_acc=math.pi, max_ang_vel=math.pi/2, t_a=0.050, t_f=0.222, delta_time=1/60.0, set_speed=False, polar_traj=True):
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_ang_acc = max_ang_acc
        self.max_ang_vel = max_ang_vel
        self.t_a = t_a
        self.t_f = t_f
        self.set_speed = set_speed
        self.polar_traj = polar_traj
        self.delta_time = delta_time

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


    # courtesy of: 
    # https://hackaday.io/project/5765-flexsea-wearable-robotics-toolkit/log/24796-trajectory-generation-trapezoidal-speed-profile
    # 1D only
    def get_point_to_point_traj(self, x_1, x_2, vel_i):

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

    def get_time_based_traj(self, x_1, x_2, last_vel):
        dist = x_2 - x_1
        max_vel = dist/ float(self.t_f - self.t_a)
        diff_vel = last_vel - max_vel
        tmp_acc = diff_vel / self.t_a
        acc_steps = int(self.t_a/self.delta_time)
        const_steps = int(self.t_f - (2 * self.t_a)/ self.delta_time)
        vel_incr = diff_vel / float(acc_steps)
        
        #print 'max vel: {0}, acc: {1}, diff_vel: {2}, vel step: {3}\n'.format(max_vel, tmp_acc, diff_vel, vel_incr)
        
        vels = []
        tmp_vel = last_vel
        # accelerate
        for i in range(acc_steps):
            tmp_vel += vel_incr
            vels.append(tmp_vel)
        # const speed
        for i in range(const_steps):
            tmp_vel = max_vel
            vels.append(tmp_vel)
        # decelerate
        for i in range(acc_steps):
            tmp_vel -= vel_incr
            vels.append(tmp_vel)
        #print vels
        return vels
    
    def rotate_coords(self, x, y, theta):
        x_new = (x * math.cos(theta)) - (y * math.sin(theta))
        y_new = (x * math.sin(theta)) + (y * math.cos(theta))
        return x_new, y_new

    # Get all of the trajectory
    def get_traj(self, way_pts, theta):
        x_traj = []
        y_traj = []
        #theta_traj = []
        last_x = 0.0
        last_y = 0.0
        #last_theta = 0.0

        #polar_wp = [pt.convert_to_polar() for pt in way_pts]
        for curr, nxt in zip(way_pts, way_pts[1:]):
        #for curr, nxt in zip(polar_wp, polar_wp[1:]):
            tmp_x = []
            tmp_y = []
            #tmp_theta = []
            if self.set_speed:
                tmp_x = self.get_point_to_point_traj(curr.x, nxt.x, last_x)
                tmp_y = self.get_point_to_point_traj(curr.y, nxt.y, last_y)
            #elif self.polar_traj:
                #tmp_x, tmp_theta = self.get_polar_traj(curr, nxt, last_x, last_theta)
            else:
                #print 'getting traj'
                tmp_x = self.get_time_based_traj(curr.x, nxt.x, last_x)
                tmp_y = self.get_time_based_traj(curr.y, nxt.y, last_y)
                #tmp_theta = self.get_time_based_traj(curr.y, nxt.y, last_theta)
            x_traj += tmp_x
            y_traj += tmp_y
            #theta_traj += tmp_theta
            last_x = tmp_x[len(tmp_x)-1] 
            last_y = tmp_y[len(tmp_y)-1]
            #last_theta = tmp_theta[len(tmp_theta)-1]
    
        x_traj.append(0.0)
        y_traj.append(0.0)
        #theta_traj = [theta for i in range(len(x_traj))]
        #return map(self.rotate_coords, x_traj, y_traj, theta_traj)
        #print zip(x_traj, y_traj)
        #theta_traj.append(0.0)
        #print theta_traj
        return zip(x_traj, y_traj)
        #return zip(x_traj, theta_traj)
