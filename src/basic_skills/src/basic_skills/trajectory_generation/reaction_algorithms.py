'''
This module contains all the algorithms that are needed modify the goal trajectories
'''
import math

class Reactor(object):
    def __init__(self):
        pass

    def apply_reaction(self):
        pass

class APF(Reactor):
    def __init__(self):
        super(APF, self).__init__()
        self.matrix = [[]] # need to figure out which library to use for this
        self.scaling_factor = 10
        self.apply_goal_gradient()

    def apply_reaction(self, robot):
        self.update(robot.obs)
        return self.get_velocity(robot)

    def update(self, obs):
        for ob in obs:
            self.apply_obstacle_gradient(ob)

    def get_velocity(self, robot):
        self.apply_sobel(0)
        self.apply_sobel(1)

        position = robot.position
        return self.get_gradient( position )

    def get_gradient(self, x, y):
        return -1 * self.scaling_factor * self.matrix[x][y]

    def apply_goal_gradient(self, goal):
        for i in len(self.matrix[0]):
            for j in len(self.matrix[i]):
                self.matrix[i][j] = self.distance( (i, j), goal) ** 2

    def apply_obstacle_gradient(self, obs):
        for i in len(self.matrix[0]):
            for j in len(self.matrix[i]):
                self.matrix[i][j] = self.distance( (i, j), obs) ** -1

    def distance(self, pt_1, pt_2):
        x_diff = pt_1[0] - pt_2[0]
        y_diff = pt_1[1] - pt_2[1]
        return math.sqrt(x_diff ** 2 + y_diff ** 2)

class ORCA(Reactor):
    def __init__(self):
        super(ORCA, self).__init__()

    def apply_reaction(self, robot):
        return self.get_safe_velocity(robot.obs)

    def get_safe_velocity(self, vel_x, vel_y, obs):
        v_obs = [self.get_velocity_obstacles(ob) for ob in obs]
        safe_region = self.get_safe_region(v_obs)
        safe_velocity = self.get_closest(vel_x, vel_y, safe_region)
        return safe_velocity

    def get_velocity_obstacles(self, obs):
        pass

    def get_safe_region(self, v_obs):
        pass

    def get_closest(self, x, y, region):
        pass
