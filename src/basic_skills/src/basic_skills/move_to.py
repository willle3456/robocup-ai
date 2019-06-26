#!/usr/env/bin  python
from action import Action, CommandStatus
from path_planning.rrt import RRT
from path_planning.prm import PRM
from path_planning.fmt import FMT
#from path_planning.planner_plotter import PlannerPlotter
from player_data import Pose2D
from path_planning.traj_generator import TrajGenerator
import networkx as nx
import math
import time

class MoveTo(Action):
    def __init__(self, robot, goal=None):
        super(MoveTo, self).__init__()
        self.planner = RRT(step_size=400, num_iter=200)
        #self.planner = PRM(num_samples=600, num_neigh=5)
        #self.planner = FMT(num_samples=400, radius=800)
        self.traj_gen = TrajGenerator(t_a=0.45, t_f=1.200, delta_time=1/60.0, set_speed=False, polar_traj=False)
        #self.plotter = PlannerPlotter(self.planner)
        
        self.traj = []
        self.traj_idx = 0
        self.start_time = 0
        self.generated = False

        self.set_robot(robot)
        self.planner.goal = goal if goal is not None else self._robot.ego_data.location
        self.last_x = 0.0
        self.last_y = 0.0
    
    def set_goal(self, goal):
        self.planner.goal = goal
    
    def get_players_as_obs(self):
        obs = []
        for k,f in self._robot.friends.iteritems():
            o = Pose2D(f.location.x, f.location.y, 0)
            obs.append(o)
            
        for k,e in self._robot.enemies.iteritems():
            o = Pose2D(e.location.x, e.location.y, 0)
            obs.append(o)

        return obs
    
    def start(self):
        self.run(0.01)

    def run(self, delta_time):

        #print 'running'
        # update environment
        #self.update_plot()
        obs = self.get_players_as_obs()
        self.planner.update_obstacles(obs)
        #self.plotter.set_current(self._robot.ego_data.location)

        # initialize start
        self.planner.start = self._robot.ego_data.location
        self.planner.theta = 0.0

        # run planner
        self.planner.run()
        '''
        x_traj = self.traj_gen.get_time_based_traj(self.planner.path[0].x, self.planner.path[1].x, self.last_x)
        y_traj = self.traj_gen.get_time_based_traj(self.planner.path[0].y, self.planner.path[1].y, self.last_y)

        x, y = self.traj_gen.rotate_coords(x_traj[0], y_traj[0], self._robot.get_orientation())

        self.norm_vel = x/1000.0
        self.tang_vel = y/1000.0
        self.rot_vel = 0.0


        print 'vels: {0}, {1}'.format(x, y)
        self.last_x = x
        self.last_y = y
        '''
        if not self.traj:
            self.traj = self.traj_gen.get_traj(self.planner.path, 0.0)

        elif self.traj_idx < len(self.traj):
            x, y = self.traj_gen.rotate_coords(self.traj[self.traj_idx][0], self.traj[self.traj_idx][1], self.get_robot().get_orientation())

            self.traj_idx += 1

            self.norm_vel = x/600.0
            self.tang_vel = y/600.0
            self.rot_vel = 0.0
        else:
            self.norm_vel = 0.0
            self.tang_vel = 0.0
            self.rot_vel = 0.0
            self.status = CommandStatus.COMPLETED
            self.traj_idx = 0
            self.traj = []

        return [self.kick, self.chip, self.norm_vel, self.tang_vel, self.rot_vel]
