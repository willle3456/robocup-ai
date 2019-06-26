#!/usr/bin/env python

from basic_skills.action import CommandStatus
from basic_skills.move_to import MoveTo
from basic_skills.path_planning.sampler import ConfigSpace, sample_uniform
from action_selection import get_zone_boundaries
import random

class MoveToZone(MoveTo):
    def __init__(self, robot, zone=None):
        super(MoveToZone, self).__init__(robot)
        
        if zone is None:
            self.planner.goal = self._robot.ego_data.location
        else:
            self.planner.goal = self.get_zone_goal(zone)

    def get_zone_goal(self, zone):
        x_min, x_max, y_min, y_max = get_zone_boundaries(zone)
        tmp_space = ConfigSpace(x_min, x_max, y_min, y_max)
        return sample_uniform(tmp_space)

    def set_goal(self, zone):
        self.planner.goal = self.get_zone_goal(zone)

    def start(self):
        print 'starting again'
        goal_zone = random.randint(0, 19)
        robot_zone = self.get_robot().zone
        
        while goal_zone != robot_zone:
            goal_zone = random.randint(0,19)

        print 'new goal zone: {0}'.format(goal_zone)
        self.planner.goal = self.get_zone_goal(goal_zone)
        
        print self.planner.is_graph_empty()
        self.status = CommandStatus.RUNNING

    def end(self, status):
        self.planner.graph.clear()
