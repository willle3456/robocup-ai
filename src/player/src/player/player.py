#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from robocup_msgs.msg import Position
from basic_skills.path_planning.path_planner import Point
from robot import Robot

#Player Node
class Player(Robot):
    def __init__(self, planner, id_num, num_bots=6, color='yellow'):
        super(Player, self).__init__(Pose(), Twist(), id_num)
        self.num = num_bots
        self.friends = {i : Robot(Pose(), Twist(), i) for i in range(self.num)if i != self.id_num}
        self.enemies = {i : Robot(Pose(), Twist(), i) for i in range(self.num)}
        self.color = color
        # for when trajectory generation is ready
        #self.cmd_pub = rospy.Publisher('command_speed', Twist, queue_size=10)
        self.position_sub = rospy.Subscriber('locations', Position, self.update_positions)
        self.planner = planner
        self.planner.goal = Point(-2000.0, -2000.0)
        # investigate changing how this is used to get initial goal generation
        self.init_flg = False

    def update_positions(self, data):
        for f in data.friends:
            if f.id == self.id_num:
                self.update_vals(f.r_pose, f.r_twist)
                if not self.init_flg:
                    self.init_flg = True
                    self.planner.start = Point(self.pos.position.x, self.pos.position.y)
                    #self.planner.graph.add_node(self.planner.start)
                '''
                revisit this when the robots start moving
                else:
                    self.planner.graph.remove_node(self.planner.start)
                    self.planner.start = Point(self.pos.position.x, self.pos.position.y)
                    self.planner.graph.add_node(self.planner.start)
                '''
            else:
                self.friends[f.id].update_vals(f.r_pose, f.r_twist)
        for e in data.enemies:
            self.enemies[e.id].update_vals(e.r_pose, e.r_twist)

    def get_players_as_obs(self):
        obs = []
        for k,f in self.friends.iteritems():
            o = Point(f.pos.position.x, f.pos.position.y)
            obs.append(o)
            
        for k,e in self.enemies.iteritems():
            o = Point(e.pos.position.x, e.pos.position.y)
            obs.append(o)

        return obs
            
    def run(self):
        if self.init_flg:
            obs = self.get_players_as_obs()
            self.planner.update_obstacles(obs)
            self.planner.run()
