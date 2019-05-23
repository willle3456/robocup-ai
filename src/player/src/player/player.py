#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Twist
from robocup_msgs.msg import Position, Strategy
#from robocup_msgs.srv import Command
from basic_skills.move_to import MoveTo
from basic_skills.path_planning.path_planner import Point
from basic_skills.robot import Robot
from action_selection.selectors import *
from strategy_execution.impl.test_strategy import TestStrategy

#Player Node
class Player(Robot):
    def __init__(self, id_num, num_bots=6, is_blue=False):
        super(Player, self).__init__(id_num, num_bots, is_blue)
        self.num = num_bots

        # for when trajectory generation is ready
        self.cmd_pub = rospy.Publisher('command_speed', Twist, queue_size=10)
        #self.cmd_client = rospy.ServiceProxy('player_cmds', Command)
        self.position_sub = rospy.Subscriber('locations', Position, self.update_position)
        self.strat_sub = rospy.Subscriber('strategy', Strategy, self.update_strat)

        self.strats = [TestStrategy()]
        self.strat_id = 0
    
        self.zone = 0
        self.prev_action = None
        self._default_action = MoveTo()
        self._default_action.set_robot(self)
        self._default_action.set_goal(self.ego_data.location.convert_to_point())
        
        # investigate changing how this is used to get initial goal generation
        self.init_flg = True
    
    def update_position(self, data):
        for f in data.friends:
            if f.id == self.id:
                self.ego_data.update(f.r_pose, f.r_twist)
                self.zone = get_zone(self.ego_data.location)
            else:
                self.friends[f.id].update(f.r_pose, f.r_twist)

        for e in data.enemies:
            self.enemies[e.id].update(e.r_pose, e.r_twist)

        if data.ball_pos is not None:
            self.ball.location.x = data.ball_pos.position.x
            self.ball.location.y = data.ball_pos.position.y
        
        if data.ball_speed is not None:
            self.ball.velocities.x = data.ball_speed.linear.x
            self.ball.velocities.y = data.ball_speed.linear.y
        
        if self.init_flg:
            self.init_flg = False
            self._default_action.set_goal(self.ego_data.location.convert_to_point())
        
    def update_strat(self, data):
        self.strat_id = data.strategy

    def run(self):
        if self.init_flg:
            print 'initializing'
            self._default_action.set_robot(self)
            self._default_action.set_goal(self.ego_data.location.convert_to_point())
            #self.run_action(0.01)
        else:
            next_action = self.strats[self.strat_id].apply(self.zone, self)

            if type(next_action) is not type(self.prev_action):
                #print 'adding action'
                self.add_action(next_action)
                self.prev_action = next_action

            if type(next_action) is MoveTo:
                #print 'setting goal'
                next_action.set_goal(Point(-2000.0, -2000.0))
        
            self.run_action(0.01)
            #self.add_action(next_action)
            #result = self.run_action()
            
            #setup cmd
            #cmd = Twist()
            #cmd.linear.x = result[0]
            #cmd.linear.y = result[1]
            #cmd.angular.z = result[2]
            #cmd_pub.publish(cmd)
