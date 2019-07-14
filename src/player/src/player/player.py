#/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Twist
from robocup_msgs.msg import Position, Strategy, Graph
#from robocup_msgs.srv import Command
from basic_skills.move_to import MoveTo
from skill_execution.zonal.move_to_zone import MoveToZone
from basic_skills.player_data import Pose2D
from basic_skills.robot import Robot
from action_selection.selectors import *
from strategy_execution.impl.test_strategy import TestStrategy
from sim_sender import SimSender
from grSim_Packet_pb2 import grSim_Packet
import time
import random

#Player Node
class Player(Robot):
    def __init__(self, id_num, num_bots=6, is_blue=False, is_sim=False):
        super(Player, self).__init__(id_num, num_bots, is_blue)
        self.num = num_bots

        # for when trajectory generation is ready
        # ROS control will be moved to the RPis
        self.cmd_pub = rospy.Publisher('/robocup_control/command_speed', Twist, queue_size=10)
        self.graph_pub = rospy.Publisher('graph_data', Graph, queue_size=10)        

        self.sender = SimSender()
        self.sender.connect()
        #self.cmd_client = rospy.ServiceProxy('player_cmds', Command)
        self.position_sub = rospy.Subscriber('locations', Position, self.update_position)
        self.strat_sub = rospy.Subscriber('strategy', Strategy, self.update_strat)

        # initialize strategy
        self.strats = [TestStrategy(self)]
        self.strat_id = 0
        self.zone = 0

        # initialize actions
        self.prev_action = None
        self._default_action = MoveToZone(self)
        #self._default_action.set_robot(self)
        
        # investigate changing how this is used to get initial goal generation
        self.init_flg = True
        self.is_sim = is_sim
        self.last_time = time.time()
    
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
            #self._default_action.set_goal(self.ego_data.location)
        
    def update_strat(self, data):
        self.strat_id = data.strategy

    def write_output(self, robot_command, result):
        """
        Populates the given grSim_Robot_Command with the current outputs
        :param robot_command: The grSim_Robot_Command to populate
        """
        robot_command.id = self.id

        # We don't set wheel speeds individually, so fill these out with zeros
        robot_command.wheelsspeed = False
        robot_command.wheel1 = 0.0
        robot_command.wheel2 = 0.0
        robot_command.wheel3 = 0.0
        robot_command.wheel4 = 0.0

        robot_command.veltangent = result[2]
        robot_command.velnormal = result[3]
        robot_command.velangular = result[4]

        # Fill out kicker information
        robot_command.kickspeedx = 0.0
        robot_command.kickspeedz = 0.0
        robot_command.spinner = False
    
    def send_commands(self, result):
        """
        Sends all robot outputs in a packet to grSim
        """
        sim_packet = grSim_Packet()
        #sim_packet.commands.isteamyellow = self.team_bots.team == Team.YELLOW
        sim_packet.commands.isteamyellow = True
        sim_packet.commands.timestamp = 0

        self.write_output(sim_packet.commands.robot_commands.add(), result)
        self.sender.send(sim_packet)

    def run(self):
        result = [0 for i in range(5)]

        if self.init_flg:
            result[2] = 0.0
            result[3] = 0.0
            result[4] = 0.0
        else:
            next_action = self.strats[self.strat_id].apply(self.zone, self)
            
            if type(next_action) is not type(self.prev_action):
                self.add_action(next_action)
                self.prev_action = next_action
                
            if type(self._action) is MoveToZone:
                self._action.set_goal(random.randint(0,19))

            #print type(self._action)
            result  = self.run_action(0.01)
            self.graph_pub.publish(self._action.planner.get_graph())

        if not self.is_sim:
            cmd = Twist()
            cmd.linear.x = result[2]
            cmd.linear.y = result[3]
            cmd.angular.z = result[4]
            self.cmd_pub.publish(cmd)
        else:
            self.send_commands(result)          
