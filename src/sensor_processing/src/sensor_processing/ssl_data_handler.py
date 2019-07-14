#!/usr/bin/env  python
from ssl_receiver import *
from robocup_msgs.msg import Position, RobotData
from geometry_msgs.msg import Pose, Twist, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
import math

YELLOW = 0
BLUE = 1

class SSLDataHandler(object):
    
    def __init__(self, team_color=YELLOW):

        self.team_color = team_color

        # dictionary of teammates and opponenets
        # format: {data: RobotData, time: last_timestamp}
        self.friends = {}
        self.enemies = {}

        # ball data
        self.ball_pos = Pose()
        self.ball_speed = Twist()
        self.ball_time = 0.0

    def update_all(self, pkt):
        '''
        Updates all the data for the robots when new packets are received
        :param: pkt is protobuf packet received from SSL Vision
        '''
        time = pkt.detection.t_capture

        if self.team_color == YELLOW:
            is_friend = True
            for robot in pkt.detection.robots_yellow:
                self.update_robot(robot.robot_id, robot, time, is_friend)
            
            is_friend = False
            for robot in pkt.detection.robots_blue:
                self.update_robot(robot.robot_id, robot, time, is_friend)

        else:
            is_friend = True
            for robot in pkt.detection.robots_blue:
                self.update_robot(robot.robot_id, robot, time, is_friend)
            
            is_friend = False
            for robot in pkt.detection.robots_yellow:
                self.update_robot(robot.robot_id, robot, time, is_friend)

        if pkt.detection.balls:
            self.update_ball(pkt.detection.balls[0], time)

    def update_robot(self, r_id, robot, time, is_friend):
        if is_friend:
            # attempt to update existing robot data, otherwise, generate new robot data
            try:
                self.friends[r_id] = self.fill_robot_data(self.friends[r_id], robot, time)
            except KeyError:
                self.friends[r_id] = self.new_robot_data(r_id, robot, time)
        else:
            try:
                self.enemies[r_id] = self.fill_robot_data(self.enemies[r_id], robot, time)
            except KeyError:
                self.enemies[r_id] = self.new_robot_data(r_id, robot, time)
                

    def fill_robot_data(self, robot_data, robot, time):
            
        time_diff = time - robot_data['time']

        quat = robot_data['data'].r_pose.orientation
        angle = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        if time_diff > 0:
            robot_data['data'].r_twist.linear.x = (robot.x - robot_data['data'].r_pose.position.x) / time_diff
            robot_data['data'].r_twist.linear.y = (robot.y - robot_data['data'].r_pose.position.y) / time_diff
            robot_data['data'].r_twist.angular.z = (robot.orientation - angle[2]) / time_diff

        robot_data['data'].r_pose.position.x = robot.x
        robot_data['data'].r_pose.position.y = robot.y
        robot_data['data'].r_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.radians(robot.orientation)))
        robot_data['time'] = time

        return robot_data

    def new_robot_data(self, r_id, robot, time):
        robot_data = RobotData()
        robot_data.id = r_id
        robot_data.r_pose.position.x = robot.x
        robot_data.r_pose.position.y = robot.y
        robot_data.r_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.radians(robot.orientation)))
        return {'data': robot_data, 'time': time}
                    
    def update_ball(self, ball, time):
        time_diff = time - self.ball_time

        if time_diff > 0:
            self.ball_speed.linear.x = ball.x - self.ball_pos.position.x / time_diff
            self.ball_speed.linear.y = ball.y - self.ball_pos.position.y / time_diff

        self.ball_pos.position.x = ball.x
        self.ball_pos.position.y = ball.y
        self.ball_time = time

    def fill_msg(self, msg):
        for key, value in self.friends.iteritems():
            msg.friends.append(value['data'])
    
        for key, value in self.enemies.iteritems():
            msg.enemies.append(value['data'])

        msg.ball_pos = self.ball_pos
        msg.ball_speed = self.ball_speed
