#!/usr/bin/env  python

import unittest
import socket
import random
from sensor_processing.ssl_receiver import SSLReceiver
from sensor_processing.ssl_data_handler import SSLDataHandler
from sensor_processing.messages_robocup_ssl_detection_pb2 import SSL_DetectionBall, SSL_DetectionRobot
from sensor_processing.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from subprocess import Popen
'''
Unit Tests for the utility functions for SSLVisionNode
'''
# Receiver Tests
class TestReceiverOpen(unittest.TestCase):
    def setUp(self):
        self.test_vision = Popen(['/home/developer/Documents/grSim/bin/grsim', '--headless'])

    def tearDown(self):
        self.test_vision.kill()
        
    def test_open_success(self):
        receiver = SSLReceiver()
        
        try:
            receiver.open()

        except socket.error:
            receiver.close()
            self.assertTrue(False)

        receiver.close() 
        self.assertTrue(True)

    def test_open_fail_port(self):
        receiver = SSLReceiver(port=80)
        
        try:
            receiver.open()

        except socket.error:
            receiver.close()
            self.assertTrue(True)

        receiver.close() 
        self.assertTrue(True)

class TestReceiveMsg(unittest.TestCase):
    def setUp(self):
        self.test_vision = Popen(['/home/developer/Documents/grSim/bin/grsim'])

    def tearDown(self):
        self.test_vision.kill()
        
    def test_recv_msg_success(self):
        receiver = SSLReceiver()
        receiver.open()
        msg = receiver.recv_msg()
        
        receiver.close()
        self.assertEqual(type(msg), SSL_WrapperPacket, type(msg))
    
    def test_recv_msg_fail(self):
        receiver = SSLReceiver(port=10000)
        receiver.open()

        msg = receiver.recv_msg()

        receiver.close()
        self.assertEqual(type(msg), type(None))

# Data Processing Tests
class TestRobotDataProcessing(unittest.TestCase):
    def test_new_robot_data(self):
        handler = SSLDataHandler()
        r_id = 0
        robot = SSL_DetectionRobot()
        time = 10000.0
        
        test_value = handler.new_robot_data(r_id, robot, time)
        expected_robot = RobotData()
        expected_time = time
        expected_return = {'data': expected_robot, 'time': expected_time}

        self.assertEqual(test_value, expected_return)
    
    #TODO new_robot_data input type validation, just in case

    def test_fill_robot_data(self):
        handler = SSLDataHandler()
        test_data = SSL_DetectionRobot()
        test_robot = RobotData() #it's a message type

        test_robot.r_pose.position.x = 1.0
        test_robot.r_pose.position.y = 1.0
        test_robot.r_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 1.0))

        test_dict = {'data': test_robot, 'time': 9999.0}
        test_time = 10000.0

        result = handler.fill_robot_data(test_dict, test_data, test_time)
        
        expected_velocities = Twist()
        expected_velocities.linear.x = -1.0
        expected_velocities.linear.y = -1.0
        expected_velocities.angular.z = -1.0

        self.assertEqual(result['data'].r_pose, test_robot['data'].position)
        self.assertEqual(result['time'], test_time)
        self.assertEqual(result['data'].r_twist, expected_velocities)
        

    def test_fill_robot_data_bad_time(self):
        handler = SSLDataHandler()
        test_data = SSL_DetectionRobot()
        test_robot = RobotData() #it's a message type

        test_robot.r_pose.position.x = 1.0
        test_robot.r_pose.position.y = 1.0
        test_robot.r_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 1.0))

        test_dict = {'data': test_robot, 'time': 10001.0}
        test_time = 10000.0

        result = handler.fill_robot_data(test_dict, test_data, test_time)

        self.assertEqual(result['data'].r_pose, test_robot.r_pose)
        self.assertEqual(result['time'], test_time)

        # velocity is expected to not change
        self.assertEqual(result['data'].r_twist, test_robot.r_twist)

    #TODO fill_robot_data input type valdiation, just in case

    def test_update_robot_new_friend(self):
        handler = SSLDataHandler()
        
        robot_idx = 0
        robot = SSL_DetectionRobot()
        time = 1000.0
        
        handler.update_robot(robot_idx, robot, time, is_friend=True)

        self.assertEqual(handler.friends[robot_idx]['data'], robot)
        self.assertEqual(handler.firends[robot_idx]['time'], time)

    def test_update_robot_old_friend(self):
        handler = SSLDataHandler()

        robot_idx = 0
        robot = SSL_DetectionRobot()
        robot_1 = SSL_DetectionRobot()
        time = 1000.0
        time_1 = 1001.0

        handler.update_robot(robot_idx, robot, time, is_friend=True)
        handler.update_robot(robot_idx, robot, time, is_friend=True)

        self.assertEqual(handler.friends[robot_idx]['data'].r_pose, robot_1.r_pose)
        self.assertEqual(handler.friends[robot_idx]['time'], time_1)
        self.assertEqual(handler.friends[robot_idx]['data'].r_twist, expected_twist)

    def test_update_robot_new_opp(self):
        handler = SSLDataHandler()
        
        robot_idx = 0
        robot = SSL_DetectionRobot()
        time = 1000.0
        
        handler.update_robot(robot_idx, robot, time, is_friend=False)

        self.assertEqual(handler.enemies[robot_idx]['data'], robot)
        self.assertEqual(handler.enemies[robot_idx]['time'], time)

    def test_update_robot_old_opp(self):
        handler = SSLDataHandler()

        robot_idx = 0
        robot = SSL_DetectionRobot()
        robot_1 = SSL_DetectionRobot()
        time = 1000.0
        time_1 = 1001.0

        handler.update_robot(robot_idx, robot, time, is_friend=False)
        handler.update_robot(robot_idx, robot, time, is_friend=False)

        self.assertEqual(handler.enemies[robot_idx]['data'].r_pose, robot_1.r_pose)
        self.assertEqual(handler.enemies[robot_idx]['time'], time_1)
        self.assertEqual(handler.enemies[robot_idx]['data'].r_twist, expected_twist)

class TestBallDataProcessing(unittest.TestCase):
    def test_update_new_ball(self):
        handler = SSLDataHandler()
        
        ball = SSL_DetectionBall()
        time = 1000.0
        handler.update_ball(ball, time)
        
        expected_ball = Pose()

        self.assertEqual(self.ball_pos, expected)
        
    def test_update_old_ball(self):
        handler = SSLDataHandler()

        ball_1 = SSL_DetectionBall()
        time_1 = 1000.0
        result = handler.update_ball(ball_1, time)

        ball_2 = SSL_DetectionBall()
        ball_2.x = 1.0
        ball_2.y = 1.0
        time_2 = 1001.0
        result = handler.update_ball(ball_2, time)

        expected_ball = Pose()
        expected_ball.position.x = 1.0
        expected_ball.position.y = 1.0
        expected_ball_speed = Twist()
        expected_ball_speed.linear.x = 1.0
        expected_ball_speed.linear.y = 1.0
        
        self.assertEqual(handler.ball_pos, expected_ball)
        self.assertEqual(handler.ball_speed, expected_ball_speed)
        self.assertEqual(handler.ball_time, time_2)

    def test_update_ball_bad_time(self):
        handler = SSLDataHandler()
        
        ball_1 = SSL_DetectionBall()
        time_1 = 1000.0
        result = handler.update_ball(ball_1, time)

        ball_2 = SSL_DetectionBall()
        ball_2.x = 1.0
        ball_2.y = 1.0
        time_2 = 999.0
        result = handler.update_ball(ball_2, time)

        expected_ball = Pose()
        expected_ball.position.x = 1.0
        expected_ball.position.y = 1.0
        expected_ball_speed = Twist()
        
        self.assertEqual(handler.ball_pos, expected_ball)
        self.assertEqual(handler.ball_speed, expected_ball_speed)
        self.assertEqual(handler.ball_time, time_2)
        

class TestAllDataProcessing(unittest.TestCase):

    #TODO more complete test to set all values of packet
    def test_update_all_yellow(self):
        handler = SSLDataHandler(team_color=blue)
        pkt = SSL_WrapperPacket()
        pkt.detection.t_capture = 1000.0
        
        ball = pkt.detection.balls.add()
        ball.x = 1.0
        ball.y = -1.0
        
        for i in range(6):
            yellow = pkt.detection.robots_yellow.add()
            yellow.x = i * 1000.0
            yellow.y = i * -1000.0
            yellow.orientation = 0.0

            blue = pkt.detection.robots_blue.add()
            blue.x = i * 500.0
            blue.y = i * -500.0
            blue.orientation = 180.0
        
        handler.update_all(pkt)

        expected_ball = Pose()
        expected_ball.x = 1.0
        expected_ball.y = -1.0

        expected_friends = [RobotData() for i in range(6)]
        expected_enemies = [RobotData() for i in range(6)]

        for i in range(6):
            expected_friends[i].r_pose.position.x = i * 1000.0
            expected_friends[i].r_pose.position.y = i * -1000.0
            expected_friends[i].r_pose.orientation = 0.0
            
            expected_enemies[i].r_pose.position.x = i * 500.0
            expected_enemies[i].r_pose.position.y = i * -500.0
            expected_enemies[i].r_pose.orientation = 180.0

        # check all the components are where they are expected
        self.assertEqual(handler.ball_pos, expected_ball)
        self.assertEqual(handler.friends, expected_friends)
        self.assertEqual(handler.enemies, expected_enemies)

    def test_update_all_blue(self):
        handler = SSLDataHandler(team_color=yellow)
        pkt = SSL_WrapperPacket()
        pkt.detection.t_capture = 1000.0
        
        ball = pkt.detection.balls.add()
        ball.x = 1.0
        ball.y = -1.0
        
        for i in range(6):
            yellow = pkt.detection.robots_yellow.add()
            yellow.x = i * 1000.0
            yellow.y = i * -1000.0
            yellow.orientation = 0.0

            blue = pkt.detection.robots_blue.add()
            blue.x = i * 500.0
            blue.y = i * -500.0
            blue.orientation = 180.0
        
        handler.update_all(pkt)

        expected_ball = Pose()
        expected_ball.x = 1.0
        expected_ball.y = -1.0

        expected_friends = [RobotData() for i in range(6)]
        expected_enemies = [RobotData() for i in range(6)]

        for i in range(6):
            expected_friends[i].r_pose.position.x = i * 1000.0
            expected_friends[i].r_pose.position.y = i * -1000.0
            expected_friends[i].r_pose.orientation = 0.0
            
            expected_enemies[i].r_pose.position.x = i * 500.0
            expected_enemies[i].r_pose.position.y = i * -500.0
            expected_enemies[i].r_pose.orientation = 180.0

        # check all the components are where they are expected
        self.assertEqual(handler.ball_pos, expected_ball)
        self.assertEqual(handler.friends, expected_friends)
        self.assertEqual(handler.enemies, expected_enemies)
    
    def test_update_all_ball(self):
        handler = SSLDataHandler(team_color=yellow)
        pkt = SSL_WrapperPacket()
        pkt.detection.t_capture = 1000.0
        
        ball = pkt.detection.balls.add()
        ball.x = 1.0
        ball.y = -1.0
        
        for i in range(6):
            yellow = pkt.detection.robots_yellow.add()
            yellow.x = i * 1000.0
            yellow.y = i * -1000.0
            yellow.orientation = 0.0

            blue = pkt.detection.robots_blue.add()
            blue.x = i * 500.0
            blue.y = i * -500.0
            blue.orientation = 180.0
        
        handler.update_all(pkt)

        expected_ball = Pose()
        expected_ball.x = 1.0
        expected_ball.y = -1.0

        expected_friends = [RobotData() for i in range(6)]
        expected_enemies = [RobotData() for i in range(6)]

        for i in range(6):
            expected_friends[i].r_pose.position.x = i * 1000.0
            expected_friends[i].r_pose.position.y = i * -1000.0
            expected_friends[i].r_pose.orientation = 0.0
            
            expected_enemies[i].r_pose.position.x = i * 500.0
            expected_enemies[i].r_pose.position.y = i * -500.0
            expected_enemies[i].r_pose.orientation = 180.0

        # check all the components are where they are expected
        self.assertEqual(handler.ball_pos, expected_ball)
        self.assertEqual(handler.friends, expected_friends)
        self.assertEqual(handler.enemies, expected_enemies)

    def test_fill_msg_empty(self):
        handler = SSLDataHandler()
        msg = Position()
        handler.fill_msg(msg)
        
        expected_msg = Position()
        self.assertEqual(msg, expected_msg)

    def test_fill_msg_just_ball(self):
        handler = SSLDataHandler()
        msg = Position()
        ball = SSL_DetectionBall()
        ball.x = 10.0
        ball.y = -10.0

        handler.update_ball(ball)
        handler.fill_msg(msg)
        
        expected_msg = Position()
        expected_msg.ball_pos = 10.0
        expected_msg.ball_pos = -10.0
        self.assertEqual(msg, expected_msg)

    def test_fill_msg_just_friend(self):
        handler = SSLDataHandler()
        msg = Position()
        handler.update_all()
        handler.fill_msg(msg)
        
        expected_msg = Position()
        self.assertEqual(msg, expected_msg)

    def test_fill_msg_just_opp(self):
        self.assertTrue(True)

    def test_fill_msg_just_robots(self):
        self.assertTrue(True)

    def test_fill_msg_no_yellow(self):
        self.assertTrue(True)

    def test_fill_msg_no_blue(self):
        self.assertTrue(True)

    def test_fill_msg(self):
        self.assertTrue(True)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('sensor_processing', 'test_receiver_open', TestReceiverOpen)
    rosunit.unitrun('sensor_processing', 'test_receive_msg', TestReceiveMsg)
    #rosunit.unitrun('sensor_processing', 'test_robot_data_processing', TestRobotDataProcessing)
    #rosunit.unitrun('sensor_processing', 'test_ball_data_processing', TestBallDataProcessing)
    #rosunit.unitrun('sensor_processing', 'test_all_data_processing', TestAllDataProcessing)
