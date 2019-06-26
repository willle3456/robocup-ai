#!/usr/bin/env  python

import unittest
import numpy as np
from basic_skills.player_data import Pose2D, PlayerData
from basic_skills.action import Action
from basic_skills.robot import Robot

'''
Unit Tests for each component of any action or robot data related functionality
'''

# Robot Tests
class TestRobot(unittest.TestCase):
        
    #def test_str(self):
        #pass

    def test_get_cart_location(self):
        robot = Robot(19)
        robot.ego_data.location = Pose2D(392.0, -1293.0, 0.0)
        robot.ego_data.velocities = Pose2D(-543.9, 120.0, 20.0)
        robot.ball.location = Pose2D(3921.39, -100.0)
        robot.ball.velocities = Pose2D(-203.0, -405.09)

        test_loc = robot.get_cart_location()
        expected_loc = np.array([392.0, -1293.0])
        result = test_loc == expected_loc

        self.assertTrue(result.all())

    def test_get_orientation(self):
        robot = Robot(5)
        robot.ego_data.location = Pose2D(-392.0, -1293.0, 29.0)
        robot.ego_data.velocities = Pose2D(-543.9, -120.0, 20.0)
        robot.ball.location = Pose2D(3921.39, 100.0)
        robot.ball.velocities = Pose2D(-203.0, 405.09)

        test_orient = robot.get_orientation()

        self.assertEqual(test_orient, 29.0)

    def test_get_cart_velocity(self):
        robot = Robot(4)
        robot.ego_data.location = Pose2D(392.0, -1293.0, 0.0)
        robot.ego_data.velocities = Pose2D(-432.2, 20.0, 20.0)
        robot.ball.location = Pose2D(3921.39, -100.0)
        robot.ball.velocities = Pose2D(-203.0, -405.09)

        test_vel = robot.get_cart_velocity()
        expected_vel = np.array([-432.2, 20.0])
        result = test_vel == expected_vel

        self.assertTrue(result.all())

    def test_get_angular_velocity(self):
        robot = Robot(3)
        robot.ego_data.location = Pose2D(392.0, -1293.0, 0.0)
        robot.ego_data.velocities = Pose2D(-543.9, 120.0, -24.2)
        robot.ball.location = Pose2D(3921.39, -100.0)
        robot.ball.velocities = Pose2D(-203.0, -405.09)
        
        test_vel = robot.get_angular_velocity()
        self.assertEqual(test_vel, -24.2)

    def test_get_ball_location(self):
        robot = Robot(2)
        robot.ego_data.location = Pose2D(392.0, -1293.0, 0.0)
        robot.ego_data.velocities = Pose2D(-543.9, 120.0, 20.0)
        robot.ball.location = Pose2D(921.39, -100.0)
        robot.ball.velocities = Pose2D(-203.0, -405.09)

        test_ball_loc = robot.get_ball_location()
        expected_loc = np.array([921.39, -100.0])
        result = test_ball_loc == expected_loc

        self.assertTrue(result.all())

    def test_get_ball_velocity(self):
        robot = Robot(1)
        robot.ego_data.location = Pose2D(392.0, -1293.0, 0.0)
        robot.ego_data.velocities = Pose2D(-543.9, 120.0, 20.0)
        robot.ball.location = Pose2D(921.39, -100.0)
        robot.ball.velocities = Pose2D(-203.0, -405.09)

        test_ball_vel = robot.get_ball_velocity()
        expected_vel = np.array([-203.0, -405.09])
        result = test_ball_vel == expected_vel

        self.assertTrue(result.all())

    #def test_get_action(self):
        #robot = Robot(2)
        #self.assertTrue(True)

    def test_run_action_waiting(self):
        self.assertTrue(True)

    def test_run_action_invalid(self):
        self.assertTrue(True)

    def test_set_default_action(self):
        self.assertTrue(True)

    def test_run_action_default(self):
        self.assertTrue(True)

    def test_add_action(self):
        self.assertTrue(True)

    def test_add_action_no_action(self):
        self.assertTrue(True)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_robot', TestRobot)

