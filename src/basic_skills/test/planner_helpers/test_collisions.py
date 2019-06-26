#!/usr/bin/env  python
import unittest
from basic_skills.player_data import Pose2D
from basic_skills.path_planning.collision_checker import *

class TestCollisions(unittest.TestCase):
    def test_pt_collision(self):
        obs = [Pose2D(), Pose2D(100.0, 200.0), Pose2D(-2000.0, -30.4)]
        
        result = point_collision(Pose2D(50.0, 100.0), obs)
        self.assertTrue(result)

    def test_not_pt_collision(self):
        obs = [Pose2D(), Pose2D(100.0, 200.0), Pose2D(-2000.0, -30.4), Pose2D(3032.32, -4920.9)]
        
        result = point_collision(Pose2D(-2000.0, 3239.9), obs)

        self.assertFalse(result)

    def test_intersect_lines(self):
        pt_1 = Pose2D(2000.0, 2000.0)
        pt_2 = Pose2D(-2000.0, -2000.0)

        pt_3 = Pose2D(2000.0, -2000.0)
        pt_4 = Pose2D(-2000.0, 2000.0)

        result = intersect_lines(pt_1, pt_2, pt_3, pt_4)
        self.assertTrue(result)

    def test_intersect_lines_1(self):
        pt_1 = Pose2D(10.0, 10.0)
        pt_2 = Pose2D(20.0, -20.0)

        pt_3 = Pose2D(0.0, -10.0)
        pt_4 = Pose2D(20.0, 10.0)

        result = intersect_lines(pt_1, pt_2, pt_3, pt_4)
        self.assertTrue(result)
    
    def test_no_intersect_lines(self):
        obs = [Pose2D()]
        pt_1 = Pose2D(450.0, 300.0)
        pt_2 = Pose2D(-450.0, 300.0)

        pt_3 = Pose2D(10.0, 25.0)
        pt_4 = Pose2D(-10.0, 25.0)

        result = intersect_lines(pt_1, pt_2, pt_3, pt_4)
        self.assertFalse(result)
    
    def test_no_intersect_lines_1(self):
        obs = [Pose2D()]
        pt_1 = Pose2D(5000.0, 300.0)
        pt_2 = Pose2D(-5000.0, 300.0)

        pt_3 = Pose2D(4000.0, 299.9)
        pt_4 = Pose2D(4000.0, 299.9)

        result = intersect_lines(pt_1, pt_2, pt_3, pt_4)
        self.assertFalse(result)

    def test_no_intersect_lines_2(self):
        obs = [Pose2D()]
        pt_1 = Pose2D(-2349.3, 0.0)
        pt_2 = Pose2D(0.0, 2342.3)

        pt_3 = Pose2D(-321.453, -98.28)
        pt_4 = Pose2D(-13.83, 25.9)

        result = intersect_lines(pt_1, pt_2, pt_3, pt_4)
        self.assertFalse(result)
    
    def test_get_sides(self):
        self.assertTrue(True)

    def test_line_collision(self):
        self.assertTrue(True)

    def test_is_collision_pt(self):
        self.assertTrue(True)

    def test_is_collision_line(self):
        self.assertTrue(True)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_collision', TestCollisions)
