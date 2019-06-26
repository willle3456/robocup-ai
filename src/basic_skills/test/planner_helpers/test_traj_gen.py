#!/usr/bin/env  python
import unittest
from basic_skills.player_data import Pose2D
from basic_skills.path_planning.collision_checker import *
'''
Unit Tests for all sub components of path planning
'''

# Trajectory Generation Tests
class TestTrajGen(unittest.TestCase):
    def test_point_to_point(self):
        self.assertTrue(True)

    def test_time_based(self):
        self.assertTrue(True)

    def test_rotate_coords(self):
        self.assertTrue(True)

    def test_get_traj(self):
        self.assertTrue(True)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_traj_gen', TestTrajGen)
