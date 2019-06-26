#!/usr/bin/env  python

import unittest
import numpy as np
from basic_skills.player_data import Pose2D

# Pose2D Tests
class TestPose2D(unittest.TestCase):
    def test_eq(self):
        p1 = Pose2D(1.0, 2.3, 45.0)
        p2 = Pose2D(1.0, 2.3, 45.0)

        self.assertEqual(p1, p2)

    # note that equality is just x & y for now, might need to change to orientation in the future
    def test_ne(self):
        p1 = Pose2D(1.0, 2.3, 45.0)
        p2 = Pose2D(1.0, 2.4, 46.0)

        self.assertFalse(p1 == p2)

    def test_str(self):
        p1 = Pose2D(1.0, 2.0, 3.0)
        tmp_str = str(p1)
        expected_str = 'x: 1.0, y: 2.0, theta: 3.0\n'
        
        self.assertEqual(tmp_str, expected_str)

    def test_repr(self):
        p_list = [Pose2D(), Pose2D(1.0, 1.0, 1.0)]
        tmp_str = str(p_list)
        expected_str = '[x: 0.0, y: 0.0, theta: 0.0\n, x: 1.0, y: 1.0, theta: 1.0\n]'

        self.assertEqual(tmp_str, expected_str)

    def test_convert_to_array(self):
        p1 = Pose2D(3.4, 5.6, 7.8)
        tmp_arr = p1.convert_to_array()
        expected_arr = np.array([3.4, 5.6])
        
        result = tmp_arr == expected_arr
        self.assertTrue(result.all())

    def test_get_distance(self):
        p1 = Pose2D()
        p2 = Pose2D(2.0, 4.0, 90.0)
        tmp_dist = p1.get_distance(p2)
        expected_dist = np.sqrt(20.0)

        self.assertEqual(tmp_dist, expected_dist)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_pose', TestPose2D)
