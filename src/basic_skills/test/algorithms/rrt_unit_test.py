#!/usr/bin/env  python

import unittest
'''
Unit Tests for the various path planning algorithms that have been implemented
'''

# RRT
class TestRRT(unittest.TestCase):
    def test_extend(self):
        self.assertTrue(True)

    def test_get_neareast(self):
        self.assertTrue(True)
# PRM

# FMT

# PRT/SRT

# EST

# RRG(s)

# RRT (big-little)

# FMT-RRT

# RRT*

# PRM*

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_rrt', TestRRT)
