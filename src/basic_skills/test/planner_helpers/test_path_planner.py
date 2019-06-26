#!/usr/bin/env  python
import unittest

class TestPathPlanner(unittest.TestCase):
    def test_update_obs(self):
        self.assertTrue(True)

    def test_get_max_dist(self):
        self.assertTrue(True)

    def test_graph_empty(self):
        self.assertTrue(True)

    def test_sample(self):
        self.assertTrue(True)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_path_planner', TestPathPlanner)
