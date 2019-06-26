#!/usr/bin/env  python
import unittest

class TestFMT(unittest.TestCase):

    def test_get_cost(self):
        self.assertTrue(True)

    def test_lowest_cost(self):
        self.assertTrue(True)

    def tes_get_neighbors_in_range(self):
        self.assertTrue(True)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_fmt', TestFMT)
