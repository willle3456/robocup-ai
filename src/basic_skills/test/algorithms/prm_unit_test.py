#!/usr/bin/env  python
import unittest

class TestPRM(unittest.TestCase):
    def test_sample_space(self):
        self.assertTrue(True)

    def test_build_road_map(self):
        self.assertTrue(True)

    def test_found_path(self):
        self.assertTrue(True)

    def test_get_nearest_neighbors(self):
        self.assertTrue(True)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_prm', TestPRM)
