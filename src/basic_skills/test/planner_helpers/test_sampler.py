#!/usr/bin/env  python
import unittest
class TestSampler(unittest.TestCase):
    def test_uniform(self):
        self.assertTrue(True)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_sample', TestSampler)
