#!/usr/bin/env  python

import unittest

'''
Unit Tests for each implemented action
'''

# MoveTo Tests
class TestMoveTo(unittest.TestCase):
    def test_get_players_as_obs(self):
        pass

    def test_run(self):
        pass

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_move_to', TestMoveTo)
