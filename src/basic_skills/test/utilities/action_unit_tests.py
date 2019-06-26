#!/usr/bin/env  python

import unittest
from basic_skills.action import Action
from basic_skills.robot import Robot

# Action Tests
class TestAction(unittest.TestCase):
    def test_run(self):
        test_action = Action()
        test_action.kick = 100.0
        test_action.chip = 20.0
        test_action.norm_vel = -398.21
        test_action.tang_vel = -432.0
        test_action.rot_vel = 12.0

        test_result = test_action.run(0.1)
        expected_result = [100.0, 20.0, -398.21, -432.0, 12.0]

        self.assertEqual(test_result, expected_result)

    def test_add(self):
        test_action = Action()
        test_robot = Robot(1)

        test_action.add(test_robot)

        self.assertEqual(test_action._robot, test_robot)

    def test_get_robot(self):
        test_action = Action()
        test_robot = Robot(0)

        test_action.add(test_robot)

        test_result = test_action.get_robot()

        self.assertEqual(test_result, test_robot)

    #def test_is_finished(self):
        #self.assertTrue(True)

    #def test_set_robot(self):
        #self.assertTrue(True)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_action', TestAction)
