#!/usr/bin/env  python

import unittest
from basic_skills.player_data import Pose2D, PlayerData
from geometry_msgs.msg import Pose, Twist, Quaternion
from tf.transformations import quaternion_from_euler

# Player Data Tests
class TestPlayerData(unittest.TestCase):
    def test_eq(self):
        p1 = PlayerData()
        p1.id_num = 2
        p1.location = Pose2D(1.0, 2.0, 3.0)
        p1.velocities = Pose2D(0.2, 0.4, 5.0)

        p2 = PlayerData()
        p2.id_num = 2
        p2.location = Pose2D(1.0, 2.0, 3.0)
        p2.velocities = Pose2D(0.2, 0.4, 5.0)

        self.assertEqual(p1, p2) 

    def test_ne(self):
        p1 = PlayerData()
        p1.id_num = 2
        p1.location = Pose2D(1.0, 2.0, 3.0)
        p1.velocities = Pose2D(0.2, 0.4, 5.0)

        p2 = PlayerData()
        p2.id_num = 2
        p2.location = Pose2D(1.1, 2.0, 3.0)
        p2.velocities = Pose2D(0.2, 0.4, 5.0)

        self.assertFalse(p1 == p2)

    #def test_str(self):
        #pass

    #def test_repr(self):
        #pass

    def test_update(self):
        tst_pose = Pose()
        tst_pose.position.x = 1012.0
        tst_pose.position.y = 334.4
        tst_pose.position.z = 0.0
        tst_pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))

        tst_twist = Twist()
        tst_twist.linear.x = 2302.4
        tst_twist.linear.y = 323.21
        tst_twist.linear.z = 0.0
        tst_twist.angular.x = 0.0
        tst_twist.angular.y = 0.0
        tst_twist.angular.z = 1.0

        p1 = PlayerData()
        p1.update(tst_pose, tst_twist)

        expected_data = PlayerData()
        expected_data.location.x = tst_pose.position.x
        expected_data.location.y = tst_pose.position.y
        expected_data.location.theta = 0.0
        expected_data.velocities.x = tst_twist.linear.x
        expected_data.velocities.y = tst_twist.linear.y
        expected_data.velocities.theta = tst_twist.angular.z

        self.assertEqual(p1, expected_data)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('basic_skills', 'test_player_data', TestPlayerData)

