#!/usr/bin/env  python
#from path_planning.path_planner import Point
from geometry_msgs.msg import Pose, Twist
import tf.transformations as tr
import numpy as np

class PlayerData(object):
    def __init__(self, id_num=0):
        self.id_num = id_num
        self.location = Pose2D()
        self.velocities = Pose2D()

    def update(self, pose, twist):
        tmp_array = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        theta = tr.euler_from_quaternion(tmp_array)[2]
        self.location.x = pose.position.x
        self.location.y = pose.position.y
        #print 'theta: {0}'.format(theta)
        self.location.theta = theta

        self.velocities.x = twist.linear.x
        self.velocities.y = twist.linear.y
        self.velocities.theta = twist.angular.z

    def __eq__(self, other):
        return (self.id_num == other.id_num) and (self.location == other.location) and (self.velocities == other.velocities)

    def __ne__(self, other):
        return not (self == other)

    def __str__(self):
        return 'Player Data:\nID: {0}\nPose: {1}\nVelocity: {2}'.format(self.id_num, self.location, self.velocities)

    def __repr__(self):
        return str(self)

class Pose2D(object):
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    #def __eq__(self, other):
        #return (self.x == other.x) and (self.y == other.y) and (self.theta == other.theta)

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y)

    def __ne__(self, other):
        return not (self == other)

    def __str__(self):
        return 'x: {0}, y: {1}, theta: {2}\n'.format(self.x, self.y, self.theta)

    def __repr__(self):
        return str(self)

    #def convert_to_point(self):
        #return Point(self.x, self.y)
    
    def convert_to_array(self):
        return np.array([self.x, self.y])

    def get_distance(self, other):
        return np.linalg.norm(self.convert_to_array() - other.convert_to_array())

    def get_angle_diff(self, other):
        return self.theta - other.theta
