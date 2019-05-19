#!/usr/bin/env  python

class PlayerData(object):
    def __init__(self):
        self.location = Location()
        self.velocities = Velocity()

    def __eq__(self, other):
        return (self.location == other.location) and (self.velocities == other.velocities)

    def __ne__(self, other):
        return not (self == other)

    def __str__(self):
        return 'Player Data:\n{0}\n{1}'.format(self.location, self.velocities)

class Location(object):
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y) and (self.theta == other.theta)

    def __ne__(self, other):
        return not (self == other)

    def __str__(self):
        return 'Location:\nx: {0}, y: {1}, theta: {2}'.format(self.x, self.y, self.theta)

class Velocity(object):
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y) and (self.theta == other.theta)

    def __ne__(self, other):
        return not (self == other)

    def __str__(self):
        return 'Velocity:\nx: {0}, y: {1}, theta: {2}'.format(self.x, self.y, self.theta)
