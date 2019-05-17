#!/usr/bin/env python

class Robot(object):
    def __init__(self, pos, vel, id_num):
        self.pos = pos
        self.vel = vel
        self.id_num = id_num

    def update_vals(self, newPos, newVel):
        if newPos is not None:
            self.pos = newPos
        if newVel is not None:
            self.vel = newVel
