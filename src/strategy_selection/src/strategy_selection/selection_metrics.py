#!/usr/bin/env python
class SelectionMetrics(object):
    NUM_ZONES = 20
    NUM_ROBOTS = 6

    def __init__(self):
        self.pass_zone = [0 for i in range(SelectionMetrics.NUM_ZONES)]
        self.shot_zone = [0 for i in range(SelectionMetrics.NUM_ZONES)]
        self.ball_zone = [0 for i in range(SelectionMetrics.NUM_ZONES)]
        self.friend_zone = [[0 for i in range(SelectionMetrics.NUM_ZONES)] for j in range(SelectionMetrics.NUM_ROBOTS)]
        self.enemy_zone =   [[0 for i in range(SelectionMetrics.NUM_ZONES)] for j in range(SelectionMetrics.NUM_ROBOTS)]      
        self.our_score = 0
        self.their_score = 0
