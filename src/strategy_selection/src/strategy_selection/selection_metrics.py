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

    def __str__(self):
        return 'Score: Us: {0} vs. Them: {1}\nPass Zones: {2}\nShot Zones: {3}\nBall Zones: {4}\nTeam Zones: {5}\nOpp Zones: {6}\n'.format(self.our_score, self.their_score, self.pass_zone, self.shot_zone, self.ball_zone, self.friend_zone, self.enemy_zone)
