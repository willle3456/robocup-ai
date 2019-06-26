#!/usr/bin/env  python
from basic_skills.player_data import Pose2D

bot_size = 150.0

def point_collision(pt, obs):
    for o in obs:
        if o.get_distance(pt) < (2.5 *  bot_size):
           return True
    return False

def intersect_lines(p1, p2, p3, p4):
    #print p1, p2, p3, p4
    u_num = -1 * (((p1.x - p2.x) * (p1.y - p3.y)) - ((p1.y - p2.y)* (p1.x - p3.x)))
    t_num = (((p1.x - p3.x) * (p3.y - p4.y)) - ((p1.y - p3.y)* (p3.x - p4.x)))
    denom = (((p1.x - p2.x) * (p3.y - p4.y)) - ((p1.y - p2.y)*(p3.x - p4.x)))
    #print 'u_num: {0}, t_num: {1}, denom: {2}'.format(u_num, t_num, denom)
    if denom == 0:
        return False

    u = u_num / float(denom)
    t = t_num / float(denom)

    check_u = (u >= 0 and u <= 1)
    check_t = (t >= 0 and t <= 1)

    return check_u and check_t

def line_collision(pt_1, pt_2, obs):
    for o in obs:
        sides =  get_sides(o)
        for s in sides:
            if  intersect_lines(pt_1, pt_2, s[0], s[1]):
                return True
    return False
           
def get_sides(obs):
    offset =  bot_size
    point_1 = Pose2D(obs.x - offset, obs.y + offset)
    point_2 = Pose2D(obs.x + offset, obs.y + offset)
    point_3 = Pose2D(obs.x + offset, obs.y - offset)
    point_4 = Pose2D(obs.x - offset, obs.y - offset)
    side_1 = [point_1, point_2]
    side_2 = [point_2, point_3]
    side_3 = [point_3, point_4]
    side_4 = [point_4, point_1]
    return side_1, side_2, side_3, side_4

def is_collision(p1, p2, obs):
    if p2 is None:
        return point_collision(p1, obs)
    else:
        return line_collision(p1, p2, obs)
