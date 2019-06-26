#!/usr/bin/env  python
from basic_skills.player_data import PlayerData, Pose2D

HORZ_BOUNDS = [-4500, -2000, 0, 2000, 4500]
VERT_BOUNDS = [-3000, -2000, -1000, 1000, 2000, 3000]

def get_zone(location):
    return get_zone_horz(location.x) + get_zone_vert(location.y)

def is_between(value, low, high):
    return (value <= high) and (value >= low)

def get_zone_horz(value):
    if is_between(value, HORZ_BOUNDS[0], HORZ_BOUNDS[1]):
        return 0
    elif is_between(value, HORZ_BOUNDS[1], HORZ_BOUNDS[2]):
        return 5
    elif is_between(value, HORZ_BOUNDS[2], HORZ_BOUNDS[3]):
        return 10
    else:
        return 15

def get_zone_vert(value):
    if is_between(value, VERT_BOUNDS[0], VERT_BOUNDS[1]):
        return 4
    elif is_between(value, VERT_BOUNDS[1], VERT_BOUNDS[2]):
        return 3
    elif is_between(value, VERT_BOUNDS[2], VERT_BOUNDS[3]):
        return 2
    elif is_between(value, VERT_BOUNDS[3], VERT_BOUNDS[4]):
        return 1
    else:
        return 0

def get_zone_boundaries(zone):
    x_idx = zone/5
    y_idx = zone%5
    
    return HORZ_BOUNDS[x_idx], HORZ_BOUNDS[x_idx+1], VERT_BOUNDS[4-y_idx], VERT_BOUNDS[5-y_idx]
