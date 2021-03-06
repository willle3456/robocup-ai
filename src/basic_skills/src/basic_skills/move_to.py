import sys

from pysim.PySim import *

from basic_skills.action import *
from basic_skills.helper_functions import *


import numpy as np

# make large numbers small and big numbers small.
# we use this to freak out more when we are close to things
# range of outputs is [10-0)
def squash_inverse(num):
  '''
  params - num: number to squash_inverse
  returns - squashed: squashed number
  '''
  return 10000/(num + 1000)


'''
Simple Proportional Differential that controls a robot to a target location 
There is no I term
'''
class MoveTo(Action):
  def __init__(self, game, target_loc = np.array([0,0]), target_rot = False):
    Action.__init__(self, game)
    self.target_loc = target_loc
    self.target_rot = target_rot

    # translational control parameters
    self.translational_control_speed = 150
    self.locP = 34.3
    self.locD = 15

    # rotational control parameters
    self.rot_control_speed = 100
    self.rotP = 100
    self.rotD = 50
    
    self.short_planning_distance = 1000
    self.safety_distance = 300
    
  def set_target(self, target_loc, target_rot):
    '''
    make target loc valid and 
    set self.target_loc 
    and self.target_rot
    '''
    self.target_loc = put_in_bounds(target_loc)
    self.target_rot = target_rot
    
  def collision_avoidance(self, planned_target):
    lvec = planned_target - self._robot.loc
    lmag2 = np.linalg.norm(lvec)**2
    my_velocity = np.linalg.norm(self._robot.velocity)
    
    for b in self.game.blue_robots:
      if self._robot.is_blue and b.id == self._robot.id:
        continue
      closest_point = drop_perpendicular(b.loc, self._robot.loc, lvec)
      local_distance = np.sum(closest_point * lvec)
      if np.linalg.norm(closest_point - b.loc) < self.safety_distance and local_distance > 0 and local_distance < lmag2:
        push_out_factor = squash_inverse(np.linalg.norm(closest_point - self._robot.loc))
        return (1+push_out_factor)*closest_point - push_out_factor * b.loc
        
    for b in self.game.yellow_robots:
      if not self._robot.is_blue and b.id == self._robot.id:
        continue
      closest_point = drop_perpendicular(b.loc, self._robot.loc, lvec)
      local_distance = np.sum(closest_point * lvec)
      if np.linalg.norm(closest_point - b.loc) < self.safety_distance and local_distance > 0 and local_distance < lmag2:
        push_out_factor = squash_inverse(np.linalg.norm(closest_point - self._robot.loc))
        return (1+push_out_factor)*closest_point - push_out_factor * b.loc
    
    return planned_target
    
  def run(self, delta_time):
    
    target_vec = self.target_loc - self._robot.loc
    if np.linalg.norm(target_vec) < self.short_planning_distance * 2:
      planned_target = self.target_loc
    else:
      planned_target = self._robot.loc + target_vec * self.short_planning_distance / np.linalg.norm(target_vec)
      planned_target = put_in_bounds(planned_target)
    planned_target = self.collision_avoidance(planned_target)
  
    self.norm_vel, self.tang_vel = self.PID_loc(planned_target)
    self.rot_vel = self.PID_rot()
    return Action.run(self, delta_time)
    
  def PID_loc(self, planned_target):
    vector = planned_target - self._robot.loc
    distance = np.linalg.norm(vector)
    local = convert_local(vector, -self._robot.rot)
    
    local_vel = convert_local(self._robot.velocity, -self._robot.rot)
    loc_PID = self.locP*local - self.locD*local_vel
    pidMag = np.linalg.norm(loc_PID)
    if pidMag > self.translational_control_speed:
      loc_PID *= self.translational_control_speed/pidMag

    return loc_PID[0], loc_PID[1]#, rot_vel
  def PID_rot(self):
    distance = min_angle(self.target_rot - self._robot.rot)
    PID = self.rotP*distance - self.rotD*self._robot.rot_vel
    if (PID > self.rot_control_speed):
      PID = self.rot_control_speed
    elif(PID < -self.rot_control_speed):
      PID = -self.rot_control_speed

    return PID
  def done(self):
    epsilon = 15
    if np.linalg.norm(self.target_loc - self._robot.loc) < epsilon and np.linalg.norm(self._robot.velocity) < epsilon and abs(normalize_angle(self._robot.rot - self.target_rot)) < 0.1:
      return True
    return False
