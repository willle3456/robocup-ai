import sys
from basic_skills.action import *
from basic_skills.helper_functions import *

from pysim.PySim_noise import *
import threading
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons

import numpy as np

'''
Simple Proportional Differential that controls a robot to a target location 
There is no I term
'''
class move_to(action):
  def __init__(self, target_loc = np.array([0,0]), target_rot = False):
    action.__init__(self)
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
    
  def run(self):
    
    target_vec = self.target_loc - self.robot.loc
    if np.linalg.norm(target_vec) < self.short_planning_distance * 2:
      planned_target = self.target_loc
    else:
      planned_target = self.robot.loc + target_vec * self.short_planning_distance / np.linalg.norm(target_vec)
      planned_target = put_in_bounds(planned_target)
  
    self.norm_vel, self.tang_vel = self.PID_loc(planned_target)
    self.rot_vel = self.PID_rot()
    return action.run(self)
    
  def PID_loc(self, planned_target):
    '''
    translational PID control loop
    '''
    vector = planned_target - self.robot.loc
    distance = np.linalg.norm(vector)
    local = convert_local(vector, -self.robot.rot)
    
    local_vel = convert_local(self.robot.velocity, -self.robot.rot)
    loc_PID = self.locP*local - self.locD*local_vel
    pidMag = np.linalg.norm(loc_PID)
    if pidMag > self.translational_control_speed:
      loc_PID *= self.translational_control_speed/pidMag

    return loc_PID[0], loc_PID[1]#, rot_vel
    
  def PID_rot(self):
    '''
    rotational PID control loop
    '''
    distance = min_angle(self.target_rot - self.robot.rot)
    PID = self.rotP*distance - self.rotD*self.robot.rot_vel
    if (PID > self.rot_control_speed):
      PID = self.rot_control_speed
    elif(PID < -self.rot_control_speed):
      PID = -self.rot_control_speed

    return PID
    
  def done(self):
    epsilon = 15
    if np.linalg.norm(self.target_loc - self.robot.loc) < epsilon and np.linalg.norm(self.robot.velocity) < epsilon and abs(normalize_angle(self.robot.rot - self.target_rot)) < 0.1:
      return True
    return False