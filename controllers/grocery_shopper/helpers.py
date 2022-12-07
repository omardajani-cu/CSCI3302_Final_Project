"""
Name: helpers.py
Description: helper functions to aid in execution
"""

import config
import math

# sets wheels to zero for a set amount of time
def wait(time):
  config.robot_parts["wheel_left_joint"].setVelocity(0)
  config.robot_parts["wheel_right_joint"].setVelocity(0)
  for _ in range(time):
    config.robot.step(config.timestep)

# used to update x,y, theta position from GPS
def get_gps_update():
    config.pose_x = -config.gps.getValues()[0]
    config.pose_y = -config.gps.getValues()[1]
    n = config.compass.getValues()
    config.pose_theta = ((math.atan2(n[0], n[1])))


# The functions below are not used, but were designed initially to navigate the facility using hardcoding
def turn180():
  config.robot_parts["wheel_left_joint"].setVelocity(-config.MAX_SPEED/4)
  config.robot_parts["wheel_right_joint"].setVelocity(config.MAX_SPEED/4)
  for _ in range(360):
    config.robot.step(config.timestep)

def turn90():
  config.robot_parts["wheel_left_joint"].setVelocity(-config.MAX_SPEED/4)
  config.robot_parts["wheel_right_joint"].setVelocity(config.MAX_SPEED/4)
  for _ in range(170):
    config.robot.step(config.timestep)

def traverseRow():
  config.robot_parts["wheel_left_joint"].setVelocity(config.MAX_SPEED/4)
  config.robot_parts["wheel_right_joint"].setVelocity(config.MAX_SPEED/4)
  for _ in range(200*20):
    config.robot.step(config.timestep)



