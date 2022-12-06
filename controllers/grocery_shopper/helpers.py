import config
import math

def wait(time):
  config.robot_parts["wheel_left_joint"].setVelocity(0)
  config.robot_parts["wheel_right_joint"].setVelocity(0)
  for i in range(time):
    config.robot.step(config.timestep)

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

def get_gps_update():
    config.pose_x = -config.gps.getValues()[0]
    config.pose_y = -config.gps.getValues()[1]
    n = config.compass.getValues()
    config.pose_theta = ((math.atan2(n[0], n[1])))

    # print("(%f, %f, %f)" % (config.pose_x, config.pose_y, config.pose_theta))
