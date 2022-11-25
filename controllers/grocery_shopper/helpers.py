import config

def wait(time):
  config.robot_parts["wheel_left_joint"].setVelocity(0)
  config.robot_parts["wheel_right_joint"].setVelocity(0)
  for i in range(time):
    config.robot.step(config.timestep)