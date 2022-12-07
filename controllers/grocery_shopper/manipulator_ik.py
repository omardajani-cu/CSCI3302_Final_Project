"""
Name: config.py
Description: Initializes robot, sensor, mapping states and defines global constants and other global variables
"""

import config
import ikpy.chain
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import math
# import detection
import helpers
import tempfile
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

def camera_to_base(pt):
    offsets = [1.2,0.3,0.2] # These parameters still need to be tuned
    x = -1*pt[2] + offsets[0]
    y = -1*pt[0] + offsets[1]
    z = pt[1] + offsets[2]
    return [x, y, z]


def initManipulator():
      # https://gist.github.com/ItsMichal/4a8fcb330d04f2ccba582286344dd9a7
    with open("tiago_urdf.urdf", "w") as file:  
        file.write(config.robot.getUrdf())

    global my_chain
    my_chain = Chain.from_urdf_file("tiago_urdf.urdf", last_link_vector=[0.012365, 0, -0.1741],
                                    base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_joint", "TIAGo front arm"],
                                    active_links_mask=[False, False, False, False, True, True ,True ,True ,True ,True ,True , False ,False ,False ,False])


    # Initialize the arm motors and encoders.
    global motors
    motors = []
    for link in my_chain.links:
        if link.name in config.part_names and link.name != "torso_lift_joint" and link.name != "gripper_left_finger_joint" and link.name != "gripper_right_finger_joint":
            motor = config.robot.getDevice(link.name)
            if link.name == "torso_lift_joint":
                motor.setVelocity(0.07)
            else:
                motor.setVelocity(1)
                
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(config.timestep)
            motors.append(motor) 

    global INITIAL_MOTOR_CONFIG, INITIAL_POSITION
    INITIAL_MOTOR_CONFIG = [0, 0, 0, 0, 0.06988094039511938, -2.2771102318759435e-05, 2.9080808170488355e-05, 1.270004357222803, 1.3200000000362873, -5.361971225428858e-07, 1.4099999998862853, 0, 0, 0, 0]
    # notes on init pos = [y left/right,z up/down, x forward/bckwrd]
    INITIAL_POSITION = [0.13629112,-0.98848863,0.64669744]
    global DEFAULT_MANIPULATOR_POSITION, BASKET_MOTOR_CONFIG
    DEFAULT_MANIPULATOR_POSITION = [1,1,0]
    BASKET_MOTOR_CONFIG = [0,0,0,0] + [0.253,0.289,1.500,-0.320,1.283,1.390,1.739] + [0,0,0,0]
    # ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
    # initpos = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
    # my_chain.plot(my_chain.inverse_kinematics([1,0,0], initial_position = initpos), ax)
    # matplotlib.pyplot.show()


def move_manipulator(ikResults):  
  for res in range(len(ikResults)):
      if my_chain.links[res].name in config.part_names:
          config.robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])

def goto_position(target_pos):
  initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
  ikResults = []
  try:
    ikResults = my_chain.inverse_kinematics(target_pos, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")

  except:
    print("Could not perform IK")
    return -1

  move_manipulator(ikResults)
  helpers.wait(50)
  return 0

# def perform_ik_from_cam():
#   print("Grabbing object")
#   goal_pos = detection.detect_object()
#   if len(goal_pos) > 0:
#     initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]

#     print("cam coords =" + str(goal_pos))
#     offset_target = camera_to_base(goal_pos) # And here it is translated to robot/IK coordinates
#     print("base coords = " + str(offset_target))

#     ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")
#     move_manipulator(ikResults)
#     helpers.wait(100)


def manualIK():
  # while config.robot.step(config.timestep) != -1:
  initial_motor_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
  initial_cartesian_position = my_chain.forward_kinematics([0]*15)
  initial_cartesian_position = initial_cartesian_position[:3,3]

  key = config.keyboard.getKey()
  while(config.keyboard.getKey() != -1): pass
  # arm joint 1
  if key == ord('1'):
    config.robot_parts["arm_1_joint"].setPosition(initial_motor_position[4] + 0.01)
    pass
  elif key == ord('Q'):
    config.robot_parts["arm_1_joint"].setPosition(initial_motor_position[4] - 0.01)
    pass
  
  # arm joint 2
  elif key == ord('2'):
    config.robot_parts["arm_2_joint"].setPosition(initial_motor_position[5] + 0.01)
    pass
  elif key == ord('W'):
    config.robot_parts["arm_2_joint"].setPosition(initial_motor_position[5] - 0.01)
    pass

  # arm joint 3
  elif key == ord('3'):
    config.robot_parts["arm_3_joint"].setPosition(initial_motor_position[6] + 0.01)
    pass
  elif key == ord('E'):
    config.robot_parts["arm_3_joint"].setPosition(initial_motor_position[6] - 0.01)
    pass

  # arm joint 4
  elif key == ord('4'):
    config.robot_parts["arm_4_joint"].setPosition(initial_motor_position[7] + 0.01)
    pass
  elif key == ord('R'):
    config.robot_parts["arm_4_joint"].setPosition(initial_motor_position[7] - 0.01)
    pass

  # arm joint 5
  elif key == ord('5'):
    config.robot_parts["arm_5_joint"].setPosition(initial_motor_position[8] + 0.01)
    pass
  elif key == ord('T'):
    config.robot_parts["arm_5_joint"].setPosition(initial_motor_position[8] - 0.01)
    pass

  # arm joint 6
  elif key == ord('6'):
    config.robot_parts["arm_6_joint"].setPosition(initial_motor_position[9] + 0.01)
    pass
  elif key == ord('Y'):
    config.robot_parts["arm_6_joint"].setPosition(initial_motor_position[9] - 0.01)
    pass

  # arm joint 7
  elif key == ord('7'):
    config.robot_parts["arm_7_joint"].setPosition(initial_motor_position[10] + 0.01)
    pass
  elif key == ord('U'):
    config.robot_parts["arm_7_joint"].setPosition(initial_motor_position[10] - 0.01)
    pass

  # gripper open
  elif key == ord('8'):
    config.robot_parts["gripper_left_finger_joint"].setPosition(0.045)
    config.robot_parts["gripper_right_finger_joint"].setPosition(0.045)    
    pass

  # gripper glose
  elif key == ord('I'):
    config.robot_parts["gripper_left_finger_joint"].setPosition(0.0)
    config.robot_parts["gripper_right_finger_joint"].setPosition(0.0)
    pass

  # try and move to basket
  elif key == ord('B'):
    move_manipulator(BASKET_MOTOR_CONFIG)

  # try and move to default
  elif key == ord('D'):
    if goto_position([1,1,0]) == -1:
      print("Tried to move to DEFAULT but could not compute IK, please move to a different position and try again or manually move to basket position")
    pass

  # reset to straight arm position
  elif key == ord('A'):
    move_manipulator(INITIAL_MOTOR_CONFIG)
    pass

  # allow user to reposition robot closer to object
  elif key == ord('C'):
    print("Manually repositioning robot")
    config.robot_state = config.State.REPOSITIONING
    pass

  # break from arm controller
  elif key == ord('K'):
    print("Breaking from manipulator control, setting state back to navigating")
    config.robot_state = config.State.NAVIGATING
    pass
