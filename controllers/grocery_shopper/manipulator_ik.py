import config
import ikpy.chain
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import math
import detection
import helpers
from controller import Supervisor
import tempfile
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

def gripper(): 
    if(config.gripper_status=="open"):
        # Close gripper, note that this takes multiple time steps...
        config.robot_parts["gripper_left_finger_joint"].setPosition(0)
        config.robot_parts["gripper_right_finger_joint"].setPosition(0)
        if config.right_gripper_enc.getValue()<=0.005:
            config.gripper_status="closed"
    else:
        # Open gripper
        config.robot_parts["gripper_left_finger_joint"].setPosition(0.045)
        config.robot_parts["gripper_right_finger_joint"].setPosition(0.045)
        if config.left_gripper_enc.getValue()>=0.044:
            config.gripper_status="open" 

def camera_to_base(pt):
    offsets = [1.2,0.3,0.2] # These parameters still need to be tuned
    x = -1*pt[2] + offsets[0]
    y = -1*pt[0] + offsets[1]
    z = pt[1] + offsets[2]
    return [x, y, z]


def init_manipulator():
      # https://gist.github.com/ItsMichal/4a8fcb330d04f2ccba582286344dd9a7
    with open("tiago_urdf.urdf", "w") as file:  
        file.write(config.robot.getUrdf())

    global my_chain
    my_chain = Chain.from_urdf_file("tiago_urdf.urdf", last_link_vector=[0.012365, 0, -0.1741],
                                    base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_joint", "TIAGo front arm"],
                                    active_links_mask=[False, False, False, False, True, True ,True ,True ,True ,True ,True , False ,False ,False ,False])

    print(my_chain.links)
    print(my_chain.forward_kinematics([0]*15))
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
    global DEFAULT_MANIPULATOR_POSITION, BASKET_MANIPULATOR_POSITION
    DEFAULT_MANIPULATOR_POSITION = [1,1,0]

    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
    initpos = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
    my_chain.plot(my_chain.inverse_kinematics([1,1,1], initial_position = initpos), ax)
    matplotlib.pyplot.show()


def move_manipulator(ikResults):  
  for res in range(len(ikResults)):
      if my_chain.links[res].name in config.part_names:
          config.robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
          print("Setting {} to {}".format(my_chain.links[res].name, ikResults[res]))

def goto_position(target_pos):
  initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
  print("INITIAL POSITION = " + str(initial_position))
  ikResults = []
  try:
    ikResults = my_chain.inverse_kinematics(target_pos, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")
  except:
    print("Could not perform IK")
    return

  move_manipulator(ikResults)
  helpers.wait(100)

def perform_ik_from_cam():
  print("Grabbing object")
  goal_pos = detection.detect_object()
  if len(goal_pos) > 0:
    initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]

    print("cam coords =" + str(goal_pos))
    offset_target = camera_to_base(goal_pos) # And here it is translated to robot/IK coordinates
    print("base coords = " + str(offset_target))

    ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")
    move_manipulator(ikResults)
    helpers.wait(100)


