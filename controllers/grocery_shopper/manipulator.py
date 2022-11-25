import config
import ikpy.chain
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import math
import detection
import helpers

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

def move_manipulator(ikResults):  
  for res in range(len(ikResults)):
      if config.my_chain.links[res].name in config.part_names:
          config.robot.getDevice(config.my_chain.links[res].name).setPosition(ikResults[res])
          print("Setting {} to {}".format(config.my_chain.links[res].name, ikResults[res]))

def goto_position(target_pos):
  initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in config.motors] + [0,0,0,0]
  print(initial_position)
  ikResults = config.my_chain.inverse_kinematics(target_pos, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")
  move_manipulator(ikResults)
  helpers.wait(100)

def perform_ik_from_cam():
  print("Grabbing object")
  goal_pos = detection.detect_object()
  if len(goal_pos) > 0:
    initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in config.motors] + [0,0,0,0]

    print("cam coords =" + str(goal_pos))
    offset_target = camera_to_base(goal_pos) # And here it is translated to robot/IK coordinates
    print("base coords = " + str(offset_target))

    ikResults = config.my_chain.inverse_kinematics(offset_target, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")
    move_manipulator(ikResults)
    helpers.wait(100)



"""
def init_ik():
    global chain
    chain = ikpy.chain.Chain.from_urdf_file("only_arm.urdf", active_links_mask=[False, False, False, True, True, True, True, True, True, True])
    print(chain)
    curr_pos = chain.forward_kinematics([0]*10)
    print("CURR POS = " + str(curr_pos))
    ik_calc = chain.inverse_kinematics([7.17300016e-01,2.13447686e-010,5.90959298e-01])
    ik_calc = ik_calc[2:]

    # for i,joint in enumerate(['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']):
    #     config.robot_parts[joint].setPosition(ik_calc[i])
    #     config.robot_parts[joint].setVelocity(config.robot_parts[joint].getMaxVelocity() / 2.0)



def goto_pos():
    target_pos = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf',0.045,0.045)
    # The Tiago robot has multiple motors, each identified by their names below
    part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
                "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
                "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
                "gripper_left_finger_joint","gripper_right_finger_joint")

    robot_parts={}
    for i, part_name in enumerate(part_names):
        robot_parts[part_name]=config.robot.getDevice(part_name)
        robot_parts[part_name].setPosition(float(target_pos[i]))
        robot_parts[part_name].setVelocity(robot_parts[part_name].getMaxVelocity() / 2.0)


def init_chain():
    arm_chain = Chain(name='arm_chain', links=[
    OriginLink(),
    URDFLink(
      name="shoulder",
      origin_translation=[0.125, 0.018, -0.031],
      origin_orientation=[1, 1.13855e-05, -1.13856e-05],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="elbow",
      origin_translation=[-0.0200773, -0.0270003, -0.222028],
      origin_orientation=[-0.562343, -0.584712, -0.584708],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="wrist",
      origin_translation=[0, 0.016, 0],
      origin_orientation=[0.57735, 0.57735, -0.57735],
      rotation=[0, 1, 0],
    )
    ])
"""
