"""
Name: grocery_shopper.py
Description: This file contains the main thread of execution.
"""

import config
import mapping
import controls
import planner
import helpers
import manipulator_ik
import goal_object_detection
import trilateration
import numpy as np


# Initialization
print("--> Initializing Grocery Shopper <--")
config.init()

if config.robot_state == config.State.START:
    manipulator_ik.initManipulator()
    manipulator_ik.goto_position(manipulator_ik.DEFAULT_MANIPULATOR_POSITION)
    trilateration.enableSLAMLidar()
    
    print("PRESS 1 to begin manually mapping, 2 to use default points")
    while config.robot.step(config.timestep) != -1:
        key = config.keyboard.getKey()
        while(config.keyboard.getKey() != -1): pass
        if key == ord('1'):
            print("SETTING STATE TO MAPPING")
            config.robot_state = config.State.MAPPING
            break
        elif key == ord('2'):
            print("SETTING STATE TO ROUTING")
            mypath = np.load("path.npy").tolist()
            # get rid of duplicate checkpoints
            res = []
            [res.append(x) for x in mypath if x not in res]
            config.CHECKPOINTS = res
            planner.init_configuration_space()
            config.robot_state = config.State.REROUTING
            break

# Main Loop
while config.robot.step(config.timestep) != -1:

    if config.robot_state == config.State.MAPPING:
        # goal object was detected, add checkpoint          
        if goal_object_detection.colorDetection() == 1:
            helpers.get_gps_update()
            config.pts.append([config.pose_x, config.pose_y, config.pose_theta])

        config.robot_parts["wheel_left_joint"].setVelocity(config.vL)
        config.robot_parts["wheel_right_joint"].setVelocity(config.vR)

        controls.manual_controller()
        mapping.lidarMapper()
        trilateration.getTrilaterationUpdate()
        

    elif config.robot_state == config.State.NAVIGATING:
        goal_object_detection.colorDetection()

        config.robot_parts["wheel_left_joint"].setVelocity(config.vL)
        config.robot_parts["wheel_right_joint"].setVelocity(config.vR)

        mapping.lidarMapper()
        controls.ik_controller()

        # need to add ability to reposition at any time
        key = config.keyboard.getKey()
        while(config.keyboard.getKey() != -1): pass
        # arrest control from differential drive IK
        if key == ord('A'):
            print("Changing to manual navigation")
            config.robot_state = config.State.REPOSITIONING
            config.robot_parts["wheel_left_joint"].setVelocity(config.vL)
            config.robot_parts["wheel_right_joint"].setVelocity(config.vR)

    elif config.robot_state == config.State.REROUTING:
        planner.plan_path()
        controls.init_autonomous_controller()

        print("Setting state to arm manipulation")
        config.robot_state = config.State.GRABBING

    elif config.robot_state == config.State.GRABBING:
        manipulator_ik.manualIK()
        pass

    elif config.robot_state == config.State.REPOSITIONING:
        config.robot_parts["wheel_left_joint"].setVelocity(config.vL)
        config.robot_parts["wheel_right_joint"].setVelocity(config.vR)

        controls.manual_controller()
        mapping.lidarMapper()
        pass

    elif config.robot_state == config.State.END:
        config.robot_parts["wheel_left_joint"].setVelocity(0)
        config.robot_parts["wheel_right_joint"].setVelocity(0)
        break

    else:
        print("Invalid state, exiting")
        exit(-1)

print("--> Finished Execution <--")