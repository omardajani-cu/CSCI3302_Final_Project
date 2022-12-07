"""
Name: grocery_shopper.py
Description: This file contains the main thread of execution.
"""

import config
import mapping
import controls
import planner
import helpers
import manipulator
import goal_object_detection
import trilateration
import numpy as np


# Initialization
print("--> Initializing Grocery Shopper <--")
config.init()

# execution is just starting, choose to manually map or use an existing map that was created in a previous thread of execution
if config.robot_state == config.State.START:
    manipulator.initManipulator()
    # use inverse kinematics to make arm goto a position that will not interfere with obstacle lidar detector
    manipulator.goto_position(manipulator.DEFAULT_MANIPULATOR_POSITION)
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

            # load path and set checkpoints to path
            mypath = np.load("checkpoints.npy").tolist()
            # get rid of duplicate checkpoints
            res = []
            [res.append(x) for x in mypath if x not in res]
            config.CHECKPOINTS = res
            planner.init_configuration_space()
            config.robot_state = config.State.REROUTING
            break

# Main Loop
while config.robot.step(config.timestep) != -1:

    # user manually controls robot to map out facility
    if config.robot_state == config.State.MAPPING:
        # goal object was detected over a certain size using color blob detection, then add checkpoint so we can stop there when autonomously navigating          
        if goal_object_detection.colorDetection() == 1:
            helpers.get_gps_update()
            config.pts.append([config.pose_x, config.pose_y, config.pose_theta])

        config.robot_parts["wheel_left_joint"].setVelocity(config.vL)
        config.robot_parts["wheel_right_joint"].setVelocity(config.vR)

        controls.manual_controller()
        # show position from lidar mesasurments and add obstacles to map
        mapping.lidarMapper()
        # show position obtained from trilateration
        trilateration.getTrilaterationUpdate()
        
    # robot is using differential drive IK to autonomously navigate between points
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

    # robot is replanning route from current position and destinaiton
    elif config.robot_state == config.State.REROUTING:
        planner.plan_path()
        controls.init_autonomous_controller()

        print("Setting state to arm manipulation")
        config.robot_state = config.State.GRABBING

    # robot is in stationary position and user can control arm movements
    elif config.robot_state == config.State.GRABBING:
        manipulator.manualIK()
        pass
    
    # allow user to have manual control to reposition the robot
    elif config.robot_state == config.State.REPOSITIONING:
        config.robot_parts["wheel_left_joint"].setVelocity(config.vL)
        config.robot_parts["wheel_right_joint"].setVelocity(config.vR)

        controls.manual_controller()
        mapping.lidarMapper()
        pass

    # no more checkpoints
    elif config.robot_state == config.State.END:
        config.robot_parts["wheel_left_joint"].setVelocity(0)
        config.robot_parts["wheel_right_joint"].setVelocity(0)
        break

    else:
        print("Invalid state, exiting")
        exit(-1)

print("--> Finished Execution <--")