"""
Name: grocery_shopper.py
Description: This file contains the main thread of execution.
"""

import config
import mapping
import manipulator
import controls
import planner
import detection
import helpers

# Initialization
print("=== Initializing Grocery Shopper...")
config.init()




#mode = 'manual' # Part 1.1: manual mode
#mode = 'planner'
mode = 'autonomous'

planner.init_configuration_space()
planner.plan_path()
controls.init_autonomous_controller()
manipulator.goto_position(config.DEFAULT_MANIPULATOR_POSITION)
# helpers.wait(100)
# manipulator.goto_position([-0.2,0.5,0.7])

# config.stopping_condition = 2
# Main Loop
while config.robot.step(config.timestep) != -1:
    if config.stopping_condition == 0:
        detection.detect_object()

        config.robot_parts["wheel_left_joint"].setVelocity(config.vL)
        config.robot_parts["wheel_right_joint"].setVelocity(config.vR)

        # manipulator.gripper()
        mapping.manual_mapper()
    
        # controller
        if mode == 'manual':
            controls.manual_controller()
        if mode == 'autonomous':
            controls.ik_controller()

    elif config.stopping_condition == 1:
        manipulator.perform_ik_from_cam()
        config.stopping_condition = 2

    elif config.stopping_condition == 2:
        config.robot_parts["wheel_left_joint"].setVelocity(0)
        config.robot_parts["wheel_right_joint"].setVelocity(0)
            
print("Finished Execution")