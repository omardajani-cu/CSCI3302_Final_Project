"""grocery controller."""

# Nov 2, 2022

from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d
import config
import mapping
import manipulator
import controls
import planner

#Initialization
print("=== Initializing Grocery Shopper...")
config.init()

#mode = 'manual' # Part 1.1: manual mode
#mode = 'planner'
mode = 'autonomous'

# planner.plan_path()
controls.init_autonomous_controller()
    
# Main Loop

while config.robot.step(config.timestep) != -1 and config.stopping_condition == 0:

    config.robot_parts["wheel_left_joint"].setVelocity(config.vL)
    config.robot_parts["wheel_right_joint"].setVelocity(config.vR)

    manipulator.gripper()
    mapping.manual_mapper()
 
    # controller
    if mode == 'manual':
        controls.manual_controller()
    if mode == 'autonomous':
        controls.ik_controller()

            
print("Finished Execution")