"""
Name: controls.py
Description: Manual and autonomous controller
"""

import config
import numpy as np
import math
import helpers
import planner

state = 0 # iterator index through path
CONTROLLER_GAINS = [3, 10]
DISTANCE_BOUNDS = 0.3 # 0.3 # if robot is within this distance (in m) then move to next waypoint

def init_autonomous_controller():
    # Load path from disk   
    global state
    state = 0
    path = np.load("path.npy").tolist()
    config.waypoints = path


def ik_controller():
    global state
    vL, vR = config.vL, config.vR

    # error terms
    rho = math.sqrt(math.pow(config.pose_x-config.waypoints[state][0],2) + math.pow(config.pose_y-config.waypoints[state][1],2))
    alpha = (math.atan2(config.pose_y-config.waypoints[state][1],config.pose_x-config.waypoints[state][0]) - config.pose_theta)

    # Controller
    d_x = CONTROLLER_GAINS[0]*rho
    d_theta = CONTROLLER_GAINS[1]*alpha
    
    # Compute wheelspeeds
    vL = (d_x - (d_theta*config.AXLE_LENGTH*0.5))
    vR = (d_x + (d_theta*config.AXLE_LENGTH*0.5))

    # Normalize wheelspeed
    slowed_speed = config.MAX_SPEED/4
    # (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)
    if vL == 0 and vR == 0:
        pass
    elif vL == 0:
        vR = math.copysign(slowed_speed, vR)
    elif vR == 0:
        vL = math.copysign(slowed_speed, vL)
    else:
        ratio = abs(vL)/abs(vR)
        if ratio > 1:
            vR = math.copysign(slowed_speed/ratio, vR)
            vL = math.copysign(slowed_speed, vL)
            
        elif ratio < 1:
            vL = math.copysign(slowed_speed*ratio, vL)
            vR = math.copysign(slowed_speed, vR)
            
        else:
            vL = math.copysign(slowed_speed, vL)
            vR = math.copysign(slowed_speed, vR)     

    config.vL = vL
    config.vR = vR

    # once it reaches waypoints, go to next waypoint
    if rho < DISTANCE_BOUNDS:
        state += 1
        if state >= len(config.waypoints) - 1:
            print("++++++++++++++Moving to next checkpoint+++++++++++++++++++")
            config.robot_parts["wheel_left_joint"].setVelocity(0)
            config.robot_parts["wheel_right_joint"].setVelocity(0)
            
            # ran out of checkpoints, end of execution
            if config.checkpoint_idx >= len(config.CHECKPOINTS)-2:
                config.robot_state = config.State.END 

            # moving to next checkpoint
            else:
                config.checkpoint_idx += 1
                config.robot_state = config.State.REROUTING

def manual_controller():
    new_max = config.MAX_SPEED/2
    key = config.keyboard.getKey()
    # print("key " + str(key))
    while(config.keyboard.getKey() != -1): pass
    if key == config.keyboard.LEFT :
        config.vL = -new_max
        config.vR = new_max
    elif key == config.keyboard.RIGHT:
        config.vL = new_max
        config.vR = -new_max
    elif key == config.keyboard.UP:
        config.vL = new_max
        config.vR = new_max
    elif key == config.keyboard.DOWN:
        config.vL = -new_max
        config.vR = -new_max
    elif key == ord(' '):
        config.vL = 0
        config.vR = 0

    elif key == ord('Q') and config.robot_state == config.State.MAPPING:
        helpers.get_gps_update()
        config.pts.append([config.pose_x, config.pose_y, config.pose_theta])
        print("point saved")

    elif key == ord('M'):
        print("Going back to arm manipulation")
        config.robot_state = config.State.GRABBING

    elif key == ord('P'):
        if config.robot_state == config.State.MAPPING:
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            print("Beginning autonomous navigation")
            mypath = config.pts
            # get rid of duplicate checkpoints
            res = []
            [res.append(x) for x in mypath if x not in res]
            config.CHECKPOINTS = res
            np.save("map.npy",config.probability_map)
            planner.init_configuration_space()
            config.robot_state = config.State.REROUTING
        else:
            print("Restarting autonomous navigation")
            planner.init_configuration_space()
            config.robot_state = config.State.REROUTING

    else: # slow down
        config.vL *= 0.75
        config.vR *= 0.75

