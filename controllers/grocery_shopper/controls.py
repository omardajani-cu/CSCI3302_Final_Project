import config
import numpy as np
import math

state = 0 # use this to iterate through your path
CONTROLLER_GAINS = [3, 10]
DISTANCE_BOUNDS = 0.3 # 0.3 # if robot is within this distance (in m) then move to next waypoint

def init_autonomous_controller():
    # Part 3.1: Load path from disk and visualize it    
    path = np.load("path.npy").tolist()
    config.waypoints = path
    print("PATH = " + str(path))

def ik_controller():
    global state
    vL, vR = config.vL, config.vR
    # Part 3.2: Feedback controller
    #STEP 1: Calculate the error
    rho = math.sqrt(math.pow(config.pose_x-config.waypoints[state][0],2) + math.pow(config.pose_y-config.waypoints[state][1],2))
    alpha = (math.atan2(config.pose_y-config.waypoints[state][1],config.pose_x-config.waypoints[state][0]) - config.pose_theta)
    
    #STEP 2: Controller
    d_x = CONTROLLER_GAINS[0]*rho
    d_theta = CONTROLLER_GAINS[1]*alpha
    
    #STEP 3: Compute wheelspeeds
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
    # print("vl, vr = %f, %f" % (config.vL, config.vR) )
    # Actuator commands
    if rho < DISTANCE_BOUNDS:
        print("------------------ Hit Waypoint ---------------")
        # config.robot_parts[config.MOTOR_LEFT].setVelocity(0)
        # config.robot_parts[config.MOTOR_RIGHT].setVelocity(0)
        
        state += 1
        if state >= len(config.waypoints) - 1:
            print("++++++++++++++No more waypoints+++++++++++++++++++")
            config.robot_parts["wheel_left_joint"].setVelocity(0)
            config.robot_parts["wheel_right_joint"].setVelocity(0)
            config.stopping_condition = 1 


def manual_controller():
        ###################
    #
    # Controller
    #
    ###################
    new_max = config.MAX_SPEED
    key = config.keyboard.getKey()
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
    elif key == ord('S'):
        # Part 1.4: Filter map and save to filesystem          
        np.save("map.npy",config.probability_map)
        print("Map file saved")
    elif key == ord('L'):
        # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
        config.map = np.load("map.npy")
        print("Map loaded")
    else: # slow down
        config.vL *= 0.75
        config.vR *= 0.75

