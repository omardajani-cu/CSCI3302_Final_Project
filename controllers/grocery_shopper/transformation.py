"""
Name: transformation.py
Description: helper runctions to translate between world, robot, and map coordinates
"""

import math
import config

def robot_to_world(rx, ry):
    wx =  math.cos(config.pose_theta)*rx - math.sin(config.pose_theta)*ry  + config.pose_x
    wy =  +(math.sin(config.pose_theta)*rx + math.cos(config.pose_theta)*ry) + config.pose_y
    return wx, wy

def world_to_map(pt_x, pt_y):
    x = config.MAP_DIM[0] - int((config.WORLD_DIM[1]/2 - pt_y)*10)
    y = config.MAP_DIM[1] - int((config.WORLD_DIM[0]/2 + pt_x)*10)

    if x >= config.MAP_DIM[0]:
        x = config.MAP_DIM[0] - 1
    if y >= config.MAP_DIM[1]:
        y = config.MAP_DIM[1] - 1
    return x,y

def map_to_world(pt_x, pt_y):
    x = (config.MAP_DIM[1] - pt_y)/10.0 - config.WORLD_DIM[0]/2.0
    y = config.WORLD_DIM[1]/2.0 - (config.MAP_DIM[0] - pt_x)/10.0
    return x,y

def map_to_display(pt_x, pt_y):
    x = pt_x
    y = 360-pt_y
    return x,y

