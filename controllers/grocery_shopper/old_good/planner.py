"""
Name: planner.py
Description: uses algorithm to generate waypoints between current position and desired position using the map of obstacles
"""

import config
import numpy as np
import matplotlib.pyplot as plt
import transformation
from scipy.signal import convolve2d
import helpers


def h_score(n, goal):
    distance = np.linalg.norm(np.array(n) - np.array(goal))
    return distance
            
def get_neighbor_list(map, curr_point):
    x = curr_point[0]
    y = curr_point[1]
    
    neighbors_list = []
    if x - 1 >= 0 and map[(x-1,y)] != 1:
        neighbors_list.append((x-1, y))
    if x + 1 < config.MAP_DIM[0] and map[(x+1,y)] != 1:
        neighbors_list.append((x+1,y))
    if y - 1 >= 0 and map[(x,y-1)] != 1:
        neighbors_list.append((x, y-1))
    if y + 1 < config.MAP_DIM[1] and map[(x,y+1)] != 1:
        neighbors_list.append((x,y+1))
        
    # adding diagonal neighbors
    if x-1 >= 0 and y-1 >=0 and map[(x-1,y-1)] != 1:
        neighbors_list.append((x-1,y-1))     
    if x-1 >=0 and y+1 < config.MAP_DIM[1] and map[(x-1,y+1)] != 1:
        neighbors_list.append((x-1,y+1)) 
    if x+1 < config.MAP_DIM[0] and y + 1 < config.MAP_DIM[1] and map[(x+1,y+1)] != 1:
        neighbors_list.append((x+1,y+1)) 
    if x+1 < config.MAP_DIM[0] and y-1 >=0 and map[(x+1,y-1)] != 1:
        neighbors_list.append((x+1,y-1)) 
    return neighbors_list
  
def find_lowest_f_score(open_list, f_scores_points):
    low_f_score = np.inf
    curr_low_point = open_list[0]
    
    for point in open_list:
        curr_f_score = f_scores_points[point]  
        if curr_f_score < low_f_score:
            low_f_score = curr_f_score
            curr_low_point = point
            
    return curr_low_point

def create_path(previous_path, curr_point):
    total_path = [curr_point]
    while curr_point in previous_path.keys():
        curr_point = previous_path[curr_point]
        total_path.insert(0, curr_point) 
    return total_path 

def a_star(map, start, end):
    open_list = [start]
    
    f_scores = np.ones(config.MAP_DIM)*np.inf
    g_scores = np.ones(config.MAP_DIM)*np.inf
    shortest_path = {}
    
    g_scores[start] = 0
    f_scores[start] = h_score(start, end)
    
    curr_point = start 
    while(len(open_list) > 0):
        curr_point = find_lowest_f_score(open_list, f_scores)
        if curr_point == end:
            print("Found path")
            return create_path(shortest_path, curr_point)
            
        open_list.remove(curr_point)
        
        neighbors = get_neighbor_list(map, curr_point) 
        for n in neighbors:
            g_score_curr = g_scores[curr_point] + 1
            if g_score_curr < g_scores[n]:
                shortest_path[n] = curr_point
                g_scores[n] = g_score_curr
                f_scores[n] = g_score_curr + h_score(n, end)
                if n not in open_list:
                    open_list.append(n)
                    
    print("Could not find path")
    exit(0)

def init_configuration_space():
    global convolved_m
    m = np.load("map.npy")
    new_map = m > 0.5
    new_map = np.multiply(new_map, 1)

    # plt.imshow(np.rot90(m))
    # plt.show()

    # convolve map to generate configuration space by increasing boundary pixels
    convolution = np.full((config.C_SPACE_DIM,config.C_SPACE_DIM), 1)
    convolved_m = convolve2d(m, convolution, mode="same")
    convolved_m = convolved_m > 0
    convolved_m = np.multiply(convolved_m, 1)

    # plt.imshow(np.rot90(convolved_m))
    # plt.show()

def plan_path():
    world_waypoints = []

    # use current gps value
    helpers.get_gps_update()
    map_start = transformation.world_to_map(config.pose_x, config.pose_y)

    # translate world coordinates to map coordinates
    map_end = transformation.world_to_map(config.CHECKPOINTS[config.checkpoint_idx+1][0], config.CHECKPOINTS[config.checkpoint_idx+1][1])
    map_waypoints = a_star(convolved_m, map_start, map_end)

    # Turn paths into waypoints and save on disk as path.npy and visualize it    
    for m in map_waypoints:
        wx, wy = transformation.map_to_world(m[0], m[1])
        world_waypoints.append((wx,wy))
        
    # print(world_waypoints)
    np.save("path.npy", world_waypoints)