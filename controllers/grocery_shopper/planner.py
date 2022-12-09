import config
import numpy as np
import matplotlib.pyplot as plt
import transformation
from scipy.signal import convolve2d
import math
import random

#RRT* adapted from HW 2 on RRT as well as pseudo code for RRT* found here: https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378
# and here: http://paper.ijcsns.org/07_book/201610/20161004.pdf
class Node:
    """
    Node for RRT Algorithm. This is what you'll make your graph with!
    """
    def __init__(self, pt, cst, parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.cost = cst # List of points along the way from the parent node (for edge's collision checking)

def state_is_valid(map, point):
    '''
    Function checks if a given point is valid
    :param map the map of the world
    :param point the point whose validity to check
    :return: bool corresponding to whether point is valid
    '''
    return not map[int(point[0])][int(point[1])]

def get_random_valid_vertex(map, state_is_valid):
    '''
    Function that samples a randompoint which is valid (i.e. collision free and within the bounds)
    :param map: the map of the world
    :param state_is_valid: Function that takes a point and checks if it is valid
    :return: point/state
    '''
    vertex = None
    while vertex is None: # Get starting vertex
        x = np.random.randint(config.MAP_DIM[0])
        y = np.random.randint(config.MAP_DIM[1])
        pt = (x, y)
        if state_is_valid(map, pt):
            vertex = pt
    return vertex

def get_nearest_vertex(node_list, q_point):
    '''
    Function that finds a node in node_list with closest node.point to query q_point
    :param node_list: List of Node objects
    :param q_point: array representing a point
    :return Node in node_list with closest node.point to query q_point
    '''
    #initialize distance to infinity
    dist = float("inf")
    for node in node_list:
        #if node is closer than current distance, it is the new node we want to return
        if math.dist(node.point, q_point) < dist:
            dist = math.dist(node.point, q_point)
            point = node
    return point

def steer(from_point, to_point, delta_q):
    '''
    :param from_point: point where the path to "to_point" is originating from (e.g., [1.,2.])
    :param to_point: point indicating destination (e.g., [0., 0.])
    :param delta_q: Max path-length to cover, possibly resulting in changes to "to_point" (e.g., 0.2)
    :return path: Array of points leading from "from_point" to "to_point" (inclusive of endpoints)  (e.g., [ [1.,2.], [1., 1.], [0., 0.] ])
    '''
    # TODO: Figure out if you can use "to_point" as-is, or if you need to move it so that it's only delta_q distance away
    new_point = np.array(to_point)
    #if its too far away, change our endppint to be within the given distance
    if math.dist(from_point, to_point) > delta_q:
        #this is how much we need to subtract from each coordinate in our current endpoint to make it exactly 
        change = (to_point - from_point)*(delta_q/math.dist(from_point, to_point)) 
        for i in range(len(from_point)):
            new_point[i] = from_point[i] + change[i]
    # TODO Use the np.linspace function to get 10 points along the path from "from_point" to "to_point"
    new_point = np.array(new_point)
    path = np.linspace(from_point, new_point, num=10)
    return path

def check_path_valid(map, path, state_is_valid):
    '''
    Function that checks if a path (or edge that is made up of waypoints) is collision free or not
    :param path: A 1D array containing a few (10 in our case) points along an edge
    :param state_is_valid: Function that takes a point and checks if it is valid
    :return: Boolean based on whether the path is collision free or not
    '''
    for point in path:
        point = np.array(point)
        if not state_is_valid(map, point):
            return False
    return True

def rewire(node_list, new_node, rad):
    '''
    Function that rewires tree so that previously wire nodes closer to tree get wired to new node
    :param node_list: List of Node objects
    :param new_node: node to rewire to
    :param rad: distance within which nodes are considered a neighbor
    :return: rewired node list
    '''
    for node in node_list:
        dist = math.dist(node.point, new_node.point)
        if (dist <= rad):
            if (new_node.cost + dist < node.cost):
                node.cost = new_node.cost + dist
                node.parent = new_node
    return node_list

def rrt_star(map, start, end):
    '''
    Function that creates a map with RRT*
    :param map: the map of the world
    :param start: starting point
    :param edn: ending point
    :return: rewired node list
    '''
    radius = 5
    delta_q = 2
    start_node = Node(start, 0)
    node_list = [start_node]
    has_goal = False
    goal_node = None

    while not has_goal:
        goal_rand = random.random()
        #picks either the goal point or a random valid point in our space 
        if goal_rand < 0.05:
            point_to_add = end
        else:
            point_to_add = get_random_valid_vertex(map, state_is_valid)

        point_to_add = np.array(point_to_add)
        #finds closest point to our selected point currently in tree
        pt_closest = np.array(get_nearest_vertex(node_list, point_to_add).point)
        #gets a path from that closest point to our point
        parent_path = steer(pt_closest, point_to_add, delta_q)
        #sets our point to be added to the last point in the path (in case it got changed for being too far away)
        point_to_add = parent_path[len(parent_path) - 1]

        #if the path doesn't cross an obstacle, add a node of our point to the list, with its parent being the closest point in the tree already
        if check_path_valid(map, parent_path, state_is_valid):
            parent_node = None
            for node in node_list:
                if np.array_equal(node.point, pt_closest):
                    parent_node = node
                    break
            cost = math.dist(pt_closest, point_to_add) + parent_node.cost
            node_to_add = Node(point_to_add, cost, parent=parent_node)
            node_list = rewire(node_list, node_to_add, radius)
            node_list.append(node_to_add)
            if (math.dist(point_to_add, end) <= 1e-5):
                goal_node = node_to_add
                has_goal = True

    path = []
    curr_node = goal_node
    while (curr_node != None):
        path = curr_node.point + path
        curr_node = curr_node.parent

    return path
    
    


# Adapted pseudocode from https://en.wikipedia.org/wiki/A*_search_algorithm
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
    convolution = np.full((config.C_SPACE_DIM,config.C_SPACE_DIM), 1)
    convolved_m = convolve2d(m, convolution, mode="same")
    convolved_m = convolved_m > 0
    convolved_m = np.multiply(convolved_m, 1)
    # plt.imshow(np.rot90(convolved_m))
    # plt.show()

def plan_path():
    # Part 2.3 continuation: Call path_planner
    # print(convolved_m)
    ## plan path from start --> end
    # map_start = transformation.world_to_map(config.start[0], config.start[1])
    # map_end = transformation.world_to_map(config.end[0], config.end[1])
    world_waypoints = []
    for i in range(len(config.CHECKPOINTS) - 1):
        map_start = transformation.world_to_map(config.CHECKPOINTS[i][0], config.CHECKPOINTS[i][1])
        map_end = transformation.world_to_map(config.CHECKPOINTS[i+1][0], config.CHECKPOINTS[i+1][1])
        #map_waypoints = a_star(convolved_m, map_start, map_end)
        map_waypoints = rrt_star(convolved_m, map_start, map_end)
        
        # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it    
        for m in map_waypoints:
            wx, wy = transformation.map_to_world(m[0], m[1])
            world_waypoints.append((wx,wy))
        
    np.save("path.npy", world_waypoints)
        
    # for m in map_waypoints:
    #     convolved_m[m] = 3
    # plt.imshow(np.rot90(convolved_m))
    # plt.show()