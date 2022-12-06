"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12
C_SPACE_DIM = 17
MAP_DIMENSION = 360
STRIDE = 1

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis
map = None
##### ^^^ [End] Do Not Modify ^^^ #####

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
# mode = 'manual' # Part 1.1: manual mode
# mode = 'planner'
mode = 'autonomous'

# Adapted pseudocode from https://en.wikipedia.org/wiki/A*_search_algorithm
def h_score(n, goal):
    distance = np.linalg.norm(np.array(n) - np.array(goal))
    return distance
            
def get_neighbor_list(map, curr_point):
    size = len(map)
    x = curr_point[0]
    y = curr_point[1]
    
    neighbors_list = []
    if x - 1 >= 0 and map[(x-1,y)] != 1:
        neighbors_list.append((x-1, y))
    if x + 1 < 360 and map[(x+1,y)] != 1:
        neighbors_list.append((x+1,y))
    if y - 1 >= 0 and map[(x,y-1)] != 1:
        neighbors_list.append((x, y-1))
    if y + 1 < 360 and map[(x,y+1)] != 1:
        neighbors_list.append((x,y+1))
        
    # adding diagonal neighbors
    if x-1 >= 0 and y-1 >=0 and map[(x-1,y-1)] != 1:
        neighbors_list.append((x-1,y-1))     
    if x-1 >=0 and y+1 < 360 and map[(x-1,y+1)] != 1:
        neighbors_list.append((x-1,y+1)) 
    if x+1 < 360 and y + 1 < 360 and map[(x+1,y+1)] != 1:
        neighbors_list.append((x+1,y+1)) 
    if x+1 < 360 and y-1 >=0 and map[(x+1,y-1)] != 1:
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
    
def map_to_world(pt):
    return (pt[0]/30.0, pt[1]/30.0)

def world_to_map(pt):
    x = int(pt[0]*30)
    y = int(pt[1]*30)
    if x > 359:
        x = 359
    if y > 359:
        y = 359
    return (x,y)

###################
#
# Planner
#
###################
if mode == 'planner':
    # Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    start_w = (8.4357,4.6653) # (Pose_X, Pose_Z) in meters
    end_w = (7.0,10.0) # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_w from the webots coordinate frame into the map frame
    start = world_to_map(start_w) # (x, y) in 360x360 map
    end = world_to_map(end_w) # (x, y) in 360x360 map
    
    print(start)
    print(end)

    # Part 2.3: Implement A* or Dijkstra's Algorithm to find a path
    def path_planner(map, start, end):
        '''
        :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
        :param start: A tuple of indices representing the start cell in the map
        :param end: A tuple of indices representing the end cell in the map
        :return: A list of tuples as a path from the given start to the given end in the given maze
        '''
        open_list = [start]
        
        f_scores = np.ones((MAP_DIMENSION,MAP_DIMENSION))*np.inf
        g_scores = np.ones((MAP_DIMENSION,MAP_DIMENSION))*np.inf
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

    # Part 2.1: Load map (map.npy) from disk and visualize it
    m = np.load("map.npy")
    plt.imshow(np.rot90(m))
    plt.show()
    # Part 2.2: Compute an approximation of the “configuration space”
    convolution = np.full((C_SPACE_DIM,C_SPACE_DIM), 1)
    convolved_m = convolve2d(m, convolution, mode="same")
    convolved_m = convolved_m > 10
    convolved_m = np.multiply(convolved_m, 1)
    plt.imshow(np.rot90(convolved_m))
    plt.show()

    # Part 2.3 continuation: Call path_planner
    map_waypoints = path_planner(convolved_m, start, end)
    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it    
    world_waypoints = []
    for m in map_waypoints:
        world_waypoints.append(map_to_world(m))
    
    np.save("path.npy", world_waypoints)
    
    for m in map_waypoints:
        convolved_m[m] = 3
    plt.imshow(np.rot90(convolved_m))
    plt.show()



######################
#
# Map Initialization
#
######################

# Part 1.2: Map Initialization

# Initialize your map data structure here as a 2D floating point array
map = np.empty((360,360))
probability_map = np.empty((360,360)) 
waypoints = []

if mode == 'autonomous':
    # Part 3.1: Load path from disk and visualize it
    waypoints = [] # Replace with code to load your path
    path = np.load("path.npy").tolist()
    waypoints = path
    print(path)
    print(len(path))
    path_on_map = []
    for p in path:
        p_on_map = (int(p[0]*30), int(p[1]*30))
        path_on_map.append(p_on_map)
        
    display.setColor(0x00FF00)
    for i in range(len(path_on_map)):
        if i > 0:
            display.drawLine(path_on_map[i-1][0],360-path_on_map[i-1][1],path_on_map[i][0],360-path_on_map[i][1])

state = 30 # use this to iterate through your path
CONTROLLER_GAINS = [3, 10]
DISTANCE_BOUNDS = 0.3 # if robot is within this distance (in m) then move to next waypoint
robot_state = 0


while robot.step(timestep) != -1 and mode != 'planner' and robot_state == 0:

    ###################
    #
    # Mapping
    #
    ###################

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_y = -gps.getValues()[1]
    pose_x = -gps.getValues()[0]

    n = compass.getValues()
    rad = ((math.atan2(n[0], -n[2])))#-1.5708)
    pose_theta = rad
    
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
    
    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = -math.cos(alpha)*rho + 0.202
        ry = math.sin(alpha)*rho -0.004


        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  +(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y
         
        ################ ^ [End] Do not modify ^ ##################

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.
            (map_x, map_y) = world_to_map((wx,wy))
            probability_map[map_x, map_y] += 5e-3
            if probability_map[map_x, map_y] > 1:
                probability_map[map_x, map_y] = 1   
                
            # You will eventually REPLACE the following 3 lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            g = probability_map[map_x, map_y]
            color = int((g*256**2 + g*256+g)*255)
            display.setColor(color)
            display.drawPixel(map_x,360-map_y)
            map[map_x, map_y]=1 

    # Draw the robot's current pose on the 360x360 display
    display.setColor(int(0xFF0000))
    
    #print(pose_x,pose_y,pose_theta)
    display.drawPixel(int(pose_x*30),360-int(pose_y*30))



    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            # Part 1.4: Filter map and save to filesystem
            new_map = probability_map > 0.5
            new_map = np.multiply(new_map, 1)            
            np.save("map.npy",new_map)
            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            print("Map loaded")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
            
            
    else: # not manual mode
        # Part 3.2: Feedback controller
        #STEP 1: Calculate the error
        rho = math.sqrt(math.pow(pose_x-waypoints[state][0],2) + math.pow(pose_y-waypoints[state][1],2))
        # alpha = -(math.atan2(waypoints[state][1]-pose_y,waypoints[state][0]-pose_x) + pose_theta_new)
        alpha = (math.atan2(pose_y-waypoints[state][1],pose_x-waypoints[state][0]) - pose_theta)

        print("RHO:" + str(rho))
        print("Alpha: " + str(alpha))
        
        #STEP 2: Controller
        d_x = CONTROLLER_GAINS[0]*rho
        d_theta = CONTROLLER_GAINS[1]*alpha

        print("dx = " + str(d_x))
        print("d_theta_with_calcs =" + str(d_theta*AXLE_LENGTH*0.5))
        
        #STEP 3: Compute wheelspeeds
        vL = (d_x - (d_theta*AXLE_LENGTH*0.5))
        vR = (d_x + (d_theta*AXLE_LENGTH*0.5))
        
        print("Prenormalized vL" + str(vL))
        print("Prenormalized vR" + str(vR))

        # Normalize wheelspeed
        slowed_speed = MAX_SPEED/4
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

        # Actuator commands
        if rho < DISTANCE_BOUNDS:
            print("------------------ Hit Waypoint ---------------")
            robot_parts[MOTOR_LEFT].setVelocity(0)
            robot_parts[MOTOR_RIGHT].setVelocity(0)
            
            state += STRIDE
            if state >= len(waypoints) - STRIDE:
                print("++++++++++++++No more waypoints+++++++++++++++++++")
                robot_state = 1 
        else:
            print("vL=" + str(vL))
            print("vR=" + str(vR))
            # Set robot motors to the desired velocities
            robot_parts[MOTOR_LEFT].setVelocity(vL)
            robot_parts[MOTOR_RIGHT].setVelocity(vR)
            
    # Actuator commands
    # robot_parts[MOTOR_LEFT].setVelocity(vL)
    # robot_parts[MOTOR_RIGHT].setVelocity(vR)
    
    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    #pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    #pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    # pose_theta_new += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0
    ########## Do not change ######################
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    # if pose_theta > 6.28+3.14/2: pose_theta_new -= 6.28
    # if pose_theta < -3.14: pose_theta_new += 6.28
    ###############################################
    print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))
    print("Waypoint: " + str(waypoints[state]))
    print("=================================================")