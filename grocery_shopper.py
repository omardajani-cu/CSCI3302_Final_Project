"""grocery controller."""

# Nov 2, 2022

from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d

#Initialization
print("=== Initializing Grocery Shopper...")
#Consts
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12
LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# for mapping
C_SPACE_DIM = 17
MAP_DIM = (290,151)
WORLD_DIM = (29,15.1) # with walls

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
              "gripper_left_finger_joint","gripper_right_finger_joint")

# 

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.35, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf',0.045,0.045)

robot_parts={}
for i, part_name in enumerate(part_names):
    robot_parts[part_name]=robot.getDevice(part_name)
    robot_parts[part_name].setPosition(float(target_pos[i]))
    robot_parts[part_name].setVelocity(robot_parts[part_name].getMaxVelocity() / 2.0)

# Enable gripper encoders (position sensors)
left_gripper_enc=robot.getDevice("gripper_left_finger_joint_sensor")
right_gripper_enc=robot.getDevice("gripper_right_finger_joint_sensor")
left_gripper_enc.enable(timestep)
right_gripper_enc.enable(timestep)

# Enable Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# Enable GPS and compass localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Enable LiDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Enable display
display = robot.getDevice("display")

# Enable Keyboard
# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# Enable display
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0



vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None


mode = 'manual' # Part 1.1: manual mode
# mode = 'planner'
# mode = 'autonomous'


# ------------------------------------------------------------------
# Helper Functions

def map_to_world(pt):
    return (pt[0]/10.0 - 15, pt[1]/10.0 - 7)

def world_to_map(pt):
    x = int((15 - pt[1])*10)
    y = int((pt[0] + 7)*10)
    if x >= 290:
        x = 289
    if y >= 151:
        y = 150
        
    return (x,y)

def map_to_display(pt):
    x = pt[0]
    y = 360-pt[1]
    return(x,y)

map = np.empty(MAP_DIM)
probability_map = np.empty(MAP_DIM) 

gripper_status="closed"

# Main Loop
while robot.step(timestep) != -1:
    
    
    robot_parts["wheel_left_joint"].setVelocity(vL)
    robot_parts["wheel_right_joint"].setVelocity(vR)
    
    if(gripper_status=="open"):
        # Close gripper, note that this takes multiple time steps...
        robot_parts["gripper_left_finger_joint"].setPosition(0)
        robot_parts["gripper_right_finger_joint"].setPosition(0)
        if right_gripper_enc.getValue()<=0.005:
            gripper_status="closed"
    else:
        # Open gripper
        robot_parts["gripper_left_finger_joint"].setPosition(0.045)
        robot_parts["gripper_right_finger_joint"].setPosition(0.045)
        if left_gripper_enc.getValue()>=0.044:
            gripper_status="open"
    ###################
    #
    # Mapping
    #
    ###################

    ################ v [Begin] Do not modify v ##################
    # ## Ground truth pose
    pose_y = -gps.getValues()[1]
    pose_x = -gps.getValues()[0]
    
    print("POSE FROM GPS")
    print(pose_x, pose_y)
    print("MAP TRANSLATED POSE")
    print(world_to_map((pose_x, pose_y)))

    n = compass.getValues()
    rad = ((math.atan2(n[0], -n[2])))#-1.5708)
    pose_theta = rad
    
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
    
    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # ## The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = -math.cos(alpha)*rho + 0.202
        ry = math.sin(alpha)*rho -0.004


        # ## Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  +(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y
         
        ################ ^ [End] Do not modify ^ ##################

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < LIDAR_SENSOR_MAX_RANGE:
            # ## Part 1.3: visualize map gray values.
            (map_x, map_y) = world_to_map((wx,wy))
            probability_map[map_x, map_y] += 5e-3
            if probability_map[map_x, map_y] > 1:
                probability_map[map_x, map_y] = 1   
                
            # ## You will eventually REPLACE the following 3 lines with a more robust version of the map
            # ## with a grayscale drawing containing more levels than just 0 and 1.
            g = probability_map[map_x, map_y]
            color = int((g*256**2 + g*256+g)*255)
            display.setColor(color)
            (display_x, display_y) = map_to_display((map_x, map_y))
            display.drawPixel(display_x,display_y)
            map[map_x, map_y]=1 

    # ## Draw the robot's current pose on the 360x360 display
    display.setColor(int(0xFF0000))
    
    (draw_x, draw_y) = world_to_map((pose_x, pose_y))
    (disp_x, disp_y) = map_to_display((draw_x, draw_y))
    display.drawPixel(disp_x, disp_y)

            
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
