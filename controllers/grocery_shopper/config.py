"""
Name: config.py
Description: Initializes robot, sensor, mapping states and defines global constants and other global variables
"""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import ikpy.chain
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import math
from enum import Enum

class State(Enum):
    START = 0
    MAPPING = 1
    NAVIGATING = 2
    REROUTING = 3
    GRABBING = 4
    REPOSITIONING = 5
    END = 6

def robot_init():
    global MAX_SPEED, MAX_SPEED_MS, AXLE_LENGTH, MOTOR_LEFT, MOTOR_RIGHT, N_PARTS, WHEEL_RADIUS
    MAX_SPEED = 7.0  # [rad/s]
    MAX_SPEED_MS = 0.633 # [m/s]
    AXLE_LENGTH = 0.4044 # m
    MOTOR_LEFT = 10
    MOTOR_RIGHT = 11
    N_PARTS = 12
    WHEEL_RADIUS = MAX_SPEED_MS/MAX_SPEED

    global robot, timestep, part_names, left_gripper_enc, right_gripper_enc, robot_parts, gripper_status, target_pos, timestep_ms
    robot = Robot()

    timestep = int(robot.getBasicTimeStep())
    timestep_ms = timestep/1000.0
    part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
                "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
                "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
                "gripper_left_finger_joint","gripper_right_finger_joint")

    robot_parts={}
    for part_name in part_names:
        robot_parts[part_name]=robot.getDevice(part_name)

    # may not need here
    robot_parts["torso_lift_joint"].setPosition(0.0)
    robot_parts["torso_lift_joint"].getPositionSensor().enable(timestep)

    robot_parts["wheel_left_joint"].setPosition(math.inf)
    robot_parts["wheel_left_joint"].setVelocity(0.0)
    robot_parts["wheel_right_joint"].setPosition(math.inf)
    robot_parts["wheel_right_joint"].setVelocity(0.0)
            

    robot_parts["gripper_left_finger_joint"].setPosition(0.045)
    robot_parts["gripper_left_finger_joint"].setVelocity(robot_parts["gripper_left_finger_joint"].getMaxVelocity()/2.0)

    robot_parts["gripper_right_finger_joint"].setPosition(0.045)
    robot_parts["gripper_right_finger_joint"].setVelocity(robot_parts["gripper_right_finger_joint"].getMaxVelocity()/2.0)
        
    

    # Enable gripper encoders (position sensors)
    left_gripper_enc=robot.getDevice("gripper_left_finger_joint_sensor")
    right_gripper_enc=robot.getDevice("gripper_right_finger_joint_sensor")
    left_gripper_enc.enable(timestep)
    right_gripper_enc.enable(timestep)
    gripper_status="closed"



def sensor_init():
    global camera, gps, compass, display, keyboard

    # Enable Camera
    camera = robot.getDevice('camera')
    camera.enable(timestep)
    camera.recognitionEnable(timestep)

    # Enable GPS and compass localization
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    compass = robot.getDevice("compass")
    compass.enable(timestep)

    # Enable display
    display = robot.getDevice("display")

    # Enable Keyboard
    # We are using a keyboard to remote control the robot
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)



def lidar_init():
    global lidar_sensor_readings, lidar_offsets, lidar

    # Enable LiDAR
    # lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
    lidar = robot.getDevice('onboard_lidar_1')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    global LIDAR_ANGLE_BINS, LIDAR_SENSOR_MAX_RANGE, LIDAR_SENSOR_MIN_RANGE, LIDAR_ANGLE_RANGE, LIDAR_THRESHOLD
    LIDAR_ANGLE_BINS = 667
    LIDAR_SENSOR_MAX_RANGE = lidar.getMaxRange()
    LIDAR_SENSOR_MIN_RANGE = lidar.getMinRange()

    LIDAR_THRESHOLD = 1.2
    LIDAR_ANGLE_RANGE = lidar.getFov()

    lidar_sensor_readings = [] # List to hold sensor readings
    lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
    lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis


def map_init():
    global WORLD_DIM, MAP_DIM, C_SPACE_DIM
    C_SPACE_DIM = 5 # 5
    MAP_DIM = (161,300)
    WORLD_DIM = (30,16.1) # without walls
    
    global map, probability_map
    map = np.empty(MAP_DIM)
    probability_map = np.empty(MAP_DIM)

    # for path planning
    global start, end, waypoints
    start = (5,0) # (Pose_X, Pose_Z) in meters
    end = (-1.5,1.7) # (Pose_X, Pose_Z) in meters
    waypoints = []

    global CHECKPOINTS, checkpoint_idx
    checkpoint_idx = 0
    goal_item_locations = [(3.51, 3.59), (3.5, 7.17), (11.6, -7.82), (-1.33, -4.05), (2.41, -3.53), (-0.718, 0.36), (-2.7, 0.31), (0.93, 0.37), (-2.66, 0.2), (-2.65, 3.63), (5.87, 7.16)]
    # (2.69, 3.59, 0.76)
    robot_item_locations = []
    for loc in goal_item_locations:
        robot_item_locations.append((-1*loc[0], -1*(loc[1] + 0.75))) # CHANGE THIS FOR OTHER ISLE
    
    global pts 
    pts = []


def pose_init():
    global pose_x, pose_y, pose_theta, vL, vR, robot_state

    # Odometry
    pose_x     = 5.0
    pose_y     = 0.0
    pose_theta = 0.0

    vL = 0
    vR = 0

    robot_state = State.START

def init():
    robot_init()
    sensor_init()
    lidar_init()
    map_init()
    pose_init()

