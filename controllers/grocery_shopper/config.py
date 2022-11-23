import math
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import numpy as np

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
    # create the Robot instance.
    robot = Robot()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    timestep_ms = timestep/1000.0
    # All motors except the wheels are controlled by position control. The wheels
    # are controlled by a velocity controller. We therefore set their position to infinite.
    target_pos = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf',0.045,0.045)
    # The Tiago robot has multiple motors, each identified by their names below
    part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
                "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
                "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
                "gripper_left_finger_joint","gripper_right_finger_joint")

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
    # LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
    LIDAR_SENSOR_MAX_RANGE = lidar.getMaxRange()
    LIDAR_SENSOR_MIN_RANGE = lidar.getMinRange()

    LIDAR_THRESHOLD = 1.2
    LIDAR_ANGLE_RANGE = lidar.getFov()

    print(LIDAR_SENSOR_MAX_RANGE)
    print(LIDAR_SENSOR_MIN_RANGE)
    print(LIDAR_ANGLE_RANGE)

    lidar_sensor_readings = [] # List to hold sensor readings
    lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
    lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis


def map_init():
    global WORLD_DIM, MAP_DIM, C_SPACE_DIM
    C_SPACE_DIM = 5
    MAP_DIM = (161,300)
    WORLD_DIM = (30,16.1) # without walls
    
    global map, probability_map
    map = np.empty(MAP_DIM)
    probability_map = np.empty(MAP_DIM)

    # for path planning
    global start, end, waypoints
    start = (5,0) # (Pose_X, Pose_Z) in meters
    end = (0.5,1.7) # (Pose_X, Pose_Z) in meters
    waypoints = []

def pose_init():
    global pose_x, pose_y, pose_theta, vL, vR, stopping_condition

    # Odometry
    pose_x     = 0
    pose_y     = 0
    pose_theta = 0

    vL = 0
    vR = 0

    stopping_condition = 0

def init():
    robot_init()
    sensor_init()
    lidar_init()
    map_init()
    pose_init()

