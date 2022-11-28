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
    robot_parts["torso_lift_joint"].setPosition(0.15)
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


    # https://gist.github.com/ItsMichal/4a8fcb330d04f2ccba582286344dd9a7
    with open("tiago_urdf.urdf", "w") as file:  
        file.write(robot.getUrdf())

    global my_chain
    my_chain = Chain.from_urdf_file("tiago_urdf.urdf", last_link_vector=[0.012365, 0, -0.1741],
                                    base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_joint", "TIAGo front arm"],
                                    active_links_mask=[False, False, True, False, True, True ,True ,True ,True ,True ,True , False ,False ,False ,False])

    
    print(my_chain.links)
    # Initialize the arm motors and encoders.
    global motors
    motors = []
    for link in my_chain.links:
        if link.name in part_names and link.name != "torso_lift_joint" and link.name != "gripper_left_finger_joint" and link.name != "gripper_right_finger_joint":
            motor = robot.getDevice(link.name)
            if link.name == "torso_lift_joint":
                motor.setVelocity(0.07)
            else:
                motor.setVelocity(1)
                
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(timestep)
            motors.append(motor) 

    global DEFAULT_MANIPULATOR_POSITION, BASKET_MANIPULATOR_POSITION
    DEFAULT_MANIPULATOR_POSITION = [1,1,0]




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

    global CHECKPOINTS
    CHECKPOINTS = [start, (-2.34, 2.9)]

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

