"""
Name: trilateration.py
Description: Code to trilaterate position based on known base stations and noisy GPS measurement
"""

import config
import helpers
import numpy as np
import transformation
import math
import random
from scipy.optimize import minimize

# Cone positions, used as landmarks or base stations with known coordinates
LANDMARKS = [[8.93,-5.79],
            [8.93, -2.04],
            [8.93, 2.04],
            [8.93, 5.89],
            [-13.8,6.03],
            [-13.8,2.2],
            [-13.8,-1.96],
            [-13.8,-5.79],
            [0,-6.74],
            [0,-3.53],
            [0,0],
            [0,3.81],
            [0,7.64],
            [-6.36,-6.36],
            [-6.36, -3.52],
            [-6.36,0],
            [-6.56,3.76],
            [-6.36,7.65]]

# enable lidar for trilateration measurments, this lidar is positioned high above the robot so that none of the obstacles interfere, it will just detect the landmarks 
# within the lidar sensor range
def enableSLAMLidar():
    global slam_lidar_0, slam_lidar_1, slam_lidar_0_sensor_readings, slam_lidar_1_sensor_readings
    global slam_lidar_0_offsets, slam_lidar_1_offsets

    # lidar for front 180 view
    slam_lidar_0 = config.robot.getDevice('slam_lidar_0')
    # lidar for rear 180 view
    slam_lidar_1 = config.robot.getDevice('slam_lidar_1')

    slam_lidar_0.enable(config.timestep)
    slam_lidar_0.enablePointCloud()
    slam_lidar_1.enable(config.timestep)
    slam_lidar_1.enablePointCloud()

    global LIDAR_ANGLE_BINS, LIDAR_SENSOR_MAX_RANGE, LIDAR_SENSOR_MIN_RANGE, LIDAR_ANGLE_RANGE, LIDAR_THRESHOLD
    
    LIDAR_ANGLE_BINS = slam_lidar_0.getHorizontalResolution()
    LIDAR_SENSOR_MAX_RANGE = slam_lidar_0.getMaxRange()

    LIDAR_THRESHOLD = 1.2
    LIDAR_ANGLE_RANGE = slam_lidar_0.getFov()

    slam_lidar_0_sensor_readings = []
    slam_lidar_1_sensor_readings = []
    slam_lidar_0_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
    diff = LIDAR_ANGLE_RANGE/LIDAR_ANGLE_BINS
    a1 = np.linspace(LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_RANGE-diff, int(LIDAR_ANGLE_BINS/2))
    a2 = np.linspace(-LIDAR_ANGLE_RANGE, -LIDAR_ANGLE_RANGE/2., int(LIDAR_ANGLE_BINS/2)+1)
    slam_lidar_1_offsets = np.concatenate([a1, a2])

# since the cones are not a single point, this function helps to associate any point detected to the absolute center of a landmark that is known
def associatePointWithLandmark(pt):
    pt = np.array(pt)
    for l in LANDMARKS:
        l = np.array(l)
        if np.linalg.norm(pt-l) < 1:
            return l

    # otherwise, cannot associate with known landmark
    return [-100,-100]

# gets the landmarks and distances to those landmarks that were detected from the trilateration lidars
# also draws the cone positions in green on the display overlay when in mapping mode 
def lidarMapperSLAM():
    global slam_lidar_0, slam_lidar_1, slam_lidar_0_sensor_readings, slam_lidar_1_sensor_readings
    global slam_lidar_0_offsets, slam_lidar_1_offsets

    helpers.get_gps_update()
    slam_lidar_0_sensor_readings = slam_lidar_0.getRangeImage()
    
    landmarks_detected = []
    distances_detected = []

    for i, rho in enumerate(slam_lidar_0_sensor_readings):
        alpha = slam_lidar_0_offsets[i]

        if rho < config.LIDAR_SENSOR_MAX_RANGE and rho > config.LIDAR_SENSOR_MIN_RANGE:
            rx = -math.cos(alpha)*rho + 0.202 # offsets are from urdf file base_link_gps(1)_joint
            ry = math.sin(alpha)*rho -0.004

            ## Convert detection from robot coordinates into world coordinates
            wx, wy = transformation.robot_to_world(rx, ry)
            lm = associatePointWithLandmark([wx,wy])
            if lm[0] == -100:
                #print("Could not associate point with landmark")
                pass
            else:
                landmarks_detected.append(lm)
                distances_detected.append(rho)

            map_x, map_y = transformation.world_to_map(wx, wy)

            # draw on display
            config.display.setColor(int(0x00FF00))
            display_x, display_y = transformation.map_to_display(map_x, map_y)
            config.display.drawPixel(display_x,display_y)

    slam_lidar_1_sensor_readings = slam_lidar_1.getRangeImage()
    for i, rho in enumerate(slam_lidar_1_sensor_readings):
        alpha = slam_lidar_1_offsets[i]

        if rho < config.LIDAR_SENSOR_MAX_RANGE and rho > config.LIDAR_SENSOR_MIN_RANGE:
            rx = -math.cos(alpha)*rho + 0.202 # offsets are from urdf file base_link_gps(1)_joint
            ry = math.sin(alpha)*rho -0.004

            ## Convert detection from robot coordinates into world coordinates
            wx, wy = transformation.robot_to_world(rx, ry)
            lm = associatePointWithLandmark([wx,wy])
            if lm[0] == -100:
                print("Could not associate point with landmark")
            else:
                landmarks_detected.append(lm)
                distances_detected.append(rho)

            map_x, map_y = transformation.world_to_map(wx, wy)

            # draw on display
            config.display.setColor(int(0x00FF00))
            display_x, display_y = transformation.map_to_display(map_x, map_y)
            config.display.drawPixel(display_x,display_y)

    return landmarks_detected, distances_detected

# adds noise to GPS measurement
def initialGuess(x,y):
    noise = [random.random(), random.random()]
    noise_sign1 = np.random.randint(2)
    noise_sign2 = np.random.randint(2)

    if noise_sign1 == 0:
        noise_sign1 = -1
    if noise_sign2 == 0:
        noise_sign2 = -1

    predicted_pose = [x + noise_sign1*noise[0],y + noise_sign2*noise[1]]
    return predicted_pose


# calculates mean squared error from predicted position (x) and given locations and distances from measurements
def meanSquaredError(x, locations, distances):
    mse = 0.0
    for location, distance in zip(locations, distances):
        distance_calculated = np.linalg.norm(x-location)
        mse += math.pow(distance_calculated - distance, 2.0)
    return mse / len(distances)

# uses the scipy minimize function to optimize the location based on an initial noisy guess and draws point in blue on the screen
def findPoint(locations, distances, initial_guess):
    plot = []
    if len(distances) < 1:
        print("Could not triangulate, using initial guess")
        print("Location:" + str(initial_guess))
        plot = initial_guess
    else:
        result = minimize(
            meanSquaredError,                         # The error function
            initial_guess,            # The initial guess
            args=(locations, distances), # Additional parameters for mse
            method='L-BFGS-B',           # The optimisation algorithm
            options={
                'ftol':1e-5,         # Tolerance
                'maxiter': 1e+7      # Maximum iterations
            })
        location = result.x
        plot = location

    map_x, map_y = transformation.world_to_map(plot[0], plot[1])

    # draw on display
    config.display.setColor(int(0x0000FF))
    display_x, display_y = transformation.map_to_display(map_x, map_y)
    config.display.drawPixel(display_x,display_y)

    return plot

# main function for getting the location based on trilateration
def getTrilaterationUpdate():
    helpers.get_gps_update()
    initial_guess = initialGuess(config.pose_x, config.pose_y)
    locations, distances = lidarMapperSLAM()

    pt = findPoint(locations, distances, initial_guess)
    return (pt[0], pt[1], config.pose_theta)


    