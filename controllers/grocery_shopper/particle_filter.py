# initialize random particles

# move particles with GPS and add noise

# associate lidar data with landmarks from particles

# update weights with associated data and add noise

# resample with best particles

# https://www.youtube.com/watch?v=eEebeuWsZBk
# https://github.com/niconielsen32/Robotics/blob/main/particle-filter/particlefilter.cpp
import numpy as np
import numpy.random
import config
import helpers


THRESHOLD = 3

class Particle:
    def __init__(self, x,y,theta,id):
        self.x = x
        self.y = y
        self.theta = theta
        self.id = id

def initializeParticles():
    # initialize one particle on each square of map
    # TODO: only initialize empty squares with particles
    global particle_map
    particle_map = np.empty(config.MAP_DIM)
    id_counter = 0
    for i in range(config.MAP_DIM[0]):
        for j in range(config.MAP_DIM[1]):
            particle_map[i,j] = Particle(i,j,0,id_counter)
            id_counter += 1


def getLidarFromMap(particle):
    curr_location = m[particle.x, particle.y]
    max_range = config.LIDAR_SENSOR_MAX_RANGE*10
    # convolution of max_range and curr_location
    
# get lidar mapping from all particles WITHIN certain radius of robot
def getParticlesLidar():
    potential_match_arr = []
    global m
    m = np.load("map.npy")
    for i, particle in enumerate(particle_map):
        # first check euclidean distance is within threshold
        if np.linalg.norm((x_guess, y_guess) - (particle.x, particle.y)) < THRESHOLD:
            potential_match_arr.append(particle)

    for i, particle in enumerate(potential_match_arr):
        getLidarFromMap(particle)
        

# get lidar scan from robot
def getRobotLidar():
    pass

# predict robot position
def predictPosition():
    # get gps position and add random noise to X, Y locations
    helpers.get_gps_update()
    global x_guess, y_guess, theta

    # TODO convert to world coordinates
    x_guess = config.pose_x
    y_guess = config.pose_y
    theta = config.pose_theta