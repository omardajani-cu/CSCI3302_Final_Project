import numpy as np
import math
import config


# initializing slam data
start_pose = [-5, 0, np.pi/2]

# INPUTS

# mean pose estimated from previous timestep
pose_previous = start_pose # [x_{t-1}, y_{t-1}, theta_{t-1}]
# covariance matrix
covariance_previous = np.repeat((3,3), 0.1) # uncertainty in pose estimate

# control inputs from current timestep
u_t = [config.vL, config.vR] # vL, vR are angular wheel velocities

# measurements from current timestep
measurements_current = []
for i, rho in enumerate(config.lidar_sensor_readings):
    alpha = config.lidar_offsets[i]
    measurements_current.append([rho, alpha])

# OUTPUTS
pose_updated = []
covariance_updated = []


## helper functions http://andrewjkramer.net/intro-to-the-ekf-step-1/

# calculates new pose based on previous pose and odometry data
def motion_update():
    x_update = config.pose_x + (config.vL+config.vR)/2*config.WHEEL_RADIUS*config.timestep_ms*math.cos(config.pose_theta)
    y_update = config.pose_y - (config.vL+config.vR)/2*config.WHEEL_RADIUS*config.timestep_ms*math.sin(config.pose_theta)
    theta_update = config.pose_theta + (config.vR-config.vL)/config.AXLE_LENGTH*config.WHEEL_RADIUS*config.timestep_ms
    
    # Bound pose_theta between [-pi, 2pi+pi/2]
    if theta_update > 6.28+3.14/2:
        theta_update -= 6.28
    if theta_update < -3.14:
        theta_update += 6.28

    pose_estimated = [x_update, y_update, theta_update]

    vL_linear = config.vL*config.WHEEL_RADIUS
    vR_linear = config.vR*config.WHEEL_RADIUS

    forward_velocity = config.AXLE_LENGTH/2*(vL_linear + vR_linear)/(vR_linear - vL_linear)
    angular_velocity = (vR_linear-vL_linear)/config.AXLE_LENGTH

    G_t = np.array([
        [1,0,-forward_velocity*config.timestep_ms*math.sin(config.pose_theta + angular_velocity*config.timestep_ms/2)],
        [0,1, forward_velocity*config.timestep_ms*math.cos(config.pose_theta + angular_velocity*config.timestep_ms/2)],
        [1,0,1]
    ])

    noise_params = [1,1,1,1]

    # noise for timestep delta_t
    M_t = np.array([
        [noise_params[0]*forward_velocity**2 + noise_params[1]*angular_velocity**2, 0],
        [0, noise_params[2]*forward_velocity**2 + noise_params[3]*angular_velocity**2]
    ])

    # Jacobian of motion model with respect to motion parameters
    V_t = np.array([
        [math.cos(config.pose_theta + angular_velocity*config.timestep_ms/2), -0.5*math.sin(config.pose_theta + angular_velocity*config.timestep_ms/2)],
        [math.sin(config.pose_theta + angular_velocity*config.timestep_ms/2), 0.5*math.cos(config.pose_theta + angular_velocity*config.timestep_ms/2)],
        [0,1]
    ])

    covariance_estimated = G_t @ covariance_previous @ G_t.T + V_t @ M_t @ V_t.T

    return pose_estimated, covariance_estimated


def sensor_update(pose_estimated, rho, alpha):
    expected_landmark = 


def slam_main():
    for i, rho in enumerate(config.lidar_sensor_readings):
        alpha = config.lidar_offsets[i]

        if rho > config.LIDAR_SENSOR_MAX_RANGE:
            continue

        # ## The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = -math.cos(alpha)*rho + 0.202
        ry = math.sin(alpha)*rho -0.004

        # ## Convert detection from robot coordinates into world coordinates
        wx =  math.cos(config.pose_theta)*rx - math.sin(config.pose_theta)*ry + config.pose_x
        wy =  +(math.sin(config.pose_theta)*rx + math.cos(config.pose_theta)*ry) + config.pose_y

key = config.keyboard.getKey()
while(config.keyboard.getKey() != -1): pass
if key == config.keyboard.LEFT :
    config.vL = -config.MAX_SPEED
    config.vR = config.MAX_SPEED
elif key == config.keyboard.RIGHT:
    config.vL = config.MAX_SPEED
    config.vR = -config.MAX_SPEED
elif key == config.keyboard.UP:
    config.vL = config.MAX_SPEED
    config.vR = config.MAX_SPEED
elif key == config.keyboard.DOWN:
    config.vL = -config.MAX_SPEED
    config.vR = -config.MAX_SPEED
elif key == ord(' '):
    config.vL = 0
    config.vR = 0
elif key == ord('S'):
    # Part 1.4: Filter map and save to filesystem          
    np.save("map.npy",config.probability_map)
    print("Map file saved")
else: # slow down
    config.vL *= 0.75
    config.vR *= 0.75

# Actuator commands
config.robot_parts[config.MOTOR_LEFT].setVelocity(config.vL)
config.robot_parts[config.MOTOR_RIGHT].setVelocity(config.vR)

