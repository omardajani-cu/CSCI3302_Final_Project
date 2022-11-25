# mapping functionality for robot
import config
import transformation
import math


def get_gps_update():
    config.pose_x = -config.gps.getValues()[0]
    config.pose_y = -config.gps.getValues()[1]
    n = config.compass.getValues()
    config.pose_theta = ((math.atan2(n[0], n[1])))

def manual_mapper():
    get_gps_update()

    config.lidar_sensor_readings = config.lidar.getRangeImage()
    config.lidar_sensor_readings = config.lidar_sensor_readings[83:len(config.lidar_sensor_readings)-83]
    
    for i, rho in enumerate(config.lidar_sensor_readings):
        alpha = config.lidar_offsets[i]

        if rho < config.LIDAR_SENSOR_MAX_RANGE and rho > config.LIDAR_SENSOR_MIN_RANGE:
            rx = -math.cos(alpha)*rho + 0.202 # offsets are from urdf file base_link_gps(1)_joint
            ry = math.sin(alpha)*rho -0.004

            ## Convert detection from robot coordinates into world coordinates
            wx, wy = transformation.robot_to_world(rx, ry)

            map_x, map_y = transformation.world_to_map(wx, wy)
            config.probability_map[map_x, map_y] += 5e-3
            if config.probability_map[map_x, map_y] > 1:
                config.probability_map[map_x, map_y] = 1 
                
            g = config.probability_map[map_x, map_y] 
            color = int((g*256**2 + g*256+g)*255)
            config.display.setColor(color)

            display_x, display_y = transformation.map_to_display(map_x, map_y)
            config.display.drawPixel(display_x,display_y)
            config.map[map_x, map_y]=1 

    ## Draw the robot's current pose on the 360x360 display
    config.display.setColor(int(0xFF0000))
    draw_x, draw_y = transformation.world_to_map(config.pose_x, config.pose_y)
    display_x, display_y = transformation.map_to_display(draw_x, draw_y)
    config.display.drawPixel(display_x, display_y)