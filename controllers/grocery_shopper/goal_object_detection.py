"""
Name: config.py
Description: Initializes robot, sensor, mapping states and defines global constants and other global variables
"""

# https://github.com/lukicdarkoo/webots-example-visual-tracking/blob/master/controllers/visual_tracker/visual_tracker.py
import config
import numpy as np
import cv2

COLOR_THRESHOLD = 1000

def get_image_from_camera():

    img = config.camera.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    return cv2.flip(img, 1)


def getCenter(largest_contour):
    largest_contour_center = cv2.moments(largest_contour)
    a = largest_contour_center['m10']
    b = largest_contour_center['m00']
    center_x = int(a/b)
    return center_x

def colorDetection():
    img = get_image_from_camera()

    # Segment the image by color in HSV color space
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img, np.array([0, 100, 100]), np.array([30, 255, 255]))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) != 0:
        largest_contour = max(contours, key=cv2.contourArea)
        area_contour = cv2.contourArea(largest_contour) 
        # print("image detected, area is: " + str(area_contour))
        if (area_contour > COLOR_THRESHOLD):
            center = getCenter(largest_contour)
            print("Center is: " + str(center))
            return 1

    return 0
