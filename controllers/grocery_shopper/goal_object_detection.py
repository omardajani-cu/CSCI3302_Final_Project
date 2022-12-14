"""
Name: goal_object_detection.py
Description: Detects yellow blobs using cv libaries
"""

import config
import numpy as np
import cv2

# add a checkpoint for images with this threshold of color
COLOR_THRESHOLD = 300

# convert webots image to opencv usable image
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

# detect yellow color using the findContours function, find the center of the largest "blob" of yellow and find area of largest yellow blob
def colorDetection():
    img = get_image_from_camera()

    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img, np.array([0, 100, 100]), np.array([30, 255, 255]))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) != 0:
        largest_contour = max(contours, key=cv2.contourArea)
        area_contour = cv2.contourArea(largest_contour) 
        if (area_contour > COLOR_THRESHOLD):
            center = getCenter(largest_contour)
            print("Center is: " + str(center))
            return 1

    return 0
