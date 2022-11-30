# CSCI 3302: Homework 3 - Color Filtering and Blob Detection
# (C) 2022 CSCI3302 Introduction to Robotics 


# Using Color Filtering with Blob Detection
# Please do not collaborate on this assignment -- your code must be your own!

# TODO: INSERT YOUR NAME HERE
LAST_NAME = "Churchill"

import pdb
import pickle
import random
import copy
import cv2 # You may have to "pip install opencv-contrib-python" to install this
import numpy as np # You may have to "pip install numpy" to install this

# List of color ranges to look for... but stored in BGR order because that's how OpenCV does it
# Example: color_ranges = [((0,0,100), (0,0,255)), ((0,100,0), (0,255,0))]
#                         Dark Red to Bright Red    Dark Green to Bright Green
color_ranges = []

def add_color_range_to_detect(lower_bound, upper_bound):
  '''
  @param lower_bound: Tuple of BGR values
  @param upper_bound: Tuple of BGR values
  '''
  global color_ranges
  color_ranges.append([lower_bound, upper_bound]) # Add color range to global list of color ranges to detect

def check_if_color_in_range(bgr_tuple):
  '''
  @param bgr_tuple: Tuple of BGR values
  @returns Boolean: True if bgr_tuple is in any of the color ranges specified in color_ranges
  '''
  global color_ranges
  for entry in color_ranges:
    lower, upper = entry[0], entry[1]
    in_range = True
    for i in range(len(bgr_tuple)):
      if bgr_tuple[i] < lower[i] or bgr_tuple[i] > upper[i]:
        in_range = False
        break
    if in_range: return True
  return False

def do_color_filtering(img):
  # Color Filtering
  # Objective: Take an RGB image as input, and create a "mask image" to filter out irrelevant pixels
  # Definition "mask image":
  #    An 'image' (really, a matrix) of 0s and 1s, where 0 indicates that the corresponding pixel in 
  #    the RGB image isn't important (i.e., what we consider background) and 1 indicates foreground.
  #
  #    Importantly, we can multiply pixels in an image by those in a mask to 'cancel out' all of the pixels we don't
  #    care about. Once we've done that step, the only non-zero pixels in our image will be foreground
  #
  # Approach:
  # Create a mask image: a matrix of zeros ( using np.zeroes([height width]) ) of the same height and width as the input image.
  # For each pixel in the input image, check if it's within the range of allowable colors for your detector
  #     If it is: set the corresponding entry in your mask to 1
  #     Otherwise: set the corresponding entry in your mask to 0 (or do nothing, since it's initialized to 0)
  # Return the mask image
  img_height = img.shape[0]
  img_width = img.shape[1]

  # Create a matrix of dimensions [height, width] using numpy
  mask = np.zeros([img_height, img_width]) # Index mask as [height, width] (e.g.,: mask[y,x])

  # TODO: Iterate through each pixel (x,y) coordinate of the image, 
  #       checking if its color is in a range we've specified using check_if_color_in_range
  # TIP: You'll need to index into 'mask' using (y,x) instead of (x,y) as you may be
  #      more familiar with, due to how the matrices are stored

  for h in range(img_height):
    for w in range(img_width):
      if check_if_color_in_range(img[h,w]):
        mask[h,w] = 1

  return mask

def expand(img_mask, cur_coordinate, coordinates_in_blob):
  # Find all of the non-zero pixels connected to a location

  # If value of img_mask at cur_coordinate is 0, or cur_coordinate is out of bounds (either x,y < 0 or x,y >= width or height of img_mask) return and stop expanding

  # Otherwise, add this to our blob:
  # Set img_mask at cur_coordinate to 0 so we don't double-count this coordinate if we expand back onto it in the future
  # Add cur_coordinate to coordinates_in_blob
  # Call expand on all 4 neighboring coordinates of cur_coordinate (above/below, right/left). Make sure you pass in the same img_mask and coordinates_in_blob objects you were passed so the recursive calls all share the same objects

  if cur_coordinate[0] < 0 or cur_coordinate[1] < 0 or cur_coordinate[0] >= img_mask.shape[0] or cur_coordinate[1] >= img_mask.shape[1]: 
    return
  if img_mask[cur_coordinate[0], cur_coordinate[1]] == 0.0: 
    return

  img_mask[cur_coordinate[0],cur_coordinate[1]] = 0
  coordinates_in_blob.append(cur_coordinate)

  above = [cur_coordinate[0]-1, cur_coordinate[1]]
  below = [cur_coordinate[0]+1, cur_coordinate[1]]
  left = [cur_coordinate[0], cur_coordinate[1]-1]
  right = [cur_coordinate[0], cur_coordinate[1]+1]
  for coord in [above, below, left, right]: 
    expand(img_mask, coord, coordinates_in_blob)


def expand_nr(img_mask, cur_coord, coordinates_in_blob):
  # Non-recursive function to find all of the non-zero pixels connected to a location

  # If value of img_mask at cur_coordinate is 0, or cur_coordinate is out of bounds (either x,y < 0 or x,y >= width or height of img_mask) return and stop expanding
  # Otherwise, add this to our blob:
  # Set img_mask at cur_coordinate to 0 so we don't double-count this coordinate if we expand back onto it in the future
  # Add cur_coordinate to coordinates_in_blob
  # Call expand on all 4 neighboring coordinates of cur_coordinate (above/below, right/left). Make sure you pass in the same img_mask and coordinates_in_blob objects you were passed so the recursive calls all share the same objects
  
  coordinates_in_blob = []
  coordinate_list = [cur_coord] # List of all coordinates to try expanding to
  while len(coordinate_list) > 0:
    cur_coordinate = coordinate_list.pop() # Take the first coordinate in the list and perform 'expand' on it
    # TODO: Check to make sure cur_coordinate is in bounds, otherwise 'continue'
    if cur_coordinate[0] < 0 or cur_coordinate[1] < 0 or cur_coordinate[0] >= img_mask.shape[0] or cur_coordinate[1] >= img_mask.shape[1]:
      continue
    else:
        # TODO: Check to see if the value is 0, if so, 'continue'
        if img_mask[cur_coordinate[0], cur_coordinate[1]] == 0:
          continue
        else:
        # TODO: Set image mask at this coordinate to 0
          img_mask[cur_coordinate[0], cur_coordinate[1]] = 0
        # TODO: Add this coordinate to 'coordinates_in_blob'
          coordinates_in_blob.append(cur_coordinate)
        # TODO: Add all neighboring coordinates (above, below, left, right) to coordinate_list to expand to them
          above = (cur_coordinate[0]-1, cur_coordinate[1])
          below = (cur_coordinate[0]+1, cur_coordinate[1])
          left = (cur_coordinate[0], cur_coordinate[1]-1)
          right = (cur_coordinate[0], cur_coordinate[1]+1)
          for coord in [above, below, left, right]: 
            coordinate_list.append(coord)


  return coordinates_in_blob

def get_blobs(img_mask):
  # Blob detection
  # Objective: Take a mask image as input, group each blob of non-zero pixels as a detected object, 
  #            and return a list of lists containing the coordinates of each pixel belonging to each blob.
  # Recommended Approach:
  # Create a copy of the mask image so you can edit it during blob detection
  # Create an empty blobs_list to hold the coordinates of each blob's pixels
  # Iterate through each coordinate in the mask:
  #   If you find an entry that has a non-zero value:
  #     Create an empty list to store the pixel coordinates of the blob
  #     Call the recursive "expand" function on that position, recording coordinates of non-zero pixels connected to it
  #     Add the list of coordinates to your blobs_list variable
  # Return blobs_list

  img_mask_height = img_mask.shape[0]
  img_mask_width = img_mask.shape[1]
 
  # TODO: Copy image mask into local variable using copy.copy
  mask_copy = copy.copy(img_mask)
  blobs_list = [] # List of all blobs, each element being a list of coordinates belonging to each blob

  # TODO: Iterate through all 'y' coordinates in img_mask
  #   TODO: Iterate through all 'x' coordinates in img_mask
  #     TODO: If mask value at [y,x] is 1, call expand_nr on copy of image mask and coordinate (y,x), giving a third argument of an empty list to populate with blob_coords.
  #     TODO: Add blob_coords to blobs_list

  for y in range(img_mask_height):
    for x in range(img_mask_width):
      if img_mask[y,x] == 1:
        blobs_coords = []
        blobs_coords = expand_nr(mask_copy, (y,x), blobs_coords)
        blobs_list.append(blobs_coords)

  return blobs_list


def get_blob_centroids(blobs_list):
  # Coordinate retrieval
  # Objective: Take a list of blobs' coordinate lists and return the coordinates of each blob's center.
  # Approach:
  #     Create an object_positions_list
  #     For each list in the blobs_list:
  #     Check to see if the list of coordinates is big enough to be an object we're looking for, otherwise continue. (The legos are several hundred pixels or more in size!)
  #     Find the centroid of all coordinates in the coordinates list (hint: np.mean(my_list_var,axis=0) will take the mean)
  #     Add that centroid (x,y) tuple to object_positions_list
  # Return object_positions_list
  list_threshold = 500 # this may need to be a bigger value --> try 300
  object_positions_list = []
  for blob_list in blobs_list:
    if len(blob_list) > list_threshold:
      centroid = np.mean(blob_list, axis=0)
      object_positions_list.append(centroid)
    else:
      continue

  # TODO: Implement blob centroid calculation
  
  return object_positions_list

def main():
  global img_height, img_width
  # Read in image using the imread function
  img = cv2.imread('./lego-blob.png')
  # Examples of adding color ranges: You'll need to find the right ones for isolating the lego blocks!
  
  # TODO: Add the color ranges for each block here!
  # HINT: Open up the image in your favorite image editor and use the eyedropper tool to find a light and dark spot on each block -- those colors can be used for each range.
  # Examples:
  add_color_range_to_detect([0,0,200], [80,80,255]) # Detect red
  add_color_range_to_detect([30,100,10], [120,255,110]) # Detect green
  add_color_range_to_detect([0,170,255], [35,255,255]) # Detect yellow
  add_color_range_to_detect([100,25,0], [255,140,80]) # Detect blue

  ########## PART 1 ############
  # Create img_mask of all foreground pixels, where foreground is defined as passing the color filter
  img_mask = do_color_filtering(img)

  ########## PART 2 ############
  # Find all the blobs in the img_mask
  blobs = get_blobs(img_mask)

  ########## PART 3 ############
  # Get the centroids of the img_mask blobs
  object_positions_list = get_blob_centroids(blobs)

  ########## PART 4 (Done for you) ############
  # Display images and blob annotations
  img_markup = img.copy()
  for obj_pos in object_positions_list:
    obj_pos_vector = np.array(obj_pos).astype(np.int32) # In case your object positions weren't numpy arrays
    img_markup = cv2.circle(img_markup,(obj_pos_vector[1], obj_pos_vector[0]),5,(0,0,0),10)
    print("Object pos: " + str(obj_pos_vector))


  print("Displaying images")
  # Display the original image, the mask, and the original image with object centers drawn on it
  # Objective: Show that your algorithm works by displaying the results!
  #
  # Approach:
  # Use the OpenCV imshow() function to display the results of your object detector
  # Create a window for each image
  cv2.imshow('orig', img)
  cv2.imshow('mask', img_mask)
  cv2.imshow('located', img_markup)
  cv2.waitKey(-1)  # Wait until a key is pressed to exit the program
  cv2.destroyAllWindows() # Close all the windows

if __name__ == '__main__':
  main()
