# Robotics Final Project

## File Structure & Descriptions
```
---------------
+ = Python file
> = npy file
* = URDF file
---------------
+ config.py
    - Defines and initializes global variables, robot sensors, and robot state
+ controls.py
    - Contains the code for the manual controller and autonomous controller for driving the robot around the world
+ goal_object_detection.py
    - Uses openCV color detection to inform if a goal object is present in the frame.
+ grocery_shopper.py
    - This file is the main thread of execution and is the file that the Tiago Robot has set to the main controller. Based on the robot's state, the file calls upon other files to change the robot behavior. The main loop resides here.
+ helpers.py
    - Contains general functions that are used throughout the program.
+ manipulator_ik.py
    - IK and manual manipulator combined. This gives user the control to adjust joints of the manipulator as well as providing functionality for the user to choose to use inverse kinematics to navigate the arm to positions.
+ mapping.py
    - Functionality for obtaining lidar readings and robot position and displaying them on a map. WHen mapping, this file also saves the position and lidar sensor information to a probability map, which is later used for path planning.
+ planner.py
    - Uses a navigation algorithm and the convolved map from the initial mapping phase to construct a path composed of waypoints. 
+ transformation.py
    - Contains functions for translating between map, world, and display coordinate systems.
> checkpoints.npy
> map.npy
> path.npy
* tiago_urdf.urdf
```

## Process

## Resources Used

## Video Link
