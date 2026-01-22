#this file contains the default parameters for the robot
import numpy as np; from numpy import pi 

#Home Position is the initial position of the robot. which are in the form of [x, y, z, roll, pitch, yaw] i.e [0, 0, 0, 90, 90, 90]
Home_Position = [0, 0, 0, 90, 90, 90]

#Default Gripper Position is the initial position of the gripper. which are in the form of 0
Default_Gripper_Position = 0

# Define robot link lengths (modify as per your robot's dimensions)
link = [22.22, 27.55, 29.4, 4, 3.25, 21]  # link[0]=a1, link[1]=a2, link[2]=a3, link[3]=a4, link[4]=a5, link[5]=a6
#link = [22, 27, 29, 4, 3, 21]

# DH Parameters Table [theta, alpha, r, d]
# theta: joint angle (variable)
# alpha: link twist angle (constant)
# r: link length (constant)
# d: link offset (constant)
'''
dh_params = [
    # [theta, alpha, r, d]
    [(5.710*pi)/180,          -pi/2,                  0,                            link[0]],           # Joint 1
    [(5.739*pi)/180,           0,                    -link[1],                      0],           # Joint 2
    [-(15.25*pi)/180,          pi/2,                  0,                            0],                # Joint 3
    [-pi/2,                   -pi/2,                  0,                          -(link[2]+link[3])], # Joint 4
    [-pi/2,                   -pi/2,                  0,                            0],                # Joint 5
    [0,                        0,                     0,                            link[-2]+link[-1]]    # Joint 6
]
'''
dh_params = [
    # [theta, alpha, r, d]
    [(0*pi)/180+pi,          -pi/2,                  0,                            link[0]],           # Joint 1
    [(0*pi)/180,           0,                    -link[1],                      0],           # Joint 2
    [-(0*pi)/180,          pi/2,                  0,                            0],                # Joint 3
    [-pi/2,                   -pi/2,                  0,                          -(link[2]+link[3])], # Joint 4
    [-pi/2,                   -pi/2,                  0,                            0],                # Joint 5
    [0,                        0,                     0,                            link[-2]+link[-1]]    # Joint 6
]