#this file contains the default parameters for the robot

#Home Position is the initial position of the robot. which are in the form of [x, y, z, roll, pitch, yaw] i.e [0, 0, 0, 90, 90, 90]
Home_Position = [0, 0, 0, 90, 90, 90]

#Default Gripper Position is the initial position of the gripper. which are in the form of 0
Default_Gripper_Position = 0

# Camera feeds shown in the web UI. Each entry binds a stable id to a ROS
# Image topic and a human-readable label. Add more entries to enable
# multi-camera switching from the persistent camera widget.
Camera_Topics = [
    {'id': 'main', 'topic': '/camera/image_raw', 'name': 'Phone Cam'},
]


# DH Parameters Table [theta, alpha, r, d]
# theta: joint angle (variable)
# alpha: link twist angle (constant)
# r: link length (constant)
# d: link offset (constant)
