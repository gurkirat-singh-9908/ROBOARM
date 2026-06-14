"""
pick.launch.py — run a pick (PC side, colour/open-loop path)

Starts the colour location source + the pick orchestrator. This is the PC
half of the split: detection + planning run here (machine with a display).
The arm-side arduino_bridge runs separately on the headless Pi and is NOT
launched here — pick publishes /joint_states and /roboarm/gripper, which
reach the Pi's bridge over the shared ROS graph (same ROS_DOMAIN_ID,
ROS_LOCALHOST_ONLY=0).

  ros2 launch tasks pick.launch.py
  ros2 launch tasks pick.launch.py object:=tomato camera_source:=2 pick_threshold:=0.8

For the ArUco + real-IK path instead, launch the location element and the IK
stack separately:
  ros2 launch location location.launch.py source:=aruco
  ros2 launch roboticarm_moveit2 ik.launch.py
  ros2 run tasks pick --ros-args -p positioning:=ik
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    obj = LaunchConfiguration('object')
    camera_source = LaunchConfiguration('camera_source')
    pick_threshold = LaunchConfiguration('pick_threshold')
    show_feed = LaunchConfiguration('show_feed')

    return LaunchDescription([
        DeclareLaunchArgument('object', default_value='tomato'),
        DeclareLaunchArgument('camera_source', default_value='0',
                              description='camera index or stream URL'),
        DeclareLaunchArgument('pick_threshold', default_value='0.75'),
        DeclareLaunchArgument('show_feed', default_value='true'),

        Node(
            package='camera',
            executable='camera',
            name='camera',
            output='screen',
            parameters=[{'source': camera_source}],
        ),
        Node(
            package='location',
            executable='color_source',
            name='color_source',
            output='screen',
            parameters=[{
                'object': obj,
                'show_feed': show_feed,
            }],
        ),
        Node(
            package='tasks',
            executable='pick',
            name='pick',
            output='screen',
            parameters=[{
                'object': obj,
                'positioning': 'open_loop',
                'pick_threshold': pick_threshold,
            }],
        ),
    ])
