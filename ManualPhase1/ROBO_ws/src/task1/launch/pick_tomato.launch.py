"""
Task 1 — pick a tomato.  (PC side)

Starts ONLY the detector + pick orchestrator. This is the PC half of the
split: detection + planning run here, on a machine with a display. The
arm-side arduino_bridge runs separately on the headless Pi and is NOT
launched here — pick_tomato just publishes /joint_states and
/roboarm/gripper, which reach the Pi's bridge over the shared ROS graph
(same ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY=0).

  ros2 launch task1 pick_tomato.launch.py
  ros2 launch task1 pick_tomato.launch.py camera_index:=2 pick_threshold:=0.8
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_index = LaunchConfiguration('camera_index')
    pick_threshold = LaunchConfiguration('pick_threshold')
    show_feed = LaunchConfiguration('show_feed')

    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('pick_threshold', default_value='0.75'),
        DeclareLaunchArgument('show_feed', default_value='true'),

        Node(
            package='task1',
            executable='tomato_detector',
            name='tomato_detector',
            output='screen',
            parameters=[{
                'camera_index': camera_index,
                'show_feed': show_feed,
            }],
        ),
        Node(
            package='task1',
            executable='pick_tomato',
            name='pick_tomato',
            output='screen',
            parameters=[{
                'pick_threshold': pick_threshold,
            }],
        ),
    ])
