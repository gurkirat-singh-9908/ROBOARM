"""
location.launch.py  —  bring up the location element

  camera ─/camera/image_raw─▶ <source> ─▶ (/aruco/pose | /location/center)
                                              │
                                       to_target ─▶ /target_pose (base_link)

Args:
  source        aruco | color           which position source to run (default aruco)
  object        tomato                  objects/<name>.yaml for the colour source
  camera_source 0                       camera index or stream URL
  show_feed     true                    open the live debug window

The aruco source already emits a metric 3D pose; to_target reframes it to
base_link for the IK element. The colour source emits a pixel centre only
(no depth yet) — run it without to_target for the open-loop pick path.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source = LaunchConfiguration('source')
    obj = LaunchConfiguration('object')
    cam = LaunchConfiguration('camera_source')
    show = LaunchConfiguration('show_feed')

    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='aruco',
                              description='aruco | color'),
        DeclareLaunchArgument('object', default_value='tomato',
                              description='objects/<name>.yaml for the colour source'),
        DeclareLaunchArgument('camera_source', default_value='0',
                              description='camera index or stream URL'),
        DeclareLaunchArgument('show_feed', default_value='true'),

        Node(package='camera', executable='camera', name='camera',
             parameters=[{'source': cam}], output='screen'),

        # ArUco 3D source + reframe to base_link
        Node(package='location', executable='aruco_source', name='aruco_source',
             condition=LaunchConfigurationEquals('source', 'aruco'),
             parameters=[{'show_feed': show}], output='screen'),
        Node(package='location', executable='to_target', name='to_target',
             condition=LaunchConfigurationEquals('source', 'aruco'),
             output='screen'),

        # Colour (pixel) source — subscribes /camera/image_raw from camera node
        Node(package='location', executable='color_source', name='color_source',
             condition=LaunchConfigurationEquals('source', 'color'),
             parameters=[{'object': obj, 'show_feed': show}], output='screen'),
    ])
