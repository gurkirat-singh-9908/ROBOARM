"""Launch camera + ArUco detector + sample collector for hand-eye calibration.

Brings up everything needed to collect calibration samples. After launching,
move the arm to varied poses and press ENTER in the sample_collector terminal
to capture each pair.
"""
import os

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share  = get_package_share_directory('handeye_calibration')
    desc_share = get_package_share_directory('roboticarm_description')
    config_path = os.path.join(pkg_share, 'config', 'handeye_config.yaml')
    xacro_path  = os.path.join(desc_share, 'urdf', 'roboticarm.xacro')

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    robot_description = xacro.process_file(xacro_path).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    camera_node = Node(
        package='ArUcoMarkerRos',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )

    aruco_node = Node(
        package='ArUcoMarkerRos',
        executable='aruco_detect',
        name='aruco_detect',
        output='screen',
        parameters=[{'show_feed': True}],
    )

    collector = Node(
        package='handeye_calibration',
        executable='sample_collector',
        name='handeye_sample_collector',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'base_frame':       cfg['base_frame'],
            'gripper_frame':    cfg['gripper_frame'],
            'samples_file':     cfg['samples_file'],
            'target_marker_id': cfg['target_marker_id'],
        }],
    )

    return LaunchDescription([robot_state_publisher, camera_node, aruco_node, collector])
