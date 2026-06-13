"""Autonomous pipeline tail: ArUco detection → base-frame /target_pose.

Brings up everything that converts what the camera sees into the IK target the
existing ik_pipeline already consumes:

  camera_node      → /camera/image_raw
  aruco_detect     → /aruco/pose (camera_1 frame)
  publish_calibration → static TF base_link → camera_1
  aruco_to_target  → transforms /aruco/pose to /target_pose (base_link frame)

Run AFTER hand-eye calibration has been computed and saved.

Typical full stack:

  Terminal A:  ros2 launch roboticarm_moveit2 ik_pipeline.launch.py
  Terminal B:  ros2 launch handeye_calibration aruco_to_target.launch.py

You can override anything via launch args, e.g.:

  ros2 launch handeye_calibration aruco_to_target.launch.py \\
       hover_offset_z:=0.10 target_marker_id:=4
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share   = get_package_share_directory('handeye_calibration')
    config_path = os.path.join(pkg_share, 'config', 'handeye_config.yaml')

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    result_path_default = os.path.expanduser(cfg.get('calibration_file', 'handeye_result.yaml'))

    calib_arg = DeclareLaunchArgument(
        'calibration_file', default_value=result_path_default,
        description='YAML produced by compute_calibration')
    hover_arg = DeclareLaunchArgument(
        'hover_offset_z', default_value='0.05',
        description='Metres above the marker for the published target pose')
    id_arg = DeclareLaunchArgument(
        'target_marker_id', default_value=str(cfg.get('target_marker_id', -1)),
        description='Only forward this marker ID. -1 = any.')
    smooth_arg = DeclareLaunchArgument(
        'smooth_alpha', default_value='0.3',
        description='Exponential smoothing on target pose (0=off, <1)')
    rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='10.0',
        description='Hz for /target_pose publication. 0 = on every aruco frame.')

    camera_node = Node(
        package='location',
        executable='camera',
        name='camera_node',
        output='screen',
    )

    aruco_node = Node(
        package='location',
        executable='aruco_source',
        name='aruco_detect',
        output='screen',
        parameters=[{
            'show_feed':    False,
            'camera_frame': cfg['camera_frame'],
        }],
    )

    publish_calib = Node(
        package='handeye_calibration',
        executable='publish_calibration',
        name='handeye_publish',
        output='screen',
        parameters=[{'calibration_file': LaunchConfiguration('calibration_file')}],
    )

    aruco_to_target = Node(
        package='location',
        executable='to_target',
        name='aruco_to_target',
        output='screen',
        parameters=[{
            'base_frame':       cfg['base_frame'],
            'camera_frame':     cfg['camera_frame'],
            'target_marker_id': LaunchConfiguration('target_marker_id'),
            'hover_offset_z':   LaunchConfiguration('hover_offset_z'),
            'smooth_alpha':     LaunchConfiguration('smooth_alpha'),
            'publish_rate':     LaunchConfiguration('publish_rate'),
        }],
    )

    return LaunchDescription([
        calib_arg, hover_arg, id_arg, smooth_arg, rate_arg,
        camera_node,
        aruco_node,
        publish_calib,
        aruco_to_target,
    ])
