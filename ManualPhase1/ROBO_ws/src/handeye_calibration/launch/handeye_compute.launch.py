"""Solve hand-eye calibration from collected samples and broadcast the result as a static TF.

Run this AFTER handeye_capture.launch.py has been used to collect ≥10 samples.
It will:
  1. Read /tmp/handeye_samples.yaml
  2. Solve AX=XB with cv2.calibrateHandEye
  3. Write handeye_result.yaml
  4. Broadcast the result as a static TF (base_link → camera_1)
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_share   = get_package_share_directory('handeye_calibration')
    config_path = os.path.join(pkg_share, 'config', 'handeye_config.yaml')

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    result_path = os.path.expanduser(cfg.get('calibration_file', 'handeye_result.yaml'))

    compute_node = Node(
        package='handeye_calibration',
        executable='compute_calibration',
        name='handeye_compute',
        output='screen',
        parameters=[{
            'samples_file':     cfg['samples_file'],
            'calibration_file': result_path,
            'mode':             cfg['mode'],
            'method':           cfg['method'],
            'min_samples':      cfg['min_samples'],
            'base_frame':       cfg['base_frame'],
            'gripper_frame':    cfg['gripper_frame'],
            'camera_frame':     cfg['camera_frame'],
        }],
    )

    publish_node = Node(
        package='handeye_calibration',
        executable='publish_calibration',
        name='handeye_publish',
        output='screen',
        parameters=[{'calibration_file': result_path}],
    )

    return LaunchDescription([compute_node, publish_node])
