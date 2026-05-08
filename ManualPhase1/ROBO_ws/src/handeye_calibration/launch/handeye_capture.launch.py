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

    arduino_bridge = Node(
        package='arduino_bridge',
        executable='bridge_node',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': cfg.get('arduino_port', '/dev/ttyUSB0'),
            'baud_rate':   cfg.get('arduino_baud', 9600),
        }],
    )

    camera_node = Node(
        package='ArUcoMarkerRos',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'source':    cfg.get('camera_source', ''),
            'frame_id':  cfg['camera_frame'],
        }],
    )

    aruco_node = Node(
        package='ArUcoMarkerRos',
        executable='aruco_detect',
        name='aruco_detect',
        output='screen',
        parameters=[{
            # Headless Pi → cv2.imshow has no display. Force off.
            'show_feed':    False,
            'camera_frame': cfg['camera_frame'],
        }],
    )

    # NOTE: manual_joint_commander and sample_collector both read stdin.
    # Run each in its own terminal AFTER this launch:
    #   ros2 run handeye_calibration manual_joint_commander
    #   ros2 run handeye_calibration sample_collector \
    #       --ros-args -p base_frame:=base_link -p gripper_frame:=gripper_1 \
    #       -p samples_file:=/tmp/handeye_samples.yaml
    return LaunchDescription([
        robot_state_publisher,
        arduino_bridge,
        camera_node,
        aruco_node,
    ])
