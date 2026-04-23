"""Software-only hand-eye calibration simulation (eye-in-hand).

No hardware required. Launches the simulation infrastructure:
  1. robot_state_publisher     — publishes TF tree from URDF
  2. joint_state_publisher_gui — GUI sliders to move the arm in simulation
  3. rviz2                     — 3D visualization
  4. fake_marker_tf            — static TF: base_link → fake_marker (world-fixed target)
  5. sim_aruco_publisher       — fakes /aruco/pose from TF (camera_1 → fake_marker)

NOTE: sample_collector is NOT launched here — it needs stdin (keyboard input)
which ros2 launch does not pipe through. Run it in a SECOND terminal:

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  ros2 run handeye_calibration sample_collector \\
      --ros-args \\
      -p base_frame:=base_link \\
      -p gripper_frame:=gripper_1 \\
      -p samples_file:=/tmp/handeye_samples.yaml

Workflow:
  1. Terminal 1: ros2 launch handeye_calibration handeye_sim.launch.py
  2. Terminal 2: ros2 run handeye_calibration sample_collector ... (see above)
  3. Use the GUI sliders to move the arm so the camera "looks at" fake_marker
     from different angles (fake_marker frame is visible in RViz)
  4. Press ENTER in Terminal 2 to capture the sample
  5. Repeat for 10+ varied poses — spread across rotations, not just translations
  6. Type 'q' + ENTER in Terminal 2 when done
  7. Run:  ros2 launch handeye_calibration handeye_compute.launch.py
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
    rviz_config = os.path.join(desc_share, 'config', 'display.rviz')

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

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    fake_marker_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_marker_tf',
        output='screen',
        arguments=[
            '--x', '0.30', '--y', '0.0', '--z', '0.20',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', cfg['base_frame'],
            '--child-frame-id', 'fake_marker',
        ],
    )

    sim_aruco = Node(
        package='handeye_calibration',
        executable='sim_aruco_publisher',
        name='sim_aruco_publisher',
        output='screen',
        parameters=[{
            'camera_frame': cfg['camera_frame'],
            'target_frame': 'fake_marker',
        }],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
        fake_marker_tf,
        sim_aruco,
    ])
