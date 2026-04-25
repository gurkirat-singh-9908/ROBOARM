"""
IK pipeline launch file — full stack, no RViz, no OMPL.

Data flow:
  fetch_data.py  →  /target_pose (PoseStamped)     position + orientation
                 →  /roboarm/gripper (Float64)      gripper %

  ik_mover       →  direct setFromIK (<10 ms)
                 →  /arm_controller/joint_trajectory

  joint_state_broadcaster  →  /joint_states (100 Hz smooth traj)

  bridge_node    →  Arduino serial

Start the full stack:
  ros2 launch roboticarm_moveit2 ik_pipeline.launch.py

Override serial port / baud rate:
  ros2 launch roboticarm_moveit2 ik_pipeline.launch.py serial_port:=/dev/ttyUSB1 baud_rate:=115200
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = get_package_share_directory('roboticarm_moveit2')

    # ── Launch arguments ───────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Arduino serial port')
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='9600',
        description='Arduino baud rate')
    jump_threshold_arg = DeclareLaunchArgument(
        'jump_threshold_deg', default_value='25.0',
        description='Per-joint jump warning threshold (degrees)')

    moveit_config = (
        MoveItConfigsBuilder('roboticarm', package_name='roboticarm_moveit2')
        .to_moveit_configs()
    )

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', name))
        )

    # ── ros2_control: mock hardware + joint_state_broadcaster ─────────────────
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            str(moveit_config.package_path / 'config/ros2_controllers.yaml'),
        ],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
        output='screen',
    )

    # ── IK mover: direct setFromIK, publishes JointTrajectory ─────────────────
    ik_mover = Node(
        package='roboticarm_moveit2',
        executable='ik_mover',
        output='screen',
        parameters=[
            moveit_config.robot_description,            # robot_description (URDF)
            moveit_config.robot_description_semantic,   # robot_description_semantic (SRDF)
            moveit_config.robot_description_kinematics, # kinematics solver config
        ],
    )

    # ── Arduino bridge: /joint_states + /roboarm/gripper → serial ─────────────
    bridge_node = Node(
        package='arduino_bridge',
        executable='bridge_node',
        output='screen',
        parameters=[{
            'serial_port':        LaunchConfiguration('serial_port'),
            'baud_rate':          LaunchConfiguration('baud_rate'),
            'jump_threshold_deg': LaunchConfiguration('jump_threshold_deg'),
            'min_send_interval':  0.05,
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        jump_threshold_arg,

        include('static_virtual_joint_tfs.launch.py'),
        include('rsp.launch.py'),        # publishes robot_description + TFs
        ros2_control_node,
        include('spawn_controllers.launch.py'),   # arm_controller + joint_state_broadcaster
        ik_mover,
        bridge_node,
    ])
