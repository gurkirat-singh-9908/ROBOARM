"""
ik.launch.py — the IK element, ros2_control-free.

Same data flow as ik_pipeline.launch.py but the JointTrajectoryController
(which segfaults on_init after the 2026-05-08 apt upgrade) is replaced by the
lightweight traj_interpolator node. Nothing here depends on ros2_control, so
the IK path is immune to that apt breakage.

  /target_pose (PoseStamped, base_link)
      → ik_mover           setFromIK, direct (<10 ms, no OMPL)
      → /arm_controller/joint_trajectory
      → traj_interpolator  ramp to /joint_states (~50 Hz, smooth)
      → bridge_node        → Arduino serial

Start:
  ros2 launch roboticarm_moveit2 ik.launch.py
Override serial port:
  ros2 launch roboticarm_moveit2 ik.launch.py serial_port:=/dev/ttyUSB1
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

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Arduino serial port')
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Arduino baud rate')
    jump_threshold_arg = DeclareLaunchArgument(
        'jump_threshold_deg', default_value='25.0',
        description='Per-joint jump warning threshold (degrees)')
    rate_arg = DeclareLaunchArgument(
        'interp_rate', default_value='50.0',
        description='traj_interpolator output rate (Hz)')

    moveit_config = (
        MoveItConfigsBuilder('roboticarm', package_name='roboticarm_moveit2')
        .to_moveit_configs()
    )

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', name))
        )

    # IK mover: direct setFromIK, publishes JointTrajectory
    ik_mover = Node(
        package='roboticarm_moveit2',
        executable='ik_mover',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Interpolator replaces the JointTrajectoryController (no ros2_control).
    traj_interpolator = Node(
        package='roboticarm_moveit2',
        executable='traj_interpolator',
        output='screen',
        parameters=[{'rate': LaunchConfiguration('interp_rate')}],
    )

    # Arduino bridge: /joint_states + /roboarm/gripper → serial
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
        rate_arg,

        include('static_virtual_joint_tfs.launch.py'),
        include('rsp.launch.py'),        # publishes robot_description + TFs
        ik_mover,
        traj_interpolator,
        bridge_node,
    ])
