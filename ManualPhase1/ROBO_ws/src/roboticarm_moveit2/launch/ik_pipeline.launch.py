"""
Non-GUI IK pipeline launch file.

Starts the full MoveIt2 stack (no RViz) plus the ik_mover node.
Send a target pose on /target_pose and the arm will plan + execute.

  ros2 launch roboticarm_moveit2 ik_pipeline.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = get_package_share_directory("roboticarm_moveit2")

    moveit_config = (
        MoveItConfigsBuilder("roboticarm", package_name="roboticarm_moveit2")
        .to_moveit_configs()
    )

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", name))
        )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    ik_mover = Node(
        package="roboticarm_moveit2",
        executable="ik_mover",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        include("static_virtual_joint_tfs.launch.py"),
        include("rsp.launch.py"),
        ros2_control_node,
        include("move_group.launch.py"),
        include("spawn_controllers.launch.py"),
        ik_mover,
    ])
