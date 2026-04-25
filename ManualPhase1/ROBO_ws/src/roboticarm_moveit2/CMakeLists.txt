cmake_minimum_required(VERSION 3.22)
project(roboticarm_moveit2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)   # pulls in moveit_core + robot_model_loader

# ── ik_mover node ──────────────────────────────────────────────────────────────
add_executable(ik_mover src/ik_mover.cpp)
ament_target_dependencies(ik_mover
  rclcpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_ros_planning_interface
)

install(TARGETS ik_mover
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python scripts (pose_publisher) ───────────────────────────────────────────
install(PROGRAMS
  scripts/pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# ── Config / launch / assets ──────────────────────────────────────────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
