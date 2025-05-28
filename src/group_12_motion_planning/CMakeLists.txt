cmake_minimum_required(VERSION 3.8)
project(group_12_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
# find_package(controller_manager_msgs REQUIRED)
# find_package(graph_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
# find_package(quaternion_utils REQUIRED)

add_executable(ur5e_moveit_controller
  src/ur5e_moveit_controller.cpp
)

# Add this after add_executable()
include_directories(
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# # Combine all dependencies in a single target_link_libraries call
target_link_libraries(ur5e_moveit_controller PRIVATE
  rclcpp::rclcpp
  # std_msgs::std_msgs
  moveit_ros_planning_interface::move_group_interface
  moveit_visual_tools::moveit_visual_tools
  moveit_core::moveit_kinematics_base
  moveit_core::moveit_common
  moveit_core::moveit_collision_detection
  moveit_msgs::moveit_msgs
  # geometry_msgs::geometry_msgs
)

# Link against all required targets
# Link against all required targets using correct names
# target_link_libraries(ur5e_moveit_controller PRIVATE
#   rclcpp::rclcpp
#   geometry_msgs::geometry_msgs
#   moveit_core::moveit_common
#   moveit_ros_planning_interface::move_group_interface
#   moveit_visual_tools::moveit_visual_tools
# )



install(TARGETS ur5e_moveit_controller
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
