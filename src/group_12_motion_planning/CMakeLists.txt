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

# target_include_directories(ur5e_moveit_controller PRIVATE
#   include
# )

ament_target_dependencies(ur5e_moveit_controller
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  moveit_core
#   controller_manager_msgs
)

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


# ament_target_dependencies(ur5e_moveit_controller
#   rclcpp
#   moveit_ros_planning_interface
#   controller_manager_msgs
#   graph_msgs 
#   moveit_visual_tools
# )


# install(TARGETS ur5e_moveit_controller
#   DESTINATION lib/${PROJECT_NAME}
# )
