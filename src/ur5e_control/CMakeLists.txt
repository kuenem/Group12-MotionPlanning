cmake_minimum_required(VERSION 3.16)
project(ur5e_control)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(ur5e_moveit_control src/ur5e_moveit_control.cpp)

# MANUAL PATHS - replace with your actual paths
target_include_directories(ur5e_moveit_control PRIVATE
  /opt/ros/jazzy/include
  /opt/ros/jazzy/include/moveit
)

target_link_libraries(ur5e_moveit_control
  moveit_move_group_interface
  moveit_planning_interface
  moveit_ros_planning_interface
)

ament_target_dependencies(ur5e_moveit_control
  rclcpp
)

install(TARGETS ur5e_moveit_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


# cmake_minimum_required(VERSION 3.8)
# project(ur5e_control)

# # Default to C++17
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# # find_package(moveit_core REQUIRED)
# # find_package(moveit_ros_planning REQUIRED)
# # find_package(moveit_ros_move_group REQUIRED)

# # Find MoveIt include directories
# find_path(MOVEIT_INCLUDE_DIR "moveit/move_group_interface/move_group_interface.h"
#   PATHS /opt/ros/${ROS_DISTRO}/include
# )

# if(NOT MOVEIT_INCLUDE_DIR)
#   message(FATAL_ERROR "Failed to find MoveIt headers")
# endif()

# add_executable(ur5e_moveit_control src/ur5e_moveit_control.cpp)
# # target_include_directories(ur5e_moveit_control PRIVATE
# #   ${Boost_INCLUDE_DIRS}
# # )
# ament_target_dependencies(ur5e_moveit_control
#   rclcpp
# #   moveit_ros_planning_interface
# #   moveit_core
# #   moveit_ros_planning
# #   moveit_ros_move_group
# )

# target_link_libraries(ur5e_moveit_control
#   ${moveit_ros_planning_interface_LIBRARIES}    
# )

# install(TARGETS ur5e_moveit_control
#   DESTINATION lib/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
