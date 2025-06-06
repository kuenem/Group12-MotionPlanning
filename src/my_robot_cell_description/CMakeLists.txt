cmake_minimum_required(VERSION 3.8)
project(my_robot_cell_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

find_package(controller_manager_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)

# find_package(controller_manager_msgs REQUIRED)

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

# target_link_libraries(ur5e_moveit_controller
#   moveit_core::moveit_core
#   moveit_ros_planning_interface::moveit_ros_planning_interface
# )

ament_package()

# install(TARGETS ur5e_moveit_controller
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY srdf/
  DESTINATION share/${PROJECT_NAME}/srdf
)


