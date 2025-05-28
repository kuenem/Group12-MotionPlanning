#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char *argv[])
{
    std::cout << "1============================================================================================1" << std::endl;

  // Initialize ROS 2
  rclcpp::init(argc, argv);
  // RCLCPP_INFO(node->get_logger(), "Main start...");
  // auto node = rclcpp::Node::make_shared("simple_moveit_motion");
  auto const node = std::make_shared<rclcpp::Node>(
    "ur5e_moveit_controller",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    // auto const node = rclcpp::Node::make_shared(
    // "ur5e_moveit_controller",
    // rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("ur5e_moveit_controller");
    std::cout << "2============================================================================================2" << std::endl;

    // Wait for MoveIt to initialize
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(logger, "ROS shutdown while initializing");
        return 1;
    }

    // Create the MoveIt MoveGroup Interface
    // using moveit::planning_interface::MoveGroupInterface;
    std::cout << "3============================================================================================3" << std::endl;
    // auto move_group_interface = MoveGroupInterface(node, "ur_manipulator"); //change
      // Initialize MoveGroupInterface with proper error handling
    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    if (!move_group.getRobotModel()) {
      RCLCPP_ERROR(logger, "Failed to initialize MoveGroupInterface");
      return 1;
    }
    std::cout << "4============================================================================================4" << std::endl;
    rclcpp::sleep_for(std::chrono::seconds(10));
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

  // Configure planner parameters
  move_group.setPlanningTime(10.0);
  std::cout << "setPlanningTime++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  move_group.setNumPlanningAttempts(5);
  std::cout << "setNumPlanningAttemps++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  move_group.setPlannerId("PTP");
  std::cout << "setPlannerId++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;


    // Set a target Pose

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = -0.2;
    target_pose.position.z = 0.5;

    std::cout << "settargetPose-----------------------------------------------------------------------------------------" << std::endl;


  // Verify target is reachable
  if (!move_group.setPoseTarget(target_pose)) {
    RCLCPP_ERROR(logger, "Invalid target pose");
    return 1;
  }
  std::cout << "setPoseTarget-----------------------------------------------------------------------------------------" << std::endl;


  // Plan and execute
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger, "Planning successful, executing...");
    move_group.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed");
    return 1;
  }

  std::cout << "planAndExecute-----------------------------------------------------------------------------------------" << std::endl;


  rclcpp::shutdown();
  return 0;
}