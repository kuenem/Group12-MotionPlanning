#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
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


// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// // #include <controller_manager_msgs/srv/switch_controller.hpp>


// int main(int argc, char *argv[])
// {
//     std::cout << "1============================================================================================1" << std::endl;

//   // Initialize ROS 2
//   rclcpp::init(argc, argv);
//   // RCLCPP_INFO(node->get_logger(), "Main start...");
//   // auto node = rclcpp::Node::make_shared("simple_moveit_motion");
//   auto const node = std::make_shared<rclcpp::Node>(
//     "ur5e_moveit_controller",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//     // auto const node = rclcpp::Node::make_shared(
//     // "ur5e_moveit_controller",
//     // rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//   );
  

//     // Create a ROS logger
//     auto const logger = rclcpp::get_logger("ur5e_moveit_controller");
//     std::cout << "2============================================================================================2" << std::endl;

//     // Wait for MoveIt to initialize
//     if (!rclcpp::ok()) {
//         RCLCPP_ERROR(logger, "ROS shutdown while initializing");
//         return 1;
//     }

//     // Create the MoveIt MoveGroup Interface
//     // using moveit::planning_interface::MoveGroupInterface;
//     std::cout << "3============================================================================================3" << std::endl;
//     // auto move_group_interface = MoveGroupInterface(node, "ur_manipulator"); //change
//       // Initialize MoveGroupInterface with proper error handling
//     moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
//     if (!move_group.getRobotModel()) {
//       RCLCPP_ERROR(logger, "Failed to initialize MoveGroupInterface");
//       return 1;
//     }

//     // // After move_group initialization
//     // if (!move_group.getController()->isActive()) {
//     //     RCLCPP_INFO(logger, "Activating controller...");
//     //     controller_manager_msgs::srv::SwitchController::Request::SharedPtr request;
//     //     auto client = node->create_client<controller_manager_msgs::srv::SwitchController>(
//     //         "/controller_manager/switch_controller");
//     //     request->strictness = 2;
//     //     request->activate_controllers = {"scaled_joint_trajectory_controller"};
//     //     client->async_send_request(request);
//     // }

//     std::cout << "4============================================================================================4" << std::endl;
//     rclcpp::sleep_for(std::chrono::seconds(10));
//     std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

//     // After initializing move_group_interface, add:
//     // auto controller_manager = node->create_client<controller_manager_msgs::srv::SwitchController>(
//     //     "/controller_manager/switch_controller"
//     // );

//     // auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
//     // request->activate_controllers = {"scaled_joint_trajectory_controller"};
//     // request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

//     // auto future = controller_manager->async_send_request(request);
//     // if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
//     //     RCLCPP_ERROR(logger, "Failed to activate controller");
//     //     return 1;
//     // }

//     // In your node:
//     // rclcpp::sleep_for(std::chrono::seconds(15));  // Wait for controller activation
//     std::cout << "TheFancyThingWeWantedToMake++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;


//   // Configure planner parameters
//   move_group.setPlanningTime(10.0);
//   std::cout << "setPlanningTime++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
//   move_group.setNumPlanningAttempts(5);
//   std::cout << "setNumPlanningAttemps++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
//   move_group.setPlannerId("RRTConnect");
//   // move_group.setPlannerId("PTP");
//   std::cout << "setPlannerId++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;


//     // Set a target Pose

//     // geometry_msgs::msg::Pose target_pose;
//     // target_pose.orientation.w = 0.002;
//     // target_pose.position.x = -0.002;
//     // target_pose.position.y = -0.709;
//     // target_pose.position.z = 0.705;
//     geometry_msgs::msg::Pose target_pose;
//     target_pose.orientation.w = 1.0;
//     target_pose.position.x = 0.28;
//     target_pose.position.y = -0.2;
//     target_pose.position.z = 0.5;

//     std::cout << "settargetPose-----------------------------------------------------------------------------------------" << std::endl;


//   // Verify target is reachable
//   if (!move_group.setPoseTarget(target_pose)) {
//     RCLCPP_ERROR(logger, "Invalid target pose");
//     return 1;
//   }
//   std::cout << "setPoseTarget-----------------------------------------------------------------------------------------" << std::endl;


//   // Plan and execute
//   moveit::planning_interface::MoveGroupInterface::Plan plan;
//   if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
//     RCLCPP_INFO(logger, "Planning successful, executing...");
//     move_group.execute(plan);
//   } else {
//     RCLCPP_ERROR(logger, "Planning failed");
//     return 1;
//   }

//   std::cout << "planAndExecute-----------------------------------------------------------------------------------------" << std::endl;


//   rclcpp::shutdown();
//   return 0;
// }
