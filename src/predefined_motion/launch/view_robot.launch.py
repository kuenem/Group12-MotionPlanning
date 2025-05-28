from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
import os
import sys
import launch
import xacro

def wait_for_user_input(context, *args, **kwargs):
    print("\n=== Please start the External Control Program on the UR5e robot ===")
    input("Press [Enter] to continue once it's started... ")
    return []


def generate_launch_description():
    # For launching connection with ur5e
    ur_launch_file = os.path.join(
        '/opt/ros/jazzy/share/ur_robot_driver/launch',
        'ur_control.launch.py'
    )

    moveit_launch = os.path.join(
        '/opt/ros/jazzy/share/ur_moveit_config/launch',
        'ur_moveit.launch.py'
    )

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            'ur_type': 'ur5e',
            'launch_rviz': 'true'
        }.items()
    )

    # Path to the test goal publishers config
    position_goals = PathJoinSubstitution(
        #[FindPackageShare("ur_robot_driver"), "config", "test_goal_publishers_config.yaml"]
        [FindPackageShare("ur_robot_driver"), "config", "our_funny_config.yaml"]
    )
   

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_launch_file),
            launch_arguments={
                'ur_type': 'ur5e',
                'robot_ip': '192.168.1.5',  # ← robot IP
                # 'headless_mode': 'true'
            }.items()
        ),


        # MoveIt launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                'ur_type': 'ur5e',
                'launch_rviz': 'true'
            }.items()
        ),

       
        # Test goal publisher
        Node(
            package="ros2_controllers_test_nodes",
            executable="publisher_joint_trajectory_controller",
            name="publisher_scaled_joint_trajectory_controller",
            parameters=[position_goals],
            output="screen"
        )
    ])
    
