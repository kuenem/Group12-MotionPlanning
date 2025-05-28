from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

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


        # New ==========================================
def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=192.168.1.5",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur5e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )


    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("my_robot_cell_description"),
                "srdf",
                "ur.srdf.xacro"
            ]),
            " ",
            "name:=ur"
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic
        # New ==========================================


def generate_launch_description():
    # For launching connection with ur5e
    ur_launch_file = os.path.join(
        '/opt/ros/jazzy/share/ur_robot_driver/launch',
        'ur_control.launch.py'
    )

    # New ==========================================

    kinematics_config = PathJoinSubstitution([
    FindPackageShare("my_robot_cell_description"),
    "config",
    "kinematics.yaml"
    ])

    # New ==========================================

    moveit_launch = os.path.join(
        '/opt/ros/jazzy/share/ur_moveit_config/launch',
        'ur_moveit.launch.py'
    )

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            'ur_type': 'ur5e',
            'launch_rviz': 'false'
        }.items()
    )

    

    # Path to the test goal publishers config
    position_goals = PathJoinSubstitution(
        #[FindPackageShare("ur_robot_driver"), "config", "test_goal_publishers_config.yaml"]
        [FindPackageShare("ur_robot_driver"), "config", "our_funny_config.yaml"]
    )
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
   

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_launch_file),
            launch_arguments={
                'ur_type': 'ur5e',
                'robot_ip': '192.168.1.5',  # ← robot IP
                # 'headless_mode': 'true'
            }.items()
        ),

        # # Delay and wait for user input
        # TimerAction(
        #     period=5.0,  # Adjust delay if needed
        #     actions=[
        #         OpaqueFunction(function=wait_for_user_input)
        #     ]
        # ),

        # MoveIt launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                'ur_type': 'ur5e',
                'launch_rviz': 'false',
                'robot_description': robot_description['robot_description'],  # Pass your robot description
                'robot_description_semantic': robot_description_semantic['robot_description_semantic'],  # Pass semantic
                'kinematics_config': kinematics_config
            }.items()
        ),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher'
        # ),

        # Node(
        #     package='move_group',
        #     executable='move_group',
        #     output='screen',
        #     parameters=[..., PathJoinSubstitution([
        #         FindPackageShare("my_robot_cell_description"),
        #         "config",
        #         "kinematics.yaml"
        #     ])]
        # ),


        # New ==========================================

        # Node(
        #     package='my_robot_cell_description',
        #     executable='move_to_joint_goal',
        #     name='move_to_joint_goal',
        #     output='screen'
        # ),

        # New ==========================================

        TimerAction(
        period=8.0,  # Increase if needed
        actions=[
        Node(
            package='group_12_motion_planning',
            executable='ur5e_moveit_controller',
            name='ur5e_moveit_controller',
            output='screen',
            parameters=[
            robot_description,
            robot_description_semantic,
            ##############new
            kinematics_config,
            ####new
        ],
        ),
        ]
        ),

       
        # Test goal publisher
        # Node(
        #     package="ros2_controllers_test_nodes",
        #     executable="publisher_joint_trajectory_controller",
        #     name="publisher_scaled_joint_trajectory_controller",
        #     parameters=[position_goals],
        #     output="screen"
        # )
    ])

# from launch import LaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction
# from launch_ros.actions import Node
# from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.parameter_descriptions import ParameterFile

# # from launch.actions import DeclareLaunchArgument
# # from launch.substitutions import LaunchConfiguration
# import os
# import sys
# import launch
# import xacro

# def wait_for_user_input(context, *args, **kwargs):
#     print("\n=== Please start the External Control Program on the UR5e robot ===")
#     input("Press [Enter] to continue once it's started... ")
#     return []


#         # New ==========================================
# def get_robot_description():
#     joint_limit_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"]
#     )
#     kinematics_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
#     )
#     physical_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"]
#     )
#     visual_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"]
#     )
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
#             " ",
#             "robot_ip:=192.168.1.5",
#             " ",
#             "joint_limit_params:=",
#             joint_limit_params,
#             " ",
#             "kinematics_params:=",
#             kinematics_params,
#             " ",
#             "physical_params:=",
#             physical_params,
#             " ",
#             "visual_params:=",
#             visual_params,
#             " ",
#            "safety_limits:=",
#             "true",
#             " ",
#             "safety_pos_margin:=",
#             "0.15",
#             " ",
#             "safety_k_position:=",
#             "20",
#             " ",
#             "name:=",
#             "ur",
#             " ",
#             "ur_type:=",
#             "ur5e",
#             " ",
#             "prefix:=",
#             '""',
#             " ",
#         ]
#     )


#     robot_description = {"robot_description": robot_description_content}
#     return robot_description

# def get_robot_description_semantic():
#     # MoveIt Configuration
#     robot_description_semantic_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution([
#                 FindPackageShare("my_robot_cell_description"),
#                 "srdf",
#                 "ur.srdf.xacro"
#             ]),
#             " ",
#             "name:=ur"
#         ]
#     )
#     robot_description_semantic = {
#         "robot_description_semantic": robot_description_semantic_content
#     }
#     return robot_description_semantic
#         # New ==========================================


# def generate_launch_description():
#     # For launching connection with ur5e
#     ur_launch_file = os.path.join(
#         '/opt/ros/jazzy/share/ur_robot_driver/launch',
#         'ur_control.launch.py'
#     )

#     # New ==========================================

#     kinematics_config = PathJoinSubstitution([
#     FindPackageShare("my_robot_cell_description"),
#     "config",
#     "kinematics.yaml"
#     ])

#     # New ==========================================

#     moveit_launch = os.path.join(
#         '/opt/ros/jazzy/share/ur_moveit_config/launch',
#         'ur_moveit.launch.py'
#     )

#     IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(moveit_launch),
#         launch_arguments={
#             'ur_type': 'ur5e',
#             'launch_rviz': 'true'
#         }.items()
#     )

    

#     # Path to the test goal publishers config
#     position_goals = PathJoinSubstitution(
#         #[FindPackageShare("ur_robot_driver"), "config", "test_goal_publishers_config.yaml"]
#         [FindPackageShare("ur_robot_driver"), "config", "our_funny_config.yaml"]
#     )
#     robot_description = get_robot_description()
#     robot_description_semantic = get_robot_description_semantic()
   

#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(ur_launch_file),
#             launch_arguments={
#                 'ur_type': 'ur5e',
#                 'robot_ip': '192.168.1.5',  # ← robot IP
#                 'initial_joint_controller': 'scaled_joint_trajectory_controller',
#                 'activate_joint_controller': 'true',
#                 'headless_mode': 'false'
#             }.items()
#         ),

#         # # Delay and wait for user input
#         # TimerAction(
#         #     period=5.0,  # Adjust delay if needed
#         #     actions=[
#         #         OpaqueFunction(function=wait_for_user_input)
#         #     ]
#         # ),

#         # MoveIt launch
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(moveit_launch),
#             launch_arguments={
#                 'ur_type': 'ur5e',
#                 'launch_rviz': 'false',
#                 'robot_description': robot_description['robot_description'],  # Pass your robot description
#                 'robot_description_semantic': robot_description_semantic['robot_description_semantic'],  # Pass semantic
#                 'kinematics_config': kinematics_config
#             }.items()
#         ),

#         # Node(
#         #     package='joint_state_publisher',
#         #     executable='joint_state_publisher',
#         #     name='joint_state_publisher'
#         # ),

#         # Node(
#         #     package='move_group',
#         #     executable='move_group',
#         #     output='screen',
#         #     parameters=[..., PathJoinSubstitution([
#         #         FindPackageShare("my_robot_cell_description"),
#         #         "config",
#         #         "kinematics.yaml"
#         #     ])]
#         # ),


#         # New ==========================================

#         # Node(
#         #     package='my_robot_cell_description',
#         #     executable='move_to_joint_goal',
#         #     name='move_to_joint_goal',
#         #     output='screen'
#         # ),

#         # New ==========================================

#         TimerAction(
#         period=8.0,  # Increase if needed
#         actions=[
#         Node(
#             package='group_12_motion_planning',
#             executable='ur5e_moveit_controller',
#             name='ur5e_moveit_controller',
#             output='screen',
#             parameters=[
#             robot_description,
#             robot_description_semantic,
#             ##############new
#             kinematics_config,
#             ####new
#         ],
#         ),
#         ]
#         ),

       
#         # Test goal publisher
#         # Node(
#         #     package="ros2_controllers_test_nodes",
#         #     executable="publisher_joint_trajectory_controller",
#         #     name="publisher_scaled_joint_trajectory_controller",
#         #     parameters=[position_goals],
#         #     output="screen"
#         # )
#     ])
    
