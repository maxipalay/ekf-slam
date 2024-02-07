import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration,\
    PythonExpression
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_src',
            description="control the source of velocity commands.",
            choices=["circle", "teleop", "none"]),
        DeclareLaunchArgument(
            'robot',
            description="select running a robot simulation or running on an actual robot.",
            choices=["nusim", "localhost", "none"]),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description="controls whether rviz is launched.",
            choices=["true", "false"]),
        SetLaunchConfiguration(
            'rviz_config',
            value='config.rviz'
            ),
        SetLaunchConfiguration(
            'robot_params_pkg',
            value='nuturtle_description'
            ),
        SetLaunchConfiguration(
            'robot_params_path',
            value='config/diff_params.yaml'
            ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "nusim/world", "--child-frame-id",
                       "odom", "--x", "0.0", "--y", "0.0"],
            ),
        Node(
            package='nusim',
            executable='nusim',
            parameters=[ParameterFile(PathJoinSubstitution(
                    [FindPackageShare("nusim"), "config/", 'basic_world.yaml'])),
                    ParameterFile(PathJoinSubstitution(
                    [FindPackageShare(LaunchConfiguration('robot_params_pkg')),
                     LaunchConfiguration('robot_params_path')]))
                    ],
            remappings=[('/red/sensor_data', '/sensor_data')]
            ),
        Node(
            package='nuturtle_control',
            executable='turtle_control',
            parameters=[
                ParameterFile(PathJoinSubstitution(
                    [FindPackageShare(LaunchConfiguration('robot_params_pkg')),
                     LaunchConfiguration('robot_params_path')]))],
            remappings=[('/wheel_cmd', '/red/wheel_cmd'),
                        ('/joint_states', '/blue/joint_states')]
        ),    
        Node(
            package='nuturtle_control',
            executable='odometry',
            parameters=[
                ParameterFile(PathJoinSubstitution(
                    [FindPackageShare(LaunchConfiguration('robot_params_pkg')),
                     LaunchConfiguration('robot_params_path')])),
                {"body_id":"blue/base_footprint",
                 "odom_id":"odom",
                 "wheel_left":"blue/wheel_left_joint",
                 "wheel_right":"blue/wheel_right_joint"}
            ],
            remappings=[('/joint_states', '/blue/joint_states')]
        ),
        Node(
            package='nuturtle_control',
            executable='circle',
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('cmd_src'), '\'', '== \'circle\''])),
        ),
        # Node(
        #     package='turtlebot3_teleop',
        #     executable='teleop_twist_keyboard',
        #     condition=IfCondition(PythonExpression(
        #         ['\'', LaunchConfiguration('cmd_src'), '\'', '== \'teleop\''])),
        # ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('use_rviz'), '\'', '== \'true\'',' and ',
                 '\'', LaunchConfiguration('robot'), '\'', '== \'nusim\''])),
            arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("nuturtle_control"), "config/",
                     LaunchConfiguration('rviz_config')]),
            ],
            on_exit=launch.actions.Shutdown()
        ),
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     condition=IfCondition(PythonExpression(
        #         ['\'', LaunchConfiguration('use_rviz'), '\'', '== \'true\'',' and ',
        #          '\'', LaunchConfiguration('robot'), '\'', '!= \'nusim\''])),
        #     arguments=["-d", PathJoinSubstitution(
        #             [FindPackageShare("nuturtle_control"), "config/",
        #              LaunchConfiguration('rviz_config')]),
        #     ],
        #     on_exit=launch.actions.Shutdown()
        # ),
        GroupAction(
            actions=[
                # push_ros_namespace to set namespace of included nodes
                PushRosNamespace('red'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [FindPackageShare("nuturtle_description"), '/launch',
                         '/load_one.launch.py']
                    ),
                    launch_arguments=[["use_rviz", "false"], ["color", "red"], ["use_jsp", "true"]]
                    ),
                ]
        ),
        GroupAction(
            actions=[
                # push_ros_namespace to set namespace of included nodes
                PushRosNamespace('blue'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [FindPackageShare("nuturtle_description"), '/launch',
                         '/load_one.launch.py']
                    ),
                    launch_arguments=[["use_rviz", "false"], ["color", "blue"], ["use_jsp", "false"]]
                    ),
                ]
        ),                
        
    ])
