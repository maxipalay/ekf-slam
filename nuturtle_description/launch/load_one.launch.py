import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration,\
    PythonExpression


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_jsp',
            default_value='true',
            description="controls whether the joint_state_publisher is used\
                  to publish default joint states.",
            choices = ["true", "false"]),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description="controls whether rviz is launched.",
            choices = ["true", "false"]),
        DeclareLaunchArgument(
            'color',
            default_value='purple',
            description="Determines the color that is passed to the xacro\
                  file as an argument.",
            choices = ["red", "green", "blue", "purple"]),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='basic_purple.rviz',
            description="rviz configuration file."),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {"robot_description":
                 Command([ExecutableInPackage("xacro", "xacro"), " ",
                         PathJoinSubstitution(
                    [FindPackageShare("nuturtle_description"), "urdf",
                     "turtlebot3_burger.urdf.xacro"]), " color:=",
                     LaunchConfiguration('color')])}
            ]
        ),
        Node(
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('use_jsp'), '\'', '== \'true\''])),
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("nuturtle_description"), "config/",
                     LaunchConfiguration('rviz_config')])],
            on_exit=launch.actions.Shutdown()
        )
    ])
