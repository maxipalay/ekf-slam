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


def generate_launch_description():
    return LaunchDescription([
        SetLaunchConfiguration(
            'rviz_config',
            value='basic_all.rviz'
            ),

        GroupAction(
            actions=[
                # push_ros_namespace to set namespace of included nodes
                PushRosNamespace('red'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [FindPackageShare("nuturtle_description"), '/launch',
                         '/load_one.launch.py']
                    ),
                    launch_arguments=[["use_rviz", "false"], ["color", "red"]]
                    ),
                ]
            ),
        GroupAction(
            actions=[
                # push_ros_namespace to set namespace of included nodes
                PushRosNamespace('green'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [FindPackageShare("nuturtle_description"), '/launch',
                         '/load_one.launch.py']
                    ),
                    launch_arguments=[["use_rviz", "false"], ["color", "green"]]
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
                    launch_arguments=[["use_rviz", "false"], ["color", "blue"]]
                    ),
                ]
            ),
        GroupAction(
            actions=[
                # push_ros_namespace to set namespace of included nodes
                PushRosNamespace('purple'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [FindPackageShare("nuturtle_description"), '/launch',
                         '/load_one.launch.py']
                    ),
                    launch_arguments=[["use_rviz", "false"], ["color", "purple"]]
                    ),
                ]
            ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "nusim/world", "--child-frame-id",
                       "red/base_footprint", "--x", "0.3"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "nusim/world", "--child-frame-id",
                       "green/base_footprint", "--y", "0.6"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "nusim/world", "--child-frame-id",
                       "blue/base_footprint", "--x", "-0.71"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "nusim/world", "--child-frame-id",
                       "purple/base_footprint", "--y", "-0.9"],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("nuturtle_description"), "config/",
                     LaunchConfiguration('rviz_config')])
                     ],
            on_exit=launch.actions.Shutdown()
        )
    ])
