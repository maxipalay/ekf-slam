import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [FindPackageShare("nuturtle_description"), '/launch',
                         '/load_one.launch.py']
                    ),
                    launch_arguments=[["use_rviz", "false"], ["color", "red"]]
            ),
        DeclareLaunchArgument(
            'config',
            default_value='basic_world.yaml',
            description="allows the user to specify a yaml file as configuration."),
        SetLaunchConfiguration(
            'rviz_config',
            value='nusim.rviz'
            ),
        Node(
            package='nusim',
            executable='nusim',
            parameters=[ParameterFile(PathJoinSubstitution(
                    [FindPackageShare("nusim"), "config/", LaunchConfiguration('config')]))]
            ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("nusim"),"config/",
                     LaunchConfiguration('rviz_config')]),
            ],
            on_exit=launch.actions.Shutdown()
        )
    ])
