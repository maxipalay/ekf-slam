from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration,\
        PythonExpression
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_src',
            description="control the source of velocity commands.",
            default_value="teleop",
            choices=["circle", "teleop", "none"]),
        SetLaunchConfiguration(
            'robot',
            value='localhost',
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
            package='numsr_turtlebot',
            executable='numsr_turtlebot',
        ),
        Node(
            package='nuturtle_control',
            executable='circle',
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('cmd_src'), '\'', '== \'circle\''])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("hls_lfcd_lds_driver"), '/launch',
                 '/hlds_laser.launch.py']
            ),
            launch_arguments=[["port", "/dev/ttyUSB0"], ["frame_id", "green/base_scan"]]
        ),
    ])
