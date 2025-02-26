import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration,\
        PythonExpression
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    return LaunchDescription([
        SetLaunchConfiguration(
            'cmd_src',
            value='teleop',
            ),
        SetLaunchConfiguration(
            'rviz_config',
            value='config.rviz',
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
                       "blue/odom", "--x", "0.0", "--y", "0.0"],
            ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "nusim/world", "--child-frame-id",
                       "map", "--x", "0.0", "--y", "0.0"],
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
            remappings=[('/red/sensor_data', '/sensor_data'),
                        ('/red/wheel_cmd', '/wheel_cmd')]
            ),
        Node(
            package='nuturtle_control',
            executable='turtle_control',
            parameters=[
                ParameterFile(PathJoinSubstitution(
                    [FindPackageShare(LaunchConfiguration('robot_params_pkg')),
                     LaunchConfiguration('robot_params_path')]))],
            remappings=[('/joint_states', '/blue/joint_states')],
        ),
        Node(
            package='nuturtle_control',
            executable='odometry',
            parameters=[
                ParameterFile(PathJoinSubstitution(
                    [FindPackageShare(LaunchConfiguration('robot_params_pkg')),
                     LaunchConfiguration('robot_params_path')])),
                {"body_id": "blue/base_footprint",
                 "odom_id": "blue/odom",
                 "wheel_left": "blue/wheel_left_joint",
                 "wheel_right": "blue/wheel_right_joint"}
            ],
            remappings=[('/joint_states', '/blue/joint_states')]
        ),
        Node(
            package='nuslam',
            executable='slam',
            parameters=[
                ParameterFile(PathJoinSubstitution(
                    [FindPackageShare(LaunchConfiguration('robot_params_pkg')),
                     LaunchConfiguration('robot_params_path')])),
                {"body_id": "green/base_footprint",
                 "odom_id": "green/odom",
                 "wheel_left": "green/wheel_left_joint",
                 "wheel_right": "green/wheel_right_joint",
                 "sensor_source": "assoc"}
            ],
            remappings=[('/joint_states', '/blue/joint_states')]
        ),
        Node(
            package='nuslam',
            executable='landmarks'
        ),
        # Begin_Citation [1] #
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('cmd_src'), '\'', '== \'teleop\''])),
            output='screen',
            prefix='xterm -e'
        ),
        # End_Citation [1]  #
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("nuslam"), "config/",
                     LaunchConfiguration('rviz_config')]),
            ],
            on_exit=launch.actions.Shutdown()
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
                    launch_arguments=[["use_rviz", "false"], ["color", "red"],
                                      ["use_jsp", "true"]],
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
                    launch_arguments=[["use_rviz", "false"], ["color", "blue"],
                                      ["use_jsp", "false"]],
                ),
            ],
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
                    launch_arguments=[["use_rviz", "false"], ["color", "green"],
                                      ["use_jsp", "true"]],
                ),
            ],
        ),
    ])
