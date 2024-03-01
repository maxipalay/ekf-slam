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
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_src',
            description="control the source of velocity commands.",
            default_value="teleop",
            choices=["circle", "teleop", "none"]),
        DeclareLaunchArgument(
            'robot',
            description="select running a robot simulation or running on an actual robot.",
            choices=["nusim", "localhost", "none"]),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description="controls whether rviz is launched.",
            choices=["true", "false"]),
        SetLaunchConfiguration(
            'rviz_config',
            value='config.rviz',
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('robot'), '\'', '== \'nusim\''])),
            ),
        SetLaunchConfiguration(
            'rviz_config',
            value='config_blue.rviz',
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('robot'), '\'', '== \'none\''])),
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
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('robot'), '\'', '== \'nusim\''])),
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
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('robot'), '\'', '== \'nusim\''
                 ' or ', '\'', LaunchConfiguration('robot'), '\'', '== \'localhost\''])),
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
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('robot'), '\'', '== \'nusim\'',
                 ' or ', '\'', LaunchConfiguration('robot'), '\'', '== \'localhost\''])),
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
                 "wheel_right": "green/wheel_right_joint"}
            ],
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('robot'), '\'', '== \'nusim\'',
                 ' or ', '\'', LaunchConfiguration('robot'), '\'', '== \'localhost\''])),
            remappings=[('/joint_states', '/blue/joint_states')]
        ),
        Node(
            package='numsr_turtlebot',
            executable='numsr_turtlebot',
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('robot'), '\'', '== \'localhost\'']))
        ),
        Node(
            package='nuturtle_control',
            executable='circle',
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('cmd_src'), '\'', '== \'circle\''])),
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
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('use_rviz'), '\'', '== \'true\'', ' and ',
                 '\'', LaunchConfiguration('robot'), '\'', '== \'nusim\'', ' or ',
                 '\'', LaunchConfiguration('use_rviz'), '\'', '== \'true\'', ' and ',
                 '\'', LaunchConfiguration('robot'), '\'', '== \'none\''])),
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
                    condition=IfCondition(PythonExpression(
                        ['\'', LaunchConfiguration('robot'), '\'', '== \'nusim\''])),
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
                    condition=IfCondition(PythonExpression(
                        ['\'', LaunchConfiguration('robot'), '\'', '== \'nusim\'',
                         ' or ', '\'', LaunchConfiguration('robot'), '\'', '== \'none\''])),
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
                    condition=IfCondition(PythonExpression(
                        ['\'', LaunchConfiguration('robot'), '\'', '== \'nusim\'',
                         ' or ', '\'', LaunchConfiguration('robot'), '\'', '== \'none\''])),
                ),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("hls_lfcd_lds_driver"), '/launch',
                 '/hlds_laser.launch.py']
            ),
            launch_arguments=[["port", "/dev/ttyUSB0"]],
            condition=IfCondition(PythonExpression(
                ['\'', LaunchConfiguration('robot'), '\'', '== \'localhost\''])),
        ),
    ])
