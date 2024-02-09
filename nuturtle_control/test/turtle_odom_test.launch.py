from launch.substitutions import LaunchConfiguration
from launch_catch_ros2 import Catch2IntegrationTestNode, Catch2LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Launch example integration test."""
    # Catch2LaunchDescription:
    # a wrapper around LaunchDescription which adds a required "result_file"
    # argument for the launch file. This file will hold the results of the test.
    return Catch2LaunchDescription([
        SetLaunchConfiguration(
            'robot_params_pkg',
            value='nuturtle_description'
            ),
        SetLaunchConfiguration(
            'robot_params_path',
            value='config/diff_params.yaml'
            ),
        Node(
            package='nuturtle_control',
            executable='odometry',
            parameters=[
                ParameterFile(PathJoinSubstitution(
                    [FindPackageShare(LaunchConfiguration('robot_params_pkg')),
                     LaunchConfiguration('robot_params_path')])),
                {"body_id": "blue/base_footprint",
                 "odom_id": "odom",
                 "wheel_left": "blue/wheel_left_joint",
                 "wheel_right": "blue/wheel_right_joint"}],
            ),
        # Catch2IntegrationTestNode:
        # a wrapper around Node which passes the "result_file" argument to Catch2.
        # There should only be one integration test node. This node will shutdown
        # the entire launch file when it exits.
        # Specific parameters and other arguments can also be passed, like the
        # "test_duration" example below.
        Catch2IntegrationTestNode(
            package='nuturtle_control',
            executable='turtle_odom_test',
            parameters=[
                {"body_id": "blue/base_footprint",
                 "odom_id": "odom"}
            ]
        ),
    ])
