import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
from geometry_msgs.msg import Twist
import time


@pytest.mark.rostest
def generate_test_description():
    action = Node(package="nuturtle_control",
                  executable="circle")
    return (
        LaunchDescription([
            action,
            launch_testing.actions.ReadyToTest()
        ]),
        # These are extra parameters that get passed to the test functions
        {
            'robot_node': action
        }
    )


class TestPublisherFrequency(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")
        self.last_timestamp = 0
        self.timestamp_sum = 0
        self.timestamp_count = 0
        self.last_time = None

    def tearDown(self):
        self.node.destroy_node()

    def sub_cb(self, msg):
        this_time = time.time()
        if self.last_time:
            self.timestamp_sum += this_time - self.last_time
            self.timestamp_count += 1
        self.last_time = this_time

    def test_frequency(self, launch_service, robot_node, proc_output):
        self.node.create_subscription(Twist, 'cmd_vel', self.sub_cb, 10)

        while self.timestamp_count < 10:
            rclpy.spin_once(self.node)

        mean_timediff = self.timestamp_sum/self.timestamp_count

        assert abs((1.0 / mean_timediff) - 100.0) < 1.0
