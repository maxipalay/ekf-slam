#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;

TEST_CASE("turtle_control_wheel_cmd_pure_translation", "[integration]") {
  // create an auxiliary node
  auto node = rclcpp::Node::make_shared("turtle_control_test_aux_node");
  // create a shared pointer to a wheel commands message.
  // the node will update it
  auto wheel_cmd = std::make_shared<nuturtlebot_msgs::msg::WheelCommands>();
  // create subscription to the topic we want to test. Note were capturing
  // a reference to wheel_cmd so the callback can modify it.
  auto sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, [&wheel_cmd](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
      wheel_cmd->right_velocity = msg->right_velocity;
      wheel_cmd->left_velocity = msg->left_velocity;
    });
  // create a publisher to publish cmd_vel messages
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // define what will be published
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.1;
  // define what we expect
  nuturtlebot_msgs::msg::WheelCommands expected;
  expected.right_velocity = 126;
  expected.left_velocity = 126;

  // set a timeout. If we dont see the expected results within this time,
  // the test will fail
  const auto TEST_DURATION = 5;
  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    // publish
    pub->publish(cmd_vel);
    // spin the node
    rclcpp::spin_some(node);
    // if we already receive the expected values, end the test
    if (wheel_cmd->right_velocity == expected.right_velocity &&
      wheel_cmd->left_velocity == expected.left_velocity)
    {
      break;
    }
  }

  REQUIRE(wheel_cmd->right_velocity == expected.right_velocity);
  REQUIRE(wheel_cmd->left_velocity == expected.left_velocity);

}

TEST_CASE("turtle_control_wheel_cmd_pure_rotation", "[integration]") {
  // create an auxiliary node
  auto node = rclcpp::Node::make_shared("turtle_control_test_aux_node");
  // create a shared pointer to a wheel commands message.
  // the node will update it
  auto wheel_cmd = std::make_shared<nuturtlebot_msgs::msg::WheelCommands>();
  // create subscription to the topic we want to test. Note were capturing
  // a reference to wheel_cmd so the callback can modify it.
  auto sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, [&wheel_cmd](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
      wheel_cmd->right_velocity = msg->right_velocity;
      wheel_cmd->left_velocity = msg->left_velocity;
    });
  // create a publisher to publish cmd_vel messages
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // define what will be published
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.angular.z = 1.0;
  // define what we expect
  nuturtlebot_msgs::msg::WheelCommands expected;
  expected.right_velocity = 101;
  expected.left_velocity = -101;

  // set a timeout. If we dont see the expected results within this time,
  // the test will fail
  const auto TEST_DURATION = 5;
  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    // publish
    pub->publish(cmd_vel);
    // spin the node
    rclcpp::spin_some(node);
    // if we already receive the expected values, end the test
    if (wheel_cmd->right_velocity == expected.right_velocity &&
      wheel_cmd->left_velocity == expected.left_velocity)
    {
      break;
    }
  }

  REQUIRE(wheel_cmd->right_velocity == expected.right_velocity);
  REQUIRE(wheel_cmd->left_velocity == expected.left_velocity);

}

TEST_CASE("turtle_control_joint_states", "[integration]") {
  // create an auxiliary node
  auto node = rclcpp::Node::make_shared("turtle_control_test_aux_node");
  // create a shared pointer to a wheel commands message.
  // the node will update it
  auto joint_state = std::make_shared<sensor_msgs::msg::JointState>();
  joint_state->position = {0.0, 0.0};
  // create subscription to the topic we want to test. Note were capturing
  // a reference to joint_state so the callback can modify it.
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [&joint_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
      joint_state->position = msg->position;
    });
  // create a publisher to publish cmd_vel messages
  auto pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
  // define what will be published
  nuturtlebot_msgs::msg::SensorData sensor_msg;
  sensor_msg.left_encoder = 0;
  sensor_msg.right_encoder = 0;
  // publish a few times zeros, so the node sets zero as the encoder offset
  for (int i = 0; i < 10; i++) {
    pub->publish(sensor_msg);
    // spin the node
    rclcpp::spin_some(node);
  }
  // set values we want to test
  sensor_msg.left_encoder = 2047;
  // define what we expect
  sensor_msgs::msg::JointState expected;
  expected.position = std::vector<double>({turtlelib::PI, 0.0});

  // set a timeout. If we dont see the expected results within this time,
  // the test will fail
  const auto TEST_DURATION = 5;
  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    // publish
    pub->publish(sensor_msg);
    // spin the node
    rclcpp::spin_some(node);
    // if we already receive the expected values, end the test
    if (turtlelib::almost_equal(
        joint_state->position.at(0), expected.position.at(0),
        0.1) && turtlelib::almost_equal(joint_state->position.at(1), expected.position.at(1), 0.1))
    {
      break;
    }
  }

  REQUIRE_THAT(
    joint_state->position.at(0),
    Catch::Matchers::WithinAbs(turtlelib::normalize_angle(expected.position.at(0)), 1.0E-1));
  REQUIRE_THAT(
    joint_state->position.at(1),
    Catch::Matchers::WithinAbs(expected.position.at(1), 1.0E-1));
}
