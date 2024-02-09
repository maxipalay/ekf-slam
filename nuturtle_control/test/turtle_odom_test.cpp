#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

TEST_CASE("turtle_odom_test_service", "[integration]") {
  // create an auxiliary node
  auto node = rclcpp::Node::make_shared("turtle_odom_test_aux_node");

  // Create a client for the service we're looking for
  auto client = node->create_client<nuturtle_control_interfaces::srv::InitialPose>("initial_pose");

  // create instance of the request
  auto request = std::make_shared<nuturtle_control_interfaces::srv::InitialPose::Request>();
  request->x = 1.0;
  request->y = 1.0;
  request->theta = 1.0;

  // wait for service to become available, with a timeout
  const auto SERVICE_TIMEOUT = 5;
  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(SERVICE_TIMEOUT) &&
    !client->wait_for_service(0s))
  )
  {
    // spin the node
    rclcpp::spin_some(node);
    // waiting for service;
  }
  bool service_found = client->wait_for_service(0s);
  REQUIRE(service_found);
  
  // send request
  auto result = client->async_send_request(request);

  auto success = false;
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    success = true;
  }

  REQUIRE(success);

}

TEST_CASE("turtle_odom_test_transform", "[integration]") {
  // create an auxiliary node
  auto node = rclcpp::Node::make_shared("turtle_odom_test_aux_node");
  
  // declare params so we can get the from the launchfile
  node->declare_parameter<std::string>("odom_id");
  node->declare_parameter<std::string>("body_id");
  
  // declare tf_buffer & listener
  auto tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // the transform that will be filled by the listener
  geometry_msgs::msg::TransformStamped t;
  
  // set a timeout. If we dont see the expected results within this time,
  // the test will fail
  const auto TEST_DURATION = 5;
  rclcpp::Time start_time = rclcpp::Clock().now();

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    // try getting the transform
    try {
      t = tf_buffer_->lookupTransform(
        node->get_parameter("body_id").as_string(), node->get_parameter("odom_id").as_string(),
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return;
    }
    // spin the node
    rclcpp::spin_some(node);
  }

  REQUIRE(t.transform.translation.x == 0.0);
  REQUIRE(t.transform.translation.y == 0.0);
  REQUIRE(t.transform.rotation.x == 0.0);
  REQUIRE(t.transform.rotation.y == 0.0);
  REQUIRE(t.transform.rotation.z == 0.0);
  REQUIRE(t.transform.rotation.w == 1.0);
}
