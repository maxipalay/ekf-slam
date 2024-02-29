/// \file
/// \brief Turtle odometry.
///
/// PARAMETERS:
///     wheel_radius (double): radius of the turtlebot's wheels
///     track_width (double): distance between the wheels
///     body_id (string): name of the body frame of the robot
///     odom_id (string): name of the odometry frame
///     wheel_left (string): name of the left wheel link
///     wheel_right (string): name of the right wheel link
///     rate (double): rate at which odometry is published (default 100)
/// PUBLISHES:
///     odom (nav_msgs::msg::Odometry): turtlebot odometry
///     tf (tf2 transform broadcast): transform between odom frame and body frame
/// SUBSCRIBES:
///     joint_states (sensor_msgs::msg::JointState): wheels position
/// SERVICES:
///     initial_pose (nuturtle_control_interfaces::srv::InitialPose): motion commands


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/se2d.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtle_control_interfaces/srv/initial_pose.hpp"

using namespace std::chrono_literals;


class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    // parameters declaration
    declare_parameter("wheel_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("track_width", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("body_id", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("wheel_left", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("wheel_right", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("odom_id", "odom");
    declare_parameter("rate", 100.0);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_track = get_parameter("track_width").as_double();
    body_id = get_parameter("body_id").as_string();
    name_wheel_left = get_parameter("wheel_left").as_string();
    name_Wheel_right = get_parameter("wheel_right").as_string();
    odom_id = get_parameter("odom_id").as_string();

    // calculate timer step
    std::chrono::milliseconds timer_step =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());

    // initialize odom data
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;
    odom_transform.header.frame_id = odom_id;
    odom_transform.child_frame_id = body_id;

    // instance diffDrive class
    ddrive = turtlelib::DiffDrive{wheel_track, wheel_radius};

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create publishers and subscribers
    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::jointStateCallback, this, std::placeholders::_1));

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // create services
    pose_service_ =
      create_service<nuturtle_control_interfaces::srv::InitialPose>(
      "initial_pose",
      std::bind(&Odometry::poseCallback, this, std::placeholders::_1, std::placeholders::_2));

    // create main loop timer
    timer_ = create_wall_timer(
      timer_step, std::bind(&Odometry::timerCallback, this));

  }

private:
  void timerCallback()
  {
    pub_odom_->publish(odom_msg);
    tf_broadcaster_->sendTransform(odom_transform);
  }

  void poseCallback(
    const std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Response>
  )
  {
    // restart the ddrive class
    ddrive = turtlelib::DiffDrive{wheel_track, wheel_radius};
    prev_transform = turtlelib::Transform2D{{request->x, request->y}, request->theta};
    odom_msg.pose.pose.position.x = request->x;
    odom_msg.pose.pose.position.y = request->y;
    odom_msg.pose.pose.orientation.x = 0.0; // will always be zero in planar rotations
    odom_msg.pose.pose.orientation.y = 0.0; // will always be zero in planar rotations
    odom_msg.pose.pose.orientation.z = std::sin(request->theta / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos(request->theta / 2.0);

    odom_transform.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_transform.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_transform.transform.rotation.x = odom_msg.pose.pose.orientation.x;
    odom_transform.transform.rotation.y = odom_msg.pose.pose.orientation.y;
    odom_transform.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    odom_transform.transform.rotation.w = odom_msg.pose.pose.orientation.w;

  }

  void jointStateCallback(const sensor_msgs::msg::JointState & msg)
  {
    auto time_now = rclcpp::Time(msg.header.stamp);
    // we should be using the names provided (name_wheel_left, name_Wheel_right)
    // to find out the indices of those in the vector
    auto transform = ddrive.FKin(msg.position.at(0), msg.position.at(1));
    if (!first_joints_cb) {
      auto dt = (time_now - time_last_joint_data).seconds();
      odom_msg.header.stamp = msg.header.stamp;
      odom_msg.pose.pose.position.x = transform.translation().x;
      odom_msg.pose.pose.position.y = transform.translation().y;
      odom_msg.pose.pose.orientation.x = 0.0;   // will always be zero in planar rotations
      odom_msg.pose.pose.orientation.y = 0.0;   // will always be zero in planar rotations
      odom_msg.pose.pose.orientation.z = std::sin(transform.rotation() / 2.0);
      odom_msg.pose.pose.orientation.w = std::cos(transform.rotation() / 2.0);
      odom_msg.twist.twist.linear.x = (transform.translation().x - prev_transform.translation().x) /
        (dt);
      odom_msg.twist.twist.linear.y = (transform.translation().y - prev_transform.translation().y) /
        (dt);
      odom_msg.twist.twist.angular.z = (turtlelib::normalize_angle(transform.rotation()) - turtlelib::normalize_angle(prev_transform.rotation())) / (dt);
      // pub_odom_->publish(odom_msg);

      odom_transform.header.stamp = msg.header.stamp;
      odom_transform.transform.translation.x = odom_msg.pose.pose.position.x;
      odom_transform.transform.translation.y = odom_msg.pose.pose.position.y;
      odom_transform.transform.rotation.x = odom_msg.pose.pose.orientation.x;
      odom_transform.transform.rotation.y = odom_msg.pose.pose.orientation.y;
      odom_transform.transform.rotation.z = odom_msg.pose.pose.orientation.z;
      odom_transform.transform.rotation.w = odom_msg.pose.pose.orientation.w;
      // tf_broadcaster_->sendTransform(odom_transform);
    } else {
      first_joints_cb = false;
    }
    time_last_joint_data = msg.header.stamp;
    prev_transform = transform;
  }
  bool first_joints_cb = true;
  double wheel_radius;
  double wheel_track;
  std::string body_id;
  std::string name_wheel_left;
  std::string name_Wheel_right;
  std::string odom_id;
  turtlelib::DiffDrive ddrive{wheel_track, wheel_radius};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry odom_msg{};
  geometry_msgs::msg::TransformStamped odom_transform;
  rclcpp::Time time_last_joint_data;
  turtlelib::Transform2D prev_transform;
  rclcpp::Service<nuturtle_control_interfaces::srv::InitialPose>::SharedPtr pose_service_;
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
