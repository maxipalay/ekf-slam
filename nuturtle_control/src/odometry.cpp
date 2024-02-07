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

    wheelRadius = get_parameter("wheel_radius").as_double();
    wheelTrack = get_parameter("track_width").as_double();
    bodyId = get_parameter("body_id").as_string();
    nameWheelLeft = get_parameter("wheel_left").as_string();
    nameWheelRight = get_parameter("wheel_right").as_string();
    odomId = get_parameter("odom_id").as_string();

    // calculate timer step
    std::chrono::milliseconds timerStep =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());

    // initialize odom data
    odomMsg.header.frame_id = odomId;
    odomMsg.child_frame_id = bodyId;
    odomTransform.header.frame_id = odomId;
    odomTransform.child_frame_id = bodyId;
    
    // instance diffDrive class
    ddrive = turtlelib::DiffDrive{wheelTrack, wheelRadius};

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create publishers and subscribers
    subJointStates_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::joints_cb, this, std::placeholders::_1));
    
    pubOdom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // create services
    pose_service_ =
      create_service<nuturtle_control_interfaces::srv::InitialPose>(
      "initial_pose",
      std::bind(&Odometry::pose_callback, this, std::placeholders::_1, std::placeholders::_2));

    // create main loop timer
    timer_ = create_wall_timer(
      timerStep, std::bind(&Odometry::timer_callback, this));
    
  }

private:
  void timer_callback(){
    pubOdom_->publish(odomMsg);
    tf_broadcaster_->sendTransform(odomTransform);
  }
 
  void pose_callback(
    const std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Response>
  ){
    // restart the ddrive class
    ddrive = turtlelib::DiffDrive{wheelTrack, wheelRadius};
    lastTransform = turtlelib::Transform2D{{request->x,request->y},request->theta};
    odomMsg.pose.pose.position.x = request->x;
    odomMsg.pose.pose.position.y = request->y;
    odomMsg.pose.pose.orientation.x = 0.0; // will always be zero in planar rotations
    odomMsg.pose.pose.orientation.y = 0.0; // will always be zero in planar rotations
    odomMsg.pose.pose.orientation.z = std::sin(request->theta/2.0);
    odomMsg.pose.pose.orientation.w = std::cos(request->theta/2.0);

    odomTransform.transform.translation.x = odomMsg.pose.pose.position.x;
    odomTransform.transform.translation.y = odomMsg.pose.pose.position.y;
    odomTransform.transform.rotation.x = odomMsg.pose.pose.orientation.x;
    odomTransform.transform.rotation.y = odomMsg.pose.pose.orientation.y;
    odomTransform.transform.rotation.z = odomMsg.pose.pose.orientation.z;
    odomTransform.transform.rotation.w = odomMsg.pose.pose.orientation.w;

  }

  void joints_cb(const sensor_msgs::msg::JointState & msg){
    auto timeNow = get_clock()->now();
    auto transform = ddrive.FKin(msg.position.at(0), msg.position.at(1));
    if (!first_joints_cb){
        auto dt = (timeNow - lastJointData).seconds();
        odomMsg.header.stamp = timeNow;
        odomMsg.pose.pose.position.x = transform.translation().x;
        odomMsg.pose.pose.position.y = transform.translation().y;
        odomMsg.pose.pose.orientation.x = 0.0; // will always be zero in planar rotations
        odomMsg.pose.pose.orientation.y = 0.0; // will always be zero in planar rotations
        odomMsg.pose.pose.orientation.z = std::sin(transform.rotation()/2.0);
        odomMsg.pose.pose.orientation.w = std::cos(transform.rotation()/2.0);
        odomMsg.twist.twist.linear.x = (transform.translation().x-lastTransform.translation().x)/(dt);
        odomMsg.twist.twist.linear.y = (transform.translation().y-lastTransform.translation().y)/(dt);
        odomMsg.twist.twist.angular.z = (transform.rotation()-lastTransform.rotation())/(dt);
        // pubOdom_->publish(odomMsg);

        odomTransform.header.stamp = timeNow;
        odomTransform.transform.translation.x = odomMsg.pose.pose.position.x;
        odomTransform.transform.translation.y = odomMsg.pose.pose.position.y;
        odomTransform.transform.rotation.x = odomMsg.pose.pose.orientation.x;
        odomTransform.transform.rotation.y = odomMsg.pose.pose.orientation.y;
        odomTransform.transform.rotation.z = odomMsg.pose.pose.orientation.z;
        odomTransform.transform.rotation.w = odomMsg.pose.pose.orientation.w;
        // tf_broadcaster_->sendTransform(odomTransform);
    } else {
        first_joints_cb = false;
    }
    lastJointData = timeNow;
    lastTransform = transform;
  }
  bool first_joints_cb = true;
  double wheelRadius;
  double wheelTrack;
  std::string bodyId;
  std::string nameWheelLeft;
  std::string nameWheelRight;
  std::string odomId;
  turtlelib::DiffDrive ddrive{wheelTrack, wheelRadius};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subJointStates_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry odomMsg{};
  geometry_msgs::msg::TransformStamped odomTransform;
  rclcpp::Time lastJointData;
  turtlelib::Transform2D lastTransform;
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