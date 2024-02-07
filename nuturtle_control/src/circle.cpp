/// \file
/// \brief Control the turtle in a circular motion.
///
/// PARAMETERS:
///     frequency (double): frequency for message publishing (hz)
/// PUBLISHES:
///     cmd_vel (geometry_msgs::msg:Twist): velocity commands
/// SERVICES:
///     control (nuturtle_control_interfaces::srv::Control): control the trajectory
///     reverse (std_msgs::srv::Empty): reverses direction of robot
///     stop (std_msgs::srv::Empty): stops the trajectory execution


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control_interfaces/srv/control.hpp"

using namespace std::chrono_literals;


class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    // parameters declaration
    declare_parameter("frequency", 100.0);

    // calculate timer step
    std::chrono::milliseconds timerStep =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("frequency").as_double());

    // create main loop timer
    timer_ = create_wall_timer(
      timerStep, std::bind(&Circle::timerCallback, this));

    // create services
    control_service_ =
      create_service<nuturtle_control_interfaces::srv::Control>(
      "control",
      std::bind(&Circle::controlCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    reverse_service_ =
      create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverseCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    stop_service_ =
      create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stopCallback, this, std::placeholders::_1, std::placeholders::_2));

    // create publishers and subscribers
    vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);
    
  }

private:

  void timerCallback(){
    if (!stop_ack){
        if (stop_flg){
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            stop_ack = true;
            stop_flg = false;
        }
        vel_publisher_->publish(cmd_vel);
    }
  }

  void controlCallback(const std::shared_ptr<nuturtle_control_interfaces::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control_interfaces::srv::Control::Response>){
        stop_ack = false;
        cmd_vel.linear.x = request->velocity * request->radius;
        cmd_vel.angular.z = request->velocity;
  }

  void reverseCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>){
    cmd_vel.linear.x *= -1.0;
    cmd_vel.angular.z *= -1.0;
  }

  void stopCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>){
    stop_flg = true;
  }
 
  rclcpp::TimerBase::SharedPtr timer_;
  u_int64_t timestep_;
  geometry_msgs::msg::Twist cmd_vel;
  bool stop_flg;
  bool stop_ack;
  rclcpp::Service<nuturtle_control_interfaces::srv::Control>::SharedPtr control_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}