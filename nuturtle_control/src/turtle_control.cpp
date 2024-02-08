/// \file
/// \brief Turtle control implementation.
///
/// PARAMETERS:
///     wheel_radius (double): radius of the turtlebot's wheels
///     track_width (double): distance between the wheels
///     motor_cmd_max (integer): motor command absolute maximum limit
///     motor_cmd_per_rad_sec (double): motor command units per rad/s of speed for wheels
///     encoder_ticks_per_rad (double): number of encoder ticks per radian

/// PUBLISHES:
///     wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): wheel commands for the turtlebot
///     joint_states (sensor_msgs::msg::JointState): wheels position
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::msg::Twist): motion commands
///     sensor_data (nuturtlebot_msgs::msg::SensorData): encoder information

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;


class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    // parameters declaration

    declare_parameter("wheel_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("track_width", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("motor_cmd_max", rclcpp::ParameterType::PARAMETER_INTEGER);
    declare_parameter("motor_cmd_per_rad_sec", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("encoder_ticks_per_rad", rclcpp::ParameterType::PARAMETER_DOUBLE);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_track = get_parameter("track_width").as_double();
    motor_cmd_max = get_parameter("motor_cmd_max").as_int();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();

    // instance diffDrive class
    ddrive = turtlelib::DiffDrive{wheel_track, wheel_radius};

    // create publishers and subscribers
    pub_wheel_cmd_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::CmdVelCallback, this, std::placeholders::_1));
    
    sub_sensor_data_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::SensorDataCallback, this, std::placeholders::_1));
    
  }

private:

  void CmdVelCallback(const geometry_msgs::msg::Twist & msg){
    // create a 2D Twist
    auto twist = turtlelib::Twist2D{msg.angular.z, msg.linear.x, msg.linear.y};
    // convert to wheel speeds (rad/s)
    auto wheel_speeds_rad = ddrive.IKin(twist);
    // convert to wheel speed in motor command units (mcu)
    auto cmd_left = int(std::round(wheel_speeds_rad.left * motor_cmd_per_rad_sec));
    auto cmd_right = int(std::round(wheel_speeds_rad.right * motor_cmd_per_rad_sec));
    // limit speeds
    cmd_left = limit_speed(cmd_left);
    cmd_right = limit_speed(cmd_right);
    // publish message
    publish_wheel_commands(cmd_left, cmd_right);
  }

  void publish_wheel_commands(const int left, const int right){
    // construct message
    nuturtlebot_msgs::msg::WheelCommands cmd_msg;
    cmd_msg.left_velocity = left;
    cmd_msg.right_velocity = right;
    // publish
    pub_wheel_cmd_->publish(cmd_msg);
  }

  int limit_speed(const int wheel_cmd){
    if (wheel_cmd > motor_cmd_max){
        return motor_cmd_max;
    }
    if (wheel_cmd < -motor_cmd_max){
        return -motor_cmd_max;
    }
    return wheel_cmd;
  }

  void SensorDataCallback(const nuturtlebot_msgs::msg::SensorData & msg){
    auto timeNow = get_clock()->now(); // register current time
    
    
    if (!first_sensor_cb){
        double left_encoder = msg.left_encoder - encoder_offset_left;
        double right_encoder = msg.right_encoder - encoder_offset_right;
        // transform ticks to wheel angles
        double left_angle_rad = left_encoder / encoder_ticks_per_rad; // - turtlelib::PI;
        double right_angle_rad = right_encoder / encoder_ticks_per_rad; // - turtlelib::PI;
        // get the difference in ticks between now and previous message
        auto left_ticks_diff = left_encoder - prev_left_encoder_ticks;
        auto right_ticks_diff = right_encoder - prev_right_encoder_ticks;
        // turn this difference into radians
        //double left_rad_diff = turtlelib::normalize_angle(left_ticks_diff / encoder_ticks_per_rad);
        //double right_rad_diff = turtlelib::normalize_angle(right_ticks_diff / encoder_ticks_per_rad);
        double left_rad_diff = left_ticks_diff / encoder_ticks_per_rad;
        double right_rad_diff = right_ticks_diff / encoder_ticks_per_rad;

        auto leftSpeed = left_rad_diff / (timeNow - time_last_sensor_data).seconds();
        auto rightSpeed = right_rad_diff / (timeNow - time_last_sensor_data).seconds();
        publish_joint_states(timeNow, left_angle_rad, right_angle_rad, leftSpeed, rightSpeed);
        prev_left_encoder_ticks = left_encoder;
        prev_right_encoder_ticks = right_encoder;
    } else {
        first_sensor_cb = false;
        encoder_offset_left = msg.left_encoder;
        encoder_offset_right = msg.right_encoder;
    }
    
    time_last_sensor_data = timeNow;
  }

  void publish_joint_states(const rclcpp::Time & time, const double left_pos_rad, const double right_pos_rad, 
                            const double left_speed_rads, const double right_speed_rads){
    sensor_msgs::msg::JointState msg;

    msg.header.stamp = time;
    msg.name = std::vector<std::string>({"wheel_left_joint", "wheel_right_joint"});

    msg.position = std::vector<double>({left_pos_rad,right_pos_rad});

    msg.velocity = std::vector<double>({left_speed_rads, right_speed_rads});

    pub_joint_states_->publish(msg);
  }

  
  double wheel_radius;
  double wheel_track;
  int motor_cmd_max;
  double motor_cmd_per_rad_sec;
  double encoder_ticks_per_rad;
  bool first_sensor_cb = true;
  int encoder_offset_left;
  int encoder_offset_right;
  turtlelib::DiffDrive ddrive{wheel_track, wheel_radius};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr pub_wheel_cmd_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sub_sensor_data_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Time time_last_sensor_data;
  int32_t prev_left_encoder_ticks{0};
  int32_t prev_right_encoder_ticks{0};

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}