/// \file
/// \brief Custom simulator core implementation.
///
/// PARAMETERS:
///     wheel_radius (double): 
///     track_width (double):   
///     motor_cmd_max (double):  
///     motor_cmd_per_rad_sec (double):
///     encoder_ticks_per_rad (double):

/// PUBLISHES:
/// SUBSCRIBES:

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
    //declare_parameter("rate", 100.0);

    wheelRadius = get_parameter("wheel_radius").as_double();
    wheelTrack = get_parameter("track_width").as_double();
    motorCmdMax = get_parameter("motor_cmd_max").as_int();
    motorCmdPerRadSec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoderTicksPerRad = get_parameter("encoder_ticks_per_rad").as_double();

    // instance diffDrive class
    ddrive = turtlelib::DiffDrive{wheelTrack, wheelRadius};

    // // calculate timer step
    // std::chrono::milliseconds timerStep =
    //   (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());
    // // we'll store the timestep in seconds
    // timestep_ = 1.0/get_parameter("rate").as_double();

    // // create main loop timer
    // timer_ = create_wall_timer(
    //   timerStep, std::bind(&TurtleControl::timer_callback, this));

    // create publishers and subscribers

    pubWheelCmd_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    pubJointStates_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    subCmdVel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::vel_cb, this, std::placeholders::_1));
    
    subSensorData_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_cb, this, std::placeholders::_1));
    
  }

private:
  // main loop
  void timer_callback()
  {

    // output timestep to screen
    RCLCPP_INFO_STREAM(get_logger(), "timer!");

  }

  void vel_cb(const geometry_msgs::msg::Twist & msg){
    // create a 2D Twist
    auto twist = turtlelib::Twist2D{msg.angular.z, msg.linear.x, msg.linear.y};
    // convert to wheel speeds (rad/s)
    auto wheelSpeedsRad = ddrive.IKin(twist);
    // convert to wheel speed in motor command units (mcu)
    auto cmdLeft = int(std::round(wheelSpeedsRad.left * motorCmdPerRadSec));
    auto cmdRight = int(std::round(wheelSpeedsRad.right * motorCmdPerRadSec));
    // limit speeds
    cmdLeft = limit_speed(cmdLeft);
    cmdRight = limit_speed(cmdRight);
    // publish message
    publish_wheel_commands(cmdLeft, cmdRight);
  }

  void publish_wheel_commands(const int left, const int right){
    // construct message
    nuturtlebot_msgs::msg::WheelCommands cmdMsg;
    cmdMsg.left_velocity = left;
    cmdMsg.right_velocity = right;
    // publish
    pubWheelCmd_->publish(cmdMsg);
  }

  int limit_speed(const int wheelCmd){
    if (wheelCmd > motorCmdMax){
        return motorCmdMax;
    }
    if (wheelCmd < -motorCmdMax){
        return -motorCmdMax;
    }
    return wheelCmd;
  }

  void sensor_cb(const nuturtlebot_msgs::msg::SensorData & msg){
    auto timeNow = get_clock()->now(); // register current time
    // transform ticks to wheel angles
    double leftAngleRad = msg.left_encoder / encoderTicksPerRad;
    double rightAngleRad = msg.right_encoder / encoderTicksPerRad;

    // get the difference in ticks between now and previous message
    auto leftTickDiff = msg.left_encoder - lastLeftEncoderTicks;
    auto rightTickDiff = msg.right_encoder - lastRightEncoderTicks;
    // turn this difference into radians
    double leftRadDiff = leftTickDiff / encoderTicksPerRad;
    double rightRadDiff = rightTickDiff / encoderTicksPerRad;
    auto leftSpeed = leftRadDiff / (timeNow - lastSensorData).seconds();
    auto rightSpeed = rightRadDiff / (timeNow - lastSensorData).seconds();

    lastLeftEncoderTicks = msg.left_encoder;
    lastRightEncoderTicks = msg.right_encoder;
    lastSensorData = timeNow;
    
    publish_joint_states(leftAngleRad, rightAngleRad, leftSpeed, rightSpeed);
  }

  void publish_joint_states(double leftPosRad, double rightPosRad, 
                            double leftSpeedRads, double rightSpeedRads){
    sensor_msgs::msg::JointState msg;

    msg.name = std::vector<std::string>({"wheel_left_joint", "wheel_right_joint"});

    msg.position = std::vector<double>({leftPosRad,rightPosRad});

    msg.velocity = std::vector<double>({leftSpeedRads, rightSpeedRads});

    pubJointStates_->publish(msg);
  }

  
  double wheelRadius;
  double wheelTrack;
  int motorCmdMax;
  double motorCmdPerRadSec;
  double encoderTicksPerRad;
  double timestep_;

  turtlelib::DiffDrive ddrive{wheelTrack, wheelRadius};
  //rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr pubWheelCmd_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr subSensorData_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointStates_;
  rclcpp::Time lastSensorData;
  int32_t lastLeftEncoderTicks{0};
  int32_t lastRightEncoderTicks{0};

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}