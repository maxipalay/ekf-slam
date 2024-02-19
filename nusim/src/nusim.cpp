/// \file
/// \brief Custom simulator core implementation.
///
/// PARAMETERS:
///     rate (double): frequency [hz] of updates of the simulation
///     x0 (double): initial x location [m] of the turtlebot
///     y0 (double): initial y location [m] of the turtlebot
///     theta0 (double): initial angle [rad] of the turtlebot
///     arena_x_length (double): x length [m] of the arena
///     arena_y_length (double): y length [m] of the arena
///     obstacles/x (std::vector<double>): x locations [m] of the obstacles
///     obstacles/y (std::vector<double>): y locations [m] of the obstacles
///     obstacles/r (std::vector<double>): radius [m] of the obstacles
///     wheel_radius (double): radius of the turtlebot's wheels
///     track_width (double): distance between the wheels
///     motor_cmd_per_rad_sec (double): motor command units per rad/s of speed for wheels
///     encoder_ticks_per_rad (double): number of encoder ticks per radian
///     input_noise (double): variance for noise injected into wheel commands
///     slip_fraction (double): limits for the uniform distribution for wheel position update noise
/// PUBLISHES:
///     ~/walls (visualization_msgs::msg::MarkerArray): marker array containing the markers of the environment walls
///     ~/obstacles (visualization_msgs::msg::MarkerArray): marker array containing the obstacles
///     ~/timestep (std_msgs::msg::UInt64): publishes the simulation time (timestep at rate frequency)
///     /red/sensor_data (nuturtlebot_msgs::msg::SensorData): simulated encoder data
/// SUBSCRIBES:
///     /red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): wheel commands
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): resets turtlebot location and simulation time
///     ~/teleport (nusim_interfaces::srv::Teleport): teleports the turtlebot to a new location

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nusim_interfaces/srv/teleport.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;


class NuSim : public rclcpp::Node
{
public:
  NuSim()
  : Node("nusim"), timestep_{0}
  {
    // parameters declaration
    declare_parameter("rate", 200.0);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("arena_x_length", 0.0);
    declare_parameter("arena_y_length", 0.0);
    declare_parameter("obstacles/x", std::vector<double>());
    declare_parameter("obstacles/y", std::vector<double>());
    declare_parameter("obstacles/r", std::vector<double>());
    declare_parameter("wheel_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("track_width", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("motor_cmd_per_rad_sec", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("encoder_ticks_per_rad", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("input_noise", 0.2);
    declare_parameter("slip_fraction", 0.1);

    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    input_noise = get_parameter("input_noise").as_double();
    slip_fraction = get_parameter("slip_fraction").as_double();
    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_track = get_parameter("track_width").as_double();

    // instance diffDrive class
    ddrive = turtlelib::DiffDrive{wheel_track, wheel_radius};

    // check if the obstacles arrays are of corresponding sizes
    check_obstacles();

    // create the reset service and bind it
    reset_service_ =
      create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&NuSim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // create the teleport service and bind it
    teleport_service_ =
      create_service<nusim_interfaces::srv::Teleport>(
      "~/teleport",
      std::bind(&NuSim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // set the turtle pose to intial pose, and set header
    turtle_pose_.header.frame_id = "nusim/world";
    turtle_pose_.child_frame_id = "red/base_footprint";
    set_initial_location();

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // calculate timer step
    std::chrono::milliseconds timerStep =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());

    //
    timestep_seconds = 1.0 / get_parameter("rate").as_double();

    // create time publisher
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // create main loop timer
    timer_ = create_wall_timer(
      timerStep, std::bind(&NuSim::timer_callback, this));

    // create QoS for markers publishers
    auto markers_qos_ = rclcpp::SystemDefaultsQoS{};
    markers_qos_.transient_local();

    // create walls publisher
    walls_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/walls",
      markers_qos_);

    // publish arena walls
    publish_arena_walls();

    // create obstacles publisher
    obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      markers_qos_);

    // create wheel cmd subscriber
    sub_wheel_cmd_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&NuSim::wheel_cmd_cb, this, std::placeholders::_1));

    // publish obstacles
    publish_obstacles();

    // sensor publisher
    sensor_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);

    gen = std::mt19937{rd()};

    uniform_dist = std::uniform_real_distribution{-slip_fraction, slip_fraction};
    normal_dist = std::normal_distribution{0.0, input_noise};

  }

private:
  // main loop
  void timer_callback()
  {
    // publish timestep update, and update timestep
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_;
    timestep_publisher_->publish(message);
    timestep_++;

    // output timestep to screen
    // RCLCPP_INFO_STREAM(get_logger(), timestep_);

    // update wheel positions, adding slip noise
    const double wheel_vel_l_noisy = wheel_vel_l*(1.0 + uniform_dist(gen));
    const double wheel_vel_r_noisy = wheel_vel_r*(1.0+ uniform_dist(gen));
    wheel_pos_l += wheel_vel_l_noisy * timestep_seconds;
    wheel_pos_r += wheel_vel_r_noisy * timestep_seconds;
    
    // get updated transform
    auto transform = ddrive.FKin(wheel_pos_l, wheel_pos_r);
    // update the transform to be broadcast
    turtle_pose_.header.stamp = get_clock()->now();
    turtle_pose_.transform.translation.x = transform.translation().x;
    turtle_pose_.transform.translation.y = transform.translation().y;
    turtle_pose_.transform.rotation.x = 0.0; // will always be zero in planar rotations
    turtle_pose_.transform.rotation.y = 0.0; // will always be zero in planar rotations
    turtle_pose_.transform.rotation.z = std::sin(transform.rotation() / 2.0);
    turtle_pose_.transform.rotation.w = std::cos(transform.rotation() / 2.0);
    // broadcast transform
    tf_broadcaster_->sendTransform(turtle_pose_);
    // update sensor data publishing
    auto sensor_msg = nuturtlebot_msgs::msg::SensorData{};
    sensor_msg.stamp = get_clock()->now();
    sensor_msg.left_encoder = encoder_ticks_per_rad * wheel_pos_l;//(wheel_pos_l+turtlelib::PI);
    sensor_msg.right_encoder = encoder_ticks_per_rad * wheel_pos_r;//(wheel_pos_r+turtlelib::PI);
    sensor_publisher_->publish(sensor_msg);
  }

  /// @brief callback for WheelCommand message subscription
  /// @param msg - nuturtlebot_msgs::msg::WheelCommands
  void wheel_cmd_cb(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    // cast message data to double
    auto wheel_vel_l_rads = static_cast<double>(msg.left_velocity);
    auto wheel_vel_r_rads = static_cast<double>(msg.right_velocity);

    // if commanded velocity is not zero, add noise
    if (wheel_vel_l_rads != 0.0){
        wheel_vel_l_rads += normal_dist(gen);
    }
    if (wheel_vel_r_rads != 0.0){
        wheel_vel_r_rads += normal_dist(gen);
    }

    // update node instance variables
    wheel_vel_l = wheel_vel_l_rads / motor_cmd_per_rad_sec;
    wheel_vel_r = wheel_vel_r_rads / motor_cmd_per_rad_sec;
  }

  // callback for the reset service
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // set the turtle back to initial location
    set_initial_location();
    // reset timestep
    timestep_ = 0;
  }

  // callback for the teleport service
  void teleport_callback(
    const std::shared_ptr<nusim_interfaces::srv::Teleport::Request> request,
    std::shared_ptr<nusim_interfaces::srv::Teleport::Response>)
  {
    // directly set the x, y location
    turtle_pose_.transform.translation.x = request->x;
    turtle_pose_.transform.translation.y = request->y;

    // translate input yaw into quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, request->theta);
    turtle_pose_.transform.rotation.x = q.x();
    turtle_pose_.transform.rotation.y = q.y();
    turtle_pose_.transform.rotation.z = q.z();
    turtle_pose_.transform.rotation.w = q.w();
  }

  // helper function to set the initial location of the turtle
  void set_initial_location()
  {
    // grab the initial location parameters
    turtle_pose_.transform.translation.x = get_parameter("x0").as_double();
    turtle_pose_.transform.translation.y = get_parameter("y0").as_double();
    // translate yaw into quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, get_parameter("theta0").as_double());
    // set quaternion
    turtle_pose_.transform.rotation.x = q.x();
    turtle_pose_.transform.rotation.y = q.y();
    turtle_pose_.transform.rotation.z = q.z();
    turtle_pose_.transform.rotation.w = q.w();
  }

  // helper function to create a wall marker
  visualization_msgs::msg::Marker create_wall(
    double scaleX, double scaleY, double scaleZ,
    double posX, double posY, double posZ, double id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = get_clock()->now();
    marker.ns = "";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = posX;
    marker.pose.position.y = posY;
    marker.pose.position.z = posZ;
    marker.scale.x = scaleX;
    marker.scale.y = scaleY;
    marker.scale.z = scaleZ;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    return marker;
  }

  // helper function to create an obstacle marker
  visualization_msgs::msg::Marker create_obstacle(
    double scaleX, double scaleY, double scaleZ,
    double posX, double posY, double posZ, double id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = get_clock()->now();
    marker.ns = "";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = posX;
    marker.pose.position.y = posY;
    marker.pose.position.z = posZ;
    marker.scale.x = scaleX;
    marker.scale.y = scaleY;
    marker.scale.z = scaleZ;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    return marker;
  }

  // helper function to check the lengths of the arguments obstacles/* are the same
  void check_obstacles()
  {
    auto obstacles_x = get_parameter("obstacles/x").as_double_array();
    auto obstacles_y = get_parameter("obstacles/y").as_double_array();
    auto obstacles_r = get_parameter("obstacles/r").as_double_array();
    if (obstacles_x.size() != obstacles_y.size() || obstacles_y.size() != obstacles_r.size()) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "dimension of obstacle arguments does not match" << std::endl);
      rclcpp::shutdown();
    }
  }

  // helper function to publish obstacles
  void publish_obstacles()
  {
    auto cylinder_height = 0.25;
    auto cylinder_z_pos = cylinder_height / 2.0;

    auto obstacles_x = get_parameter("obstacles/x").as_double_array();
    auto obstacles_y = get_parameter("obstacles/y").as_double_array();
    auto obstacles_r = get_parameter("obstacles/r").as_double_array();

    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < obstacles_x.size(); i++) {
      marker_array.markers.insert(
        marker_array.markers.end(),
        create_obstacle(
          obstacles_r[i] * 2.0, obstacles_r[i] * 2.0, cylinder_height, obstacles_x[i],
          obstacles_y[i], cylinder_z_pos, int(i)));
    }
    obstacles_publisher_->publish(marker_array);
  }

  // helper function to publish the arena walls
  void publish_arena_walls()
  {
    auto wall_thickness = 0.1;
    auto wall_height = 0.25;
    auto wall_z_pos = wall_height / 2.0;

    auto x_size = get_parameter("arena_x_length").as_double();
    auto y_size = get_parameter("arena_y_length").as_double();

    visualization_msgs::msg::MarkerArray marker_array;
    // top-down with x axis pointing up, this is the right wall
    marker_array.markers.insert(
      marker_array.markers.end(),
      create_wall(
        x_size + 2.0 * wall_thickness, wall_thickness, wall_height, 0,
        -(y_size + wall_thickness) / 2.0, wall_z_pos, 0));
    // top-down with x axis pointing up, this is the left wall
    marker_array.markers.insert(
      marker_array.markers.end(),
      create_wall(
        x_size + 2.0 * wall_thickness, wall_thickness, wall_height, 0,
        (y_size + wall_thickness) / 2.0, wall_z_pos, 1));
    // top-down with x axis pointing up, this is the top wall
    marker_array.markers.insert(
      marker_array.markers.end(),
      create_wall(
        wall_thickness, y_size + 2.0 * wall_thickness, wall_height,
        (x_size + wall_thickness) / 2.0, 0, wall_z_pos, 2));
    // top-down with x axis pointing up, this is the bottom wall
    marker_array.markers.insert(
      marker_array.markers.end(),
      create_wall(
        wall_thickness, y_size + 2.0 * wall_thickness, wall_height,
        -(x_size + wall_thickness) / 2.0, 0, wall_z_pos, 3));

    // publish
    walls_publisher_->publish(marker_array);
  }

  double timestep_seconds;
  turtlelib::DiffDrive ddrive{0.0, 0.0};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim_interfaces::srv::Teleport>::SharedPtr teleport_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped turtle_pose_;
  u_int64_t timestep_;

  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr sub_wheel_cmd_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_publisher_;
  double motor_cmd_per_rad_sec{};
  double wheel_vel_r{};
  double wheel_vel_l{};
  double wheel_pos_r{};
  double wheel_pos_l{};
  int encoder_ticks_per_rad{};
  double input_noise{};
  double slip_fraction{};
  double wheel_radius{};
  double wheel_track{};

  std::random_device rd{};
  std::mt19937 gen;
  std::normal_distribution<> normal_dist;
  std::uniform_real_distribution<> uniform_dist;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
