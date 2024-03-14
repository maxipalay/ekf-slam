/// \file
/// \brief Turtle EKF SLAM.
///
/// PARAMETERS:
///     wheel_radius (double): radius of the turtlebot's wheels
///     track_width (double): distance between the wheels
///     body_id (string): name of the body frame of the robot
///     odom_id (string): name of the odometry frame
///     wheel_left (string): name of the left wheel link
///     wheel_right (string): name of the right wheel link
///     rate (double): rate at which odometry is published (default 100)
///     path_rate (double): rate at which the path is published
/// PUBLISHES:
///     odom (nav_msgs::msg::Odometry): turtlebot odometry
///     tf (tf2 transform broadcast): transform between odom frame and body frame
///     /red/path (nav_msgs::msg::Path): path followed by the robot in the map frame
/// SUBSCRIBES:
///     joint_states (sensor_msgs::msg::JointState): wheels position
/// SERVICES:
///     initial_pose (nuturtle_control_interfaces::srv::InitialPose): motion commands

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

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
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;


class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    // parameters declaration
    declare_parameter("wheel_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("track_width", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("body_id", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("wheel_left", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("wheel_right", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("odom_id", "odom");
    declare_parameter("rate", 100.0);
    declare_parameter("path_rate", 5.0);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_track = get_parameter("track_width").as_double();
    body_id = get_parameter("body_id").as_string();
    name_wheel_left = get_parameter("wheel_left").as_string();
    name_Wheel_right = get_parameter("wheel_right").as_string();
    odom_id = get_parameter("odom_id").as_string();

    // calculate timer step
    std::chrono::milliseconds timer_step =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());

    std::chrono::milliseconds path_timer_step =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("path_rate").as_double());


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
      "joint_states", 10, std::bind(&Slam::jointStateCallback, this, std::placeholders::_1));

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("green/odom", 10);

    // create services
    pose_service_ =
      create_service<nuturtle_control_interfaces::srv::InitialPose>(
      "initial_pose",
      std::bind(&Slam::poseCallback, this, std::placeholders::_1, std::placeholders::_2));

    // create main loop timer
    timer_ = create_wall_timer(
      timer_step, std::bind(&Slam::timerCallback, this));

    timer_path_ = create_wall_timer(
      path_timer_step, std::bind(&Slam::pathTimerCallback, this));

    // create QoS for markers publishers
    auto sensor_qos_ = rclcpp::SystemDefaultsQoS{};
    sensor_qos_.transient_local();

    sub_fake_sensor_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "nusim/fake_sensor", sensor_qos_, std::bind(
        &Slam::fake_sensor_cb, this,
        std::placeholders::_1));

    // initialize slam variables
    sigma_zero = arma::mat(2 * n_landmarks + 3, 2 * n_landmarks + 3, arma::fill::zeros);
    sigma_zero.submat(3, 3, 2 * n_landmarks + 2, 2 * n_landmarks + 2).eye();
    sigma_zero.submat(3, 3, 2 * n_landmarks + 2, 2 * n_landmarks + 2) *= 10e6;
    sigma_hat = sigma_zero;
    // calculate Q bar (equation 22)
    q_bar = arma::mat(2 * n_landmarks + 3, 2 * n_landmarks + 3, arma::fill::zeros);
    q_bar.submat(0, 0, 2, 2) = arma::mat(3, 3, arma::fill::eye) * q_noise;
    R = arma::mat(2, 2, arma::fill::zeros);
    R = arma::mat(2, 2, arma::fill::eye) * r_noise;

    path_publisher_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);
    path_msg = nav_msgs::msg::Path();
    path_msg.header.frame_id = "map";

    map_transform = geometry_msgs::msg::TransformStamped();
    map_transform.header.frame_id = "map";
    map_transform.child_frame_id = odom_id;

    // create QoS for markers publishers
    auto markers_qos_ = rclcpp::SystemDefaultsQoS{};
    markers_qos_.transient_local();

    sensed_obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "green/obstacles",
      markers_qos_);


  }

private:
  void pathTimerCallback()
  {
    // publish path
    auto time_now = get_clock()->now();
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.frame_id = "map";
    pose.header.stamp = time_now;
    auto current_configuration = t_map_odom * t_odom_robot;
    pose.pose.position.x = current_configuration.translation().x;
    pose.pose.position.y = current_configuration.translation().y;
    pose.pose.orientation.x = 0.0; // will always be zero in planar rotations
    pose.pose.orientation.y = 0.0; // will always be zero in planar rotations
    pose.pose.orientation.z = std::sin(current_configuration.rotation() / 2.0);
    pose.pose.orientation.w = std::cos(current_configuration.rotation() / 2.0);
    path_msg.poses.insert(path_msg.poses.end(), pose);
    path_msg.header.stamp = time_now;
    path_publisher_->publish(path_msg);
  }

  void fake_sensor_cb(const visualization_msgs::msg::MarkerArray & msg)
  {
    fake_sensor_data_ = msg;

  }

  void timerCallback()
  {
    pub_odom_->publish(odom_msg);
    tf_broadcaster_->sendTransform(odom_transform);

    // get the time
    auto time_now = get_clock()->now();
    if (fake_sensor_data_.markers.size() > 0) {
      // EKF update

      // get the current configuration
      auto current_configuration = t_map_odom * t_odom_robot;

      // tranlsation difference, dx, dy
      double dx = current_configuration.translation().x -
        filter_previous_configuration.translation().x;
      double dy = current_configuration.translation().y -
        filter_previous_configuration.translation().y;

      // update the state
      state(0) = turtlelib::normalize_angle(current_configuration.rotation());
      state(1) = current_configuration.translation().x;
      state(2) = current_configuration.translation().y;

      // calculate A_t (equation 10)
      arma::mat a_t(2 * n_landmarks + 3, 2 * n_landmarks + 3, arma::fill::eye);
      a_t(1, 0) = -dy;
      a_t(2, 0) = dx;

      // equation 21
      sigma_hat = a_t * sigma_hat * a_t.t() + q_bar;

      // update the state with the measurements
      for (size_t i = 0; i < fake_sensor_data_.markers.size(); i++) {
        auto marker = fake_sensor_data_.markers.at(i);
        if (marker.action != visualization_msgs::msg::Marker::DELETE) {
          // RCLCPP_INFO_STREAM(get_logger(), "obstacle action: "<<marker.action <<std::endl);
          double x_bar = marker.pose.position.x;
          double y_bar = marker.pose.position.y;

          // measured range and bearing
          double range = std::sqrt(std::pow(x_bar, 2) + std::pow(y_bar, 2));
          double bearing = std::atan2(y_bar, x_bar);
          arma::vec z_t = {range, bearing};

          // initialize the obstacle if we havent seen it before
          if (state(2 * marker.id + 3) == 0.0 && state(2 * marker.id + 3 + 1) == 0.0) {
            state(2 * marker.id + 3) = state(1) + range *
              std::cos(turtlelib::normalize_angle(bearing) + state(0));
            state(2 * marker.id + 3 + 1) = state(2) + range *
              std::sin(turtlelib::normalize_angle(bearing) + state(0));
          }

          // if we were at state(1), state(2), what would be the measurements? (measurement model)
          double r_estimated =
            std::sqrt(
            std::pow(
              state(2 * marker.id + 3) - state(1),
              2) + std::pow(state(2 * marker.id + 3 + 1) - state(2), 2));
          double b_estimated =
            std::atan2(
            state(2 * marker.id + 3 + 1) - state(2),
            state(2 * marker.id + 3) - state(1)) - state(0);
          b_estimated = turtlelib::normalize_angle(b_estimated);

            RCLCPP_INFO_STREAM(get_logger(), "measured range: "<< range <<std::endl);
            RCLCPP_INFO_STREAM(get_logger(), "est range: "<< r_estimated <<std::endl);
            RCLCPP_INFO_STREAM(get_logger(), "measured bearing: "<< bearing <<std::endl);
            RCLCPP_INFO_STREAM(get_logger(), "est bearing: "<< b_estimated <<std::endl);

          // translate estimated measurement to x,y coordinates (r)
          double m_x_hat = state(1) + r_estimated * std::cos(
              state(0) + b_estimated);
          double m_y_hat = state(2) + r_estimated * std::sin(
            state(0) + b_estimated);

        
          double delta_x_hat = m_x_hat - state(1);
          double delta_y_hat = m_y_hat - state(2);

          double d_hat = std::pow(delta_x_hat, 2) + std::pow(delta_y_hat, 2);

          arma::vec z_t_hat = {
            r_estimated,
            b_estimated
          };

          // set up H_i
          arma::mat H_j = arma::mat(2, 3 + 2 * n_landmarks, arma::fill::zeros);

          H_j.submat(0, 0, 1, 2) = {{0.0, -delta_x_hat / std::sqrt(d_hat), -delta_y_hat / std::sqrt(
              d_hat)},
            {-1.0, delta_y_hat / d_hat, -delta_x_hat / d_hat},
          };

          H_j.submat(0, 3 + 2 * marker.id, 1, 3 + 2 * marker.id + 1) = {
            {delta_x_hat / std::sqrt(d_hat), delta_y_hat / std::sqrt(d_hat)},
            {-delta_y_hat / d_hat, delta_x_hat / d_hat}
          };

          // kalman gain
          arma::mat Ki = sigma_hat * H_j.t() * arma::inv(H_j * sigma_hat * H_j.t() + R);
          //   RCLCPP_INFO_STREAM(get_logger(), "Ki: "<< Ki <<std::endl);
          // update state with measurement error

          arma::vec z_diff = z_t - z_t_hat;
          z_diff(1) = turtlelib::normalize_angle(z_diff(1));

          state = state + Ki * (z_t - z_t_hat);
          //RCLCPP_INFO_STREAM(get_logger(), "z_err after: "<< z_t - z_t_hat <<std::endl);

          // normalize angle for consistency
          state(0) = turtlelib::normalize_angle(state(0));

          // recalculate sigma & update
          arma::mat sigma_t =
            (arma::mat(
              2 * n_landmarks + 3, 2 * n_landmarks + 3,
              arma::fill::eye) - Ki * H_j) * sigma_hat;
          sigma_hat = sigma_t;
        }
      }


    //   sigma_hat_t_minus_1 = sigma_hat_minus;

      // make the estimated state into a Transform
      turtlelib::Transform2D t_world_robot{{state(1), state(2)}, state(0)};

      // get the transform map->odom
      t_map_odom = t_world_robot * t_odom_robot.inv();

      // save this as the previous configuration
      filter_previous_configuration = t_world_robot;

      // publish markers
      visualization_msgs::msg::MarkerArray sensed_obstacles;

      for (size_t i = 0; i < fake_sensor_data_.markers.size(); i++) {
        // add the estimated marker to the marker array
        auto marker = fake_sensor_data_.markers.at(i);
        auto sensed_marker = create_obstacle(
          marker.scale.x, marker.scale.y, marker.scale.z,
          state(2 * marker.id + 3),
          state(
            2 * marker.id + 3 + 1),
          marker.pose.position.z, marker.id, visualization_msgs::msg::Marker::ADD, "map", "g",
          marker.header.stamp);

        sensed_obstacles.markers.insert(
          sensed_obstacles.markers.end(),
          sensed_marker);
      }
      sensed_obstacles_publisher_->publish(sensed_obstacles);

      fake_sensor_data_.markers.clear();
    }
    // broadcast the transform

    map_transform.header.stamp = time_now;
    map_transform.transform.translation.x = t_map_odom.translation().x;
    map_transform.transform.translation.y = t_map_odom.translation().y;
    map_transform.transform.rotation.x = 0.0; // will always be zero in planar rotations
    map_transform.transform.rotation.y = 0.0; // will always be zero in planar rotations
    map_transform.transform.rotation.z = std::sin(t_map_odom.rotation() / 2.0);
    map_transform.transform.rotation.w = std::cos(t_map_odom.rotation() / 2.0);

    tf_broadcaster_->sendTransform(map_transform);

  }

  visualization_msgs::msg::Marker create_obstacle(
    double scaleX, double scaleY, double scaleZ,
    double posX, double posY, double posZ, double id,
    int action = visualization_msgs::msg::Marker::ADD,
    std::string frame_id = "map", std::string color = "r",
    rclcpp::Time time_now = rclcpp::Time())
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    if (time_now.seconds() == 0) {
      marker.header.stamp = get_clock()->now();
    } else {
      marker.header.stamp = time_now;
    }

    marker.ns = "";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = action;
    marker.pose.position.x = posX;
    marker.pose.position.y = posY;
    marker.pose.position.z = posZ;
    marker.scale.x = scaleX;
    marker.scale.y = scaleY;
    marker.scale.z = scaleZ;
    if (color == "r") {
      marker.color.r = 1.0;
    } else if (color == "y") {
      marker.color.r = 1.0;
      marker.color.g = 0.87;
    } else if (color == "g") {
      marker.color.g = 1.0;
    }
    marker.color.a = 1.0;
    return marker;
  }

  void poseCallback(
    const std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control_interfaces::srv::InitialPose::Response>
  )
  {
    // restart the ddrive class
    ddrive = turtlelib::DiffDrive{wheel_track, wheel_radius};
    t_odom_robot = turtlelib::Transform2D{{request->x, request->y}, request->theta};
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
    auto time_now = get_clock()->now();
    // we should be using the names provided (name_wheel_left, name_Wheel_right)
    // to find out the indices of those in the vector
    auto transform = ddrive.FKin(msg.position.at(0), msg.position.at(1));
    if (!first_joints_cb) {
      auto dt = (time_now - time_last_joint_data).seconds();
      odom_msg.header.stamp = time_now;
      odom_msg.pose.pose.position.x = transform.translation().x;
      odom_msg.pose.pose.position.y = transform.translation().y;
      odom_msg.pose.pose.orientation.x = 0.0;   // will always be zero in planar rotations
      odom_msg.pose.pose.orientation.y = 0.0;   // will always be zero in planar rotations
      odom_msg.pose.pose.orientation.z = std::sin(transform.rotation() / 2.0);
      odom_msg.pose.pose.orientation.w = std::cos(transform.rotation() / 2.0);
      odom_msg.twist.twist.linear.x = (transform.translation().x - t_odom_robot.translation().x) /
        (dt);
      odom_msg.twist.twist.linear.y = (transform.translation().y - t_odom_robot.translation().y) /
        (dt);
      odom_msg.twist.twist.angular.z = (transform.rotation() - t_odom_robot.rotation()) / (dt);
      // pub_odom_->publish(odom_msg);

      odom_transform.header.stamp = time_now;
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
    time_last_joint_data = time_now;
    t_odom_robot = transform;
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
  geometry_msgs::msg::TransformStamped map_transform;
  rclcpp::Time time_last_joint_data;
  turtlelib::Transform2D t_odom_robot;
  rclcpp::Service<nuturtle_control_interfaces::srv::InitialPose>::SharedPtr pose_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path_msg;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr timer_path_;

  // filter stuff

  turtlelib::Transform2D t_map_odom{};
  turtlelib::Transform2D filter_previous_configuration{};
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_fake_sensor_;
  visualization_msgs::msg::MarkerArray fake_sensor_data_;
  // initialization
  int n_landmarks{20};
  double q_noise{1.0e6}; // 1e-3
  double r_noise{1.0e-2}; // 1e-4
  // equation 19
  arma::mat sigma_zero;
  arma::mat sigma_hat;
  arma::mat state = arma::vec(2 * n_landmarks + 3, arma::fill::zeros);
  arma::mat q_bar;
  arma::mat R;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensed_obstacles_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
