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
///     sensor_source (string): if sim: grabs sensor data frm simulation, if assoc, perform data association from lidar scans.
/// PUBLISHES:
///     odom (nav_msgs::msg::Odometry): turtlebot odometry
///     tf (tf2 transform broadcast): transform between odom frame and body frame
///     /red/path (nav_msgs::msg::Path): path followed by the robot in the map frame
///     green/obstacles (visualization_msgs::msg::MarkerArray): marker array containing the estimation obstacles
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
    declare_parameter("sensor_source", "sim");
    declare_parameter("rate", 100.0);
    declare_parameter("path_rate", 1.0);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_track = get_parameter("track_width").as_double();
    body_id = get_parameter("body_id").as_string();
    name_wheel_left = get_parameter("wheel_left").as_string();
    name_Wheel_right = get_parameter("wheel_right").as_string();
    odom_id = get_parameter("odom_id").as_string();
    auto sensor_source = get_parameter("sensor_source").as_string();

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

    if (sensor_source == "sim") {
      sub_fake_sensor_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "nusim/fake_sensor", sensor_qos_, std::bind(
          &Slam::fake_sensor_cb, this,
          std::placeholders::_1));
    } else if (sensor_source == "assoc") {
      sub_sensor_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "green/detected_obstacles", sensor_qos_, std::bind(
          &Slam::sensor_cb, this,
          std::placeholders::_1));
    }

    // covariance matrix
    arma::mat sigma_zero = arma::mat(2 * n_landmarks + 3, 2 * n_landmarks + 3, arma::fill::zeros);
    sigma_zero.submat(3, 3, 2 * n_landmarks + 2, 2 * n_landmarks + 2).eye();
    sigma_zero.submat(3, 3, 2 * n_landmarks + 2, 2 * n_landmarks + 2) *= 10e6;

    sigma = sigma_zero;

    // calculate Q bar (equation 22)
    q_bar = arma::mat(2 * n_landmarks + 3, 2 * n_landmarks + 3, arma::fill::zeros);
    q_bar.submat(0, 0, 2, 2) = arma::mat(3, 3, arma::fill::eye) * q_noise;

    // R
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

    auto current_configuration = t_map_odom * t_odom_robot;

    state(0) = turtlelib::normalize_angle(current_configuration.rotation());
    state(1) = current_configuration.translation().x;
    state(2) = current_configuration.translation().y;

    auto dx = current_configuration.translation().x - filter_previous_configuration.translation().x;
    auto dy = current_configuration.translation().y - filter_previous_configuration.translation().y;

    // eqns 9, 10
    arma::mat At = arma::mat(2 * n_landmarks + 3, 2 * n_landmarks + 3, arma::fill::eye);
    At(1, 0) = -dy;
    At(2, 0) = dx;

    sigma = At * sigma * At.t() + q_bar;

    // iterate over the measurements
    for (size_t i = 0; i < fake_sensor_data_.markers.size(); i++) {
      // grab the marker
      auto marker = fake_sensor_data_.markers.at(i);
      // if the marker is not in delete
      if (marker.action != visualization_msgs::msg::Marker::DELETE) {

        // eqs 11, 12
        auto meas_range =
          std::sqrt(std::pow(marker.pose.position.x, 2) + std::pow(marker.pose.position.y, 2));                   // the marker is in relative x,y already
        auto meas_bearing = std::atan2(marker.pose.position.y, marker.pose.position.x);

        // initialize the marker in the state if we haven't seen it
        if (state(3 + marker.id * 2) == 0.0 && state(3 + marker.id * 2 + 1) == 0.0) {
          state(3 + marker.id * 2) = state(1) + meas_range * std::cos(meas_bearing + state(0));
          state(3 + marker.id * 2 + 1) = state(2) + meas_range * std::sin(meas_bearing + state(0));
        }

        // estimated measurement eq 14
        auto est_range = std::sqrt(
          std::pow(state(3 + marker.id * 2) - state(1), 2) +
          std::pow(state(3 + marker.id * 2 + 1) - state(2), 2)
        );
        auto est_bearing =
          std::atan2(
          state(3 + marker.id * 2 + 1) - state(2),
          state(3 + marker.id * 2) - state(1)) - state(0);
        est_bearing = turtlelib::normalize_angle(est_bearing);

        // eq 25
        arma::vec z = {meas_range, meas_bearing};
        arma::vec z_hat = {est_range, est_bearing};

        // eqs 16, 17
        auto delta_x = state(3 + marker.id * 2) - state(1);
        auto delta_y = state(3 + marker.id * 2 + 1) - state(2);

        auto d = delta_x * delta_x + delta_y * delta_y;

        // eqn 18
        arma::mat Hj = arma::mat(2, 2 * n_landmarks + 3, arma::fill::zeros);
        Hj(0, 1) = -delta_x / std::sqrt(d);
        Hj(0, 2) = -delta_y / std::sqrt(d);
        Hj(1, 0) = -1.0;
        Hj(1, 1) = delta_y / d;
        Hj(1, 2) = -delta_x / d;
        Hj(0, 3 + 2 * marker.id) = delta_x / std::sqrt(d);
        Hj(0, 3 + 2 * marker.id + 1) = delta_y / std::sqrt(d);
        Hj(1, 3 + 2 * marker.id) = -delta_y / d;
        Hj(1, 3 + 2 * marker.id + 1) = delta_x / d;

        // eq 26
        arma::mat K = sigma * Hj.t() * arma::inv(Hj * sigma * Hj.t() + R);

        // eq 27
        arma::vec z_diff = z - z_hat;
        z_diff(1) = turtlelib::normalize_angle(z_diff(1));

        // RCLCPP_INFO_STREAM(get_logger(), "k * zdiff: "<< K*z_diff <<std::endl);
        //RCLCPP_INFO_STREAM(get_logger(), "z diff size: "<< z_diff <<std::endl);

        state = state + K * z_diff;

        // eq 28
        sigma =
          (arma::mat(2 * n_landmarks + 3, 2 * n_landmarks + 3, arma::fill::eye) - K * Hj) * sigma;

        state(0) = turtlelib::normalize_angle(state(0));

        // RCLCPP_INFO_STREAM(get_logger(), "state: "<< state <<std::endl);
      }
    }

    turtlelib::Transform2D filter_configuration =
      turtlelib::Transform2D{{state(1), state(2)}, state(0)};

    // get the transform map->odom
    t_map_odom = filter_configuration * t_odom_robot.inv();

    // broadcast the transform

    map_transform.header.stamp = msg.markers.at(0).header.stamp;
    map_transform.transform.translation.x = t_map_odom.translation().x;
    map_transform.transform.translation.y = t_map_odom.translation().y;
    map_transform.transform.rotation.x = 0.0; // will always be zero in planar rotations
    map_transform.transform.rotation.y = 0.0; // will always be zero in planar rotations
    map_transform.transform.rotation.z = std::sin(t_map_odom.rotation() / 2.0);
    map_transform.transform.rotation.w = std::cos(t_map_odom.rotation() / 2.0);

    tf_broadcaster_->sendTransform(map_transform);

    filter_previous_configuration = filter_configuration;


    // publish markers

    //   // publish markers
    visualization_msgs::msg::MarkerArray sensed_obstacles;

    for (size_t i = 0; i < msg.markers.size(); i++) {
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

  }

  void sensor_cb(const visualization_msgs::msg::MarkerArray & msg)
  {

    auto current_configuration = t_map_odom * t_odom_robot;

    state(0) = turtlelib::normalize_angle(current_configuration.rotation());
    state(1) = current_configuration.translation().x;
    state(2) = current_configuration.translation().y;

    auto dx = current_configuration.translation().x - filter_previous_configuration.translation().x;
    auto dy = current_configuration.translation().y - filter_previous_configuration.translation().y;

    // eqns 9, 10
    arma::mat At = arma::mat(2 * n_landmarks + 3, 2 * n_landmarks + 3, arma::fill::eye);
    At(1, 0) = -dy;
    At(2, 0) = dx;

    sigma = At * sigma * At.t() + q_bar;

    // for each measurement i
    for (size_t i = 0; i < msg.markers.size(); i++) {
      // grab the marker
      auto marker = msg.markers.at(i);

      // begin landmark association

      // eqs 11, 12
      auto meas_range =
        std::sqrt(std::pow(marker.pose.position.x, 2) + std::pow(marker.pose.position.y, 2));                 // the marker is in relative x,y already
      auto meas_bearing = std::atan2(marker.pose.position.y, marker.pose.position.x);

      // add the marker to the last position of the state (temporarily)

      state(3 + counter_obstacles * 2) = state(1) + meas_range * std::cos(meas_bearing + state(0));
      state(3 + counter_obstacles * 2 + 1) = state(2) + meas_range * std::sin(
        meas_bearing + state(
          0));

      counter_obstacles++;

      arma::vec mahanalobis_distances = arma::vec(counter_obstacles);

      // for each landmark k (including the recently added)
      for (size_t k = 0; k < counter_obstacles; k++) {
        //RCLCPP_INFO_STREAM(get_logger(), "for loop - k = " << k <<std::endl);
        // estimated measurement eq 14
        auto est_range = std::sqrt(
          std::pow(state(3 + k * 2) - state(1), 2) +
          std::pow(state(3 + k * 2 + 1) - state(2), 2)
        );
        auto est_bearing =
          std::atan2(state(3 + k * 2 + 1) - state(2), state(3 + k * 2) - state(1)) - state(0);
        est_bearing = turtlelib::normalize_angle(est_bearing);

        // eq 25
        arma::vec z = {meas_range, meas_bearing};
        arma::vec z_hat = {est_range, est_bearing};

        // eqs 16, 17
        auto delta_x = state(3 + k * 2) - state(1);
        auto delta_y = state(3 + k * 2 + 1) - state(2);

        auto d = delta_x * delta_x + delta_y * delta_y;

        // eqn 18
        arma::mat Hk = arma::mat(2, 2 * n_landmarks + 3, arma::fill::zeros);
        Hk(0, 1) = -delta_x / std::sqrt(d);
        Hk(0, 2) = -delta_y / std::sqrt(d);
        Hk(1, 0) = -1.0;
        Hk(1, 1) = delta_y / d;
        Hk(1, 2) = -delta_x / d;
        Hk(0, 3 + 2 * k) = delta_x / std::sqrt(d);
        Hk(0, 3 + 2 * k + 1) = delta_y / std::sqrt(d);
        Hk(1, 3 + 2 * k) = -delta_y / d;
        Hk(1, 3 + 2 * k + 1) = delta_x / d;

        //
        arma::mat psi = Hk * sigma * Hk.t() + R;

        //
        arma::vec z_diff = z - z_hat;
        z_diff(1) = turtlelib::normalize_angle(z_diff(1));

        arma::vec mah = z_diff.t() * arma::inv(psi) * z_diff;


        auto dist = mah(0);

        if (k == counter_obstacles - 1) {    // the newly added landmark
          dist = mah_threshold;
        }

        // RCLCPP_INFO_STREAM(get_logger(), "for loop - set mah distance" <<std::endl);
        mahanalobis_distances(k) = dist;
        // RCLCPP_INFO_STREAM(get_logger(), "for loop - after set mah distance" <<std::endl);

        // RCLCPP_INFO_STREAM(get_logger(), "state: "<< state <<std::endl);

      }

      auto min_index = arma::index_min(mahanalobis_distances);
      auto min = arma::min(mahanalobis_distances);

      if (min_index == counter_obstacles - 1 && min >= mah_threshold) {
        // then we have successfully added it as a new landmark
      } else if (min_index == counter_obstacles - 1 && min < mah_threshold) {
        // find the closest existing landmark
        mahanalobis_distances(min_index) = 1000.0;     // set to large value
        // find the minimum again
        min_index = arma::index_min(mahanalobis_distances);
        // its a landmark we already knew
        counter_obstacles--;
        // remove the changes in the state
        state(3 + counter_obstacles * 2) = 0.0;
        state(3 + counter_obstacles * 2 + 1) = 0.0;
      } else {
        // its a landmark we already knew
        counter_obstacles--;
        // remove the changes in the state
        state(3 + counter_obstacles * 2) = 0.0;
        state(3 + counter_obstacles * 2 + 1) = 0.0;

      }
      // end landmark association

      // estimated measurement eq 14
      auto est_range = std::sqrt(
        std::pow(state(3 + min_index * 2) - state(1), 2) +
        std::pow(state(3 + min_index * 2 + 1) - state(2), 2)
      );
      auto est_bearing = std::atan2(
        state(3 + min_index * 2 + 1) - state(2), state(
          3 + min_index * 2) - state(1)) - state(0);
      est_bearing = turtlelib::normalize_angle(est_bearing);

      // eq 25
      arma::vec z = {meas_range, meas_bearing};
      arma::vec z_hat = {est_range, est_bearing};

      // eqs 16, 17
      auto delta_x = state(3 + min_index * 2) - state(1);
      auto delta_y = state(3 + min_index * 2 + 1) - state(2);

      auto d = delta_x * delta_x + delta_y * delta_y;

      // eqn 18
      arma::mat Hj = arma::mat(2, 2 * n_landmarks + 3, arma::fill::zeros);
      Hj(0, 1) = -delta_x / std::sqrt(d);
      Hj(0, 2) = -delta_y / std::sqrt(d);
      Hj(1, 0) = -1.0;
      Hj(1, 1) = delta_y / d;
      Hj(1, 2) = -delta_x / d;
      Hj(0, 3 + 2 * min_index) = delta_x / std::sqrt(d);
      Hj(0, 3 + 2 * min_index + 1) = delta_y / std::sqrt(d);
      Hj(1, 3 + 2 * min_index) = -delta_y / d;
      Hj(1, 3 + 2 * min_index + 1) = delta_x / d;

      // eq 26
      arma::mat K = sigma * Hj.t() * arma::inv(Hj * sigma * Hj.t() + R);

      // eq 27
      arma::vec z_diff = z - z_hat;
      z_diff(1) = turtlelib::normalize_angle(z_diff(1));

      state = state + K * z_diff;

      // eq 28
      sigma =
        (arma::mat(2 * n_landmarks + 3, 2 * n_landmarks + 3, arma::fill::eye) - K * Hj) * sigma;

      state(0) = turtlelib::normalize_angle(state(0));
    }
    turtlelib::Transform2D filter_configuration =
      turtlelib::Transform2D{{state(1), state(2)}, state(0)};

    // get the transform map->odom
    t_map_odom = filter_configuration * t_odom_robot.inv();

    // broadcast the transform

    map_transform.header.stamp = msg.markers.at(0).header.stamp;
    map_transform.transform.translation.x = t_map_odom.translation().x;
    map_transform.transform.translation.y = t_map_odom.translation().y;
    map_transform.transform.rotation.x = 0.0; // will always be zero in planar rotations
    map_transform.transform.rotation.y = 0.0; // will always be zero in planar rotations
    map_transform.transform.rotation.z = std::sin(t_map_odom.rotation() / 2.0);
    map_transform.transform.rotation.w = std::cos(t_map_odom.rotation() / 2.0);

    tf_broadcaster_->sendTransform(map_transform);

    filter_previous_configuration = filter_configuration;


    // publish markers
    visualization_msgs::msg::MarkerArray sensed_obstacles;

    for (size_t i = 0; i < counter_obstacles; i++) {
      // add the estimated marker to the marker array
      auto sensed_marker = create_obstacle(
        0.1, 0.1, 0.25,
        state(2 * i + 3),
        state(
          2 * i + 3 + 1),
        0.125, i, visualization_msgs::msg::Marker::ADD, "map", "g",
        get_clock()->now());

      sensed_obstacles.markers.insert(
        sensed_obstacles.markers.end(),
        sensed_marker);
    }
    sensed_obstacles_publisher_->publish(sensed_obstacles);

  }

  void timerCallback()
  {
    pub_odom_->publish(odom_msg);
    tf_broadcaster_->sendTransform(odom_transform);
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
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_sensor_;
  visualization_msgs::msg::MarkerArray fake_sensor_data_;
  // initialization
  int n_landmarks{50};
  double q_noise{1.0e-2}; // 1e-3
  double r_noise{1.0e-2}; // 1e-4
  // equation 19

  unsigned int counter_obstacles = 0;
  double mah_threshold = 2.0;

  arma::mat sigma;
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
