#include <chrono>
#include <functional>
#include <memory>
#include <string>

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
    declare_parameter("obstacles/x", std::vector<double>{0.1});
    declare_parameter("obstacles/y", std::vector<double>{0.1});
    declare_parameter("obstacles/r", std::vector<double>());

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
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // create main loop timer
    timer_ = create_wall_timer(
      timerStep, std::bind(&NuSim::timer_callback, this));

    //
    auto markers_qos_ = rclcpp::SystemDefaultsQoS{};
    markers_qos_.transient_local();

    //
    walls_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/walls",
      markers_qos_);

    // publish arena walls
    publish_arena_walls();

    //
    obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      markers_qos_);
    publish_obstacles();

  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_;
    RCLCPP_INFO_STREAM(get_logger(), timestep_);
    timestep_publisher_->publish(message);

    turtle_pose_.header.stamp = get_clock()->now();
    tf_broadcaster_->sendTransform(turtle_pose_);

    timestep_++;
  }
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    set_initial_location();
    timestep_ = 0;
  }

  void teleport_callback(
    const std::shared_ptr<nusim_interfaces::srv::Teleport::Request> request,
    std::shared_ptr<nusim_interfaces::srv::Teleport::Response>)
  {
    turtle_pose_.transform.translation.x = request->x;
    turtle_pose_.transform.translation.y = request->y;

    tf2::Quaternion q;
    q.setRPY(0, 0, request->theta);
    turtle_pose_.transform.rotation.x = q.x();
    turtle_pose_.transform.rotation.y = q.y();
    turtle_pose_.transform.rotation.z = q.z();
    turtle_pose_.transform.rotation.w = q.w();
  }

  void set_initial_location()
  {
    turtle_pose_.transform.translation.x = get_parameter("x0").as_double();
    turtle_pose_.transform.translation.y = get_parameter("y0").as_double();
    tf2::Quaternion q;
    q.setRPY(0, 0, get_parameter("theta0").as_double());
    turtle_pose_.transform.rotation.x = q.x();
    turtle_pose_.transform.rotation.y = q.y();
    turtle_pose_.transform.rotation.z = q.z();
    turtle_pose_.transform.rotation.w = q.w();
  }

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

    walls_publisher_->publish(marker_array);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim_interfaces::srv::Teleport>::SharedPtr teleport_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped turtle_pose_;
  u_int64_t timestep_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
