/// \file
/// \brief Landmark detection form laser scan.
///
/// PARAMETERS:
///     sensor_topic (string): name of the laser scan topic
/// PUBLISHES:
///     green/detected_obstacles (visualization_msgs::msg::MarkerArray): marker array containing the detected obstacles
/// SUBSCRIBES:
///     sensor_topic (sensor_msgs::msg::LaserScan): laser scans

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/qos.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "turtlelib/se2d.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlelib/landmark_detection.hpp"

using namespace std::chrono_literals;


class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    declare_parameter("sensor_topic", "nusim/scan");

    auto sensor_topic = get_parameter("sensor_topic").as_string();

    // create publishers and subscribers

    auto laser_qos_ = rclcpp::SystemDefaultsQoS{};
    laser_qos_.best_effort();

    sub_laser_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      sensor_topic, laser_qos_, std::bind(&Landmarks::laserCallback, this, std::placeholders::_1));

    // create QoS for markers publishers
    auto markers_qos_ = rclcpp::SystemDefaultsQoS{};
    markers_qos_.transient_local();

    sensed_obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "green/detected_obstacles",
      markers_qos_);


  }

private:
  

  std::vector<std::vector<turtlelib::Point2D>> getClusters(const sensor_msgs::msg::LaserScan & msg)
  {
    // translate all readings into x,y coordinates
    // add them to clusters
    auto prev_point = turtlelib::Point2D{};

    std::vector<std::vector<turtlelib::Point2D>> clusters = {};
    std::vector<turtlelib::Point2D> current_cluster = {};
    for (size_t i = 0; i < msg.ranges.size(); i++) {
      double current_reading_angle = turtlelib::normalize_angle(
        static_cast<double>(i) * msg.angle_increment) + msg.angle_min;
      double x = msg.ranges.at(i) * std::cos(current_reading_angle) - 0.032;
      double y = msg.ranges.at(i) * std::sin(current_reading_angle);

      auto curr_point = turtlelib::Point2D{x, y};

      if (i == 0) {  // first reading in scan
        // just add this point to a cluster
        current_cluster.push_back(curr_point);
      } else {
        if (turtlelib::distance(curr_point, prev_point) <= distance_clustering_threshold) {
          // then they are on the same cluster
          current_cluster.push_back(curr_point);
        } else {
          // push current cluster into clusters vector
          clusters.push_back(current_cluster);
          // add point to new cluster
          current_cluster = {};
        }
      }
      prev_point = curr_point;
    }

    // now we're gonna check if the last cluster and the first cluster
    // are the same one, we're gonna grab the last point of the last cluster
    // and measure the distance to the first point of the first cluster
    auto first_point = clusters.at(0).at(0);
    auto last_point = prev_point;
    if (turtlelib::distance(first_point, last_point) <= distance_clustering_threshold) {
      // merge the clusters
      for (size_t i = 0; i < current_cluster.size(); i++) {
        clusters.at(0).push_back(current_cluster.at(i));     // we just add all the point from the last cluster into the first cluster
      }
    } else {
      clusters.push_back(current_cluster);      // the last cluster is indeed a separate cluster, add it to the list of clusters
    }

    return clusters;
  }


  void laserCallback(const sensor_msgs::msg::LaserScan & msg)
  {
    // get the clusters
    auto clusters = getClusters(msg);

    // make the clusters into matrices
    // also filter clusters that are very large (we assume our clusters of interest will have < 25 points)
    // or clusters that are too small (we want at least 4 points to fit the circles)

    std::vector<arma::mat> clusters_mat_list = {};

    for (size_t i = 0; i < clusters.size(); i++) { // for each cluster
      // check the size of the cluster & if the points form a circle
      if (clusters.at(i).size() > 3 && clusters.at(i).size() < 40 && turtlelib::checkCircle(clusters.at(i))) {
        // its a circle!
        // then we will convert it to a matrix
        arma::mat cluster = arma::mat(clusters.at(i).size(), 2, arma::fill::zeros);
        for (size_t j = 0; j < clusters.at(i).size(); j++) {
          cluster.row(j) = {clusters.at(i).at(j).x, clusters.at(i).at(j).y};
        }
        clusters_mat_list.push_back(cluster);
        // RCLCPP_INFO_STREAM(get_logger(), cluster <<std::endl);
      }
    }

    auto frame_id = msg.header.frame_id;
    visualization_msgs::msg::MarkerArray clusters_markers;

    for (size_t i = 0; i < clusters_mat_list.size(); i++) { // for each cluster

      double c_x;
      double c_y;
      double r;
      std::tie(c_x, c_y, r) = turtlelib::fitCircle(clusters_mat_list.at(i));
        if (r < 0.2 && std::sqrt(std::pow(c_x,2)+std::pow(c_y,2)) < 2.0){ // filter obstacles with radius greater than 0.2, and only keep those closer than 2m
      auto marker = create_obstacle(
        2.0 * r, 2.0 * r, 0.2, c_x, c_y, 0.0, i,
        visualization_msgs::msg::Marker::ADD, frame_id);
      clusters_markers.markers.push_back(marker);
        }
    }

    if (clusters_markers.markers.size() > 0){
        sensed_obstacles_publisher_->publish(clusters_markers);
    }
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

  double distance_clustering_threshold = 0.2;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensed_obstacles_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
